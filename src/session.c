/*
 * Session ownership.
 *
 * One thread owns a USB audio session for its whole life: open the two pipes,
 * run the setup control transfers in order, start the transport, and later tear
 * all of it back down.  USBD callbacks and the system-event handler do nothing
 * but post a command here and return.
 *
 * One owner is the whole design: steps run in order on one thread, so no step
 * has to prove the session is still the one it started on.
 *
 * Two hazards survive that, and only two:
 *
 *   - A control transfer that times out leaves its completion in flight.  Each
 *     carries a token and drops itself once the token moves on.
 *   - USBD offers a device exactly once, so an attach arriving during a
 *     teardown is queued behind it, never refused.
 */

#include "session.h"

#include "audio_tap.h"
#include "log.h"
#include "stream.h"
#include "uac1.h"

#include <stddef.h>
#include <stdint.h>

#include <psp2kern/kernel/cpu/cache.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv] "

/* Fixed transport: 48 kHz, stereo, signed 16-bit PCM. */
#define TARGET_RATE 48000u

/* UAC1 endpoint sampling-frequency control. */
#define UAC_SET_CUR 1u
#define UAC_EP_SAMPLING_FREQ 0x0100u
#define UAC_RATE_BYTES 3

#define SESSION_WORK_BIT 0x00000001u
#define CONTROL_DONE_BIT 0x00000001u

/* One control transfer on a live device; short because failure is recoverable. */
#define CONTROL_TIMEOUT_US 250000u
/* Must exceed one full teardown: see FEEDER_STOP_TIMEOUT_US in stream.c. */
#define SESSION_JOIN_TIMEOUT_US 6000000u
#define SESSION_PRIORITY 0x40
#define SESSION_STACK 0x2000u

/*
 * The live session.  Written only by the session thread; device_id is the one
 * field a callback reads, to refuse a second device.
 */
static struct {
	int device_id;
	int control_pipe;
	int stream_pipe;
	int interface_selected;
	Uac1Stream stream;
} live = {
	.device_id = -1,
	.control_pipe = -1,
	.stream_pipe = -1,
};

/* Commands.  Posted by callbacks, consumed by the session thread. */
static int pending_start = -1;
static int pending_stop;
static int pending_detached;
static int accepting;
static int exiting;

static SceUID session_event = -1;
static SceUID session_thread = -1;
static int session_started;

static SceUID control_event = -1;
static uint32_t control_token;
static int32_t control_result;
static int32_t control_count;

/* Its own cache line: the host controller reads it during the rate transfer. */
static uint8_t rate_buffer[64] __attribute__((aligned(64)));

static void signal_work(void)
{
	if (session_event >= 0)
		(void)ksceKernelSetEventFlag(session_event, SESSION_WORK_BIT);
}

int session_device(void)
{
	return __atomic_load_n(&live.device_id, __ATOMIC_ACQUIRE);
}

/*
 * Set while the session thread is inside setup.  Checked between steps so a
 * detach or a suspend does not have to wait for a device that is already gone
 * to answer three more control transfers.
 */
static int cancelled(void)
{
	return __atomic_load_n(&pending_stop, __ATOMIC_ACQUIRE) != 0 ||
	       __atomic_load_n(&exiting, __ATOMIC_ACQUIRE) != 0;
}

static void control_done(int32_t result, int32_t count, void *arg)
{
	/* control_wait() moves the token on before giving up, so a completion that
	 * arrives after a timeout drops itself instead of reporting into the next. */
	if ((uint32_t)(uintptr_t)arg !=
	    __atomic_load_n(&control_token, __ATOMIC_ACQUIRE))
		return;
	__atomic_store_n(&control_result, result, __ATOMIC_RELAXED);
	__atomic_store_n(&control_count, count, __ATOMIC_RELAXED);
	if (control_event >= 0)
		(void)ksceKernelSetEventFlag(control_event, CONTROL_DONE_BIT);
}

/*
 * Claim the next token, then clear the flag.  In that order: bumping first
 * retires any completion still in flight, so the clear cannot race one setting
 * the bit back while its token is still current.
 */
static void *control_begin(void)
{
	uint32_t token = __atomic_add_fetch(&control_token, 1u, __ATOMIC_ACQ_REL);

	(void)ksceKernelClearEventFlag(control_event, 0);
	return (void *)(uintptr_t)token;
}

static int control_wait(int submit, const char *what)
{
	SceUInt timeout = CONTROL_TIMEOUT_US;
	uint32_t matched;
	int result;

	(void)what; /* uac_log may compile out. */
	if (submit < 0) {
		/* No callback is coming, so the token needs no retiring. */
		uac_log(LOG_PREFIX "%s submit: 0x%08x\n", what, submit);
		return submit;
	}
	if (ksceKernelWaitEventFlag(control_event, CONTROL_DONE_BIT,
		SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, &timeout) < 0) {
		(void)__atomic_add_fetch(&control_token, 1u, __ATOMIC_ACQ_REL);
		uac_log(LOG_PREFIX "%s: timed out\n", what);
		return SESSION_ERROR_CONTROL_TIMEOUT;
	}
	result = __atomic_load_n(&control_result, __ATOMIC_RELAXED);
	uac_log(LOG_PREFIX "%s: 0x%08x\n", what, result);
	return result;
}

static int set_configuration(uint8_t configuration)
{
	void *token = control_begin();

	return control_wait(ksceUsbdSetConfiguration(live.control_pipe,
		configuration, control_done, token), "set configuration");
}

static int set_interface(uint8_t number, uint8_t alternate, const char *what)
{
	void *token = control_begin();

	return control_wait(ksceUsbdSetInterface(live.control_pipe, number,
		alternate, control_done, token), what);
}

static int set_sample_rate(void)
{
	SceUsbdDeviceRequest request;
	void *token;
	int result;

	rate_buffer[0] = (uint8_t)TARGET_RATE;
	rate_buffer[1] = (uint8_t)(TARGET_RATE >> 8);
	rate_buffer[2] = (uint8_t)(TARGET_RATE >> 16);
	ksceKernelDcacheCleanRange(rate_buffer, sizeof(rate_buffer));

	request.bmRequestType = SCE_USBD_REQTYPE_DIR_TO_DEVICE |
		SCE_USBD_REQTYPE_TYPE_CLASS | SCE_USBD_REQTYPE_RECIP_ENDPOINT;
	request.bRequest = UAC_SET_CUR;
	request.wValue = UAC_EP_SAMPLING_FREQ;
	request.wIndex = live.stream.endpoint_address;
	request.wLength = UAC_RATE_BYTES;

	token = control_begin();
	result = control_wait(ksceUsbdControlTransfer(live.control_pipe, &request,
		rate_buffer, control_done, token), "set fixed 48 kHz");
	if (result >= 0 &&
	    __atomic_load_n(&control_count, __ATOMIC_RELAXED) != UAC_RATE_BYTES)
		return -1;
	return result;
}

/*
 * Unwind whatever open_session() built, in reverse.  Safe on a partial session
 * and safe to call twice; every resource is checked first.  uac_stream_stop()
 * can take the better part of a second, which is why this is not on a callback.
 */
static void close_session(void)
{
	int detached = __atomic_exchange_n(&pending_detached, 0, __ATOMIC_ACQ_REL);
	int result;

	(void)result; /* uac_log may compile out. */
	if (live.device_id < 0 && live.control_pipe < 0 && live.stream_pipe < 0)
		return;

	uac_stream_stop();
	/* After the transport has stopped, so nothing still reads captured PCM. */
	result = audio_tap_end();
	if (result < 0)
		uac_log(LOG_PREFIX "virtual Sony DataSend stop failed: 0x%08x\n",
			result);

	/*
	 * Told to the transport whether or not USBD agreed to close it: the
	 * generation bump inside is what makes a late completion harmless, and
	 * holding a pipe we could not close fixes nothing.
	 */
	if (live.stream_pipe >= 0) {
		result = ksceUsbdClosePipe(live.stream_pipe);
		uac_log(LOG_PREFIX "stream pipe close: 0x%08x\n", result);
		uac_stream_pipe_closed(live.stream_pipe);
		live.stream_pipe = -1;
	}
	/*
	 * UAC1 reserves alternate setting 0 as the zero-bandwidth setting, and
	 * selecting it is how the host says no audio is flowing.  Closing the pipe
	 * is host-side only: the device never hears about it and keeps its clock
	 * domain locked to USB, which on an external DAC shows up as the clock
	 * source never falling back to internal.  Pointless once the device is
	 * physically gone, hence the detach check.
	 */
	if (live.interface_selected && !detached)
		(void)set_interface(live.stream.interface_number, 0, "select alt 0");
	if (live.control_pipe >= 0) {
		result = ksceUsbdClosePipe(live.control_pipe);
		uac_log(LOG_PREFIX "control pipe close: 0x%08x\n", result);
		live.control_pipe = -1;
	}

	live.interface_selected = 0;
	__atomic_store_n(&live.device_id, -1, __ATOMIC_RELEASE);
}

/*
 * Bring a session up.  Every failure posts a stop to ourselves and returns, so
 * one teardown path serves all of them.
 */
static void open_session(int device_id)
{
	Uac1Stream stream;
	int result;

	if (!__atomic_load_n(&accepting, __ATOMIC_ACQUIRE) || cancelled())
		return;
	if (live.device_id >= 0) {
		uac_log(LOG_PREFIX "attach rejected: device %d already active\n",
			live.device_id);
		return;
	}
	/* Probe and attach are separate callbacks, so rediscover the stream. */
	if (!uac1_find_stream(device_id, &stream)) {
		uac_log(LOG_PREFIX "stream rediscovery failed: device %d\n",
			device_id);
		return;
	}

	live.stream = stream;
	live.interface_selected = 0;
	__atomic_store_n(&live.device_id, device_id, __ATOMIC_RELEASE);

	live.control_pipe = ksceUsbdOpenPipe(device_id, NULL);
	uac_log(LOG_PREFIX "control pipe open: 0x%08x\n", live.control_pipe);
	if (live.control_pipe < 0 || cancelled())
		goto fail;

	live.stream_pipe = ksceUsbdOpenPipe(device_id, live.stream.endpoint);
	uac_log(LOG_PREFIX "stream pipe open: 0x%08x\n", live.stream_pipe);
	if (live.stream_pipe < 0 || cancelled())
		goto fail;

	if (set_configuration(live.stream.configuration) < 0 || cancelled())
		goto fail;

	if (set_interface(live.stream.interface_number,
		live.stream.alternate_setting, "select stream") < 0)
		goto fail;
	/* Set before the rate step: a failure past here still owes the device an
	 * alt 0 on the way out. */
	live.interface_selected = 1;
	if (cancelled())
		goto fail;

	if ((live.stream.frequency_control && set_sample_rate() < 0) || cancelled())
		goto fail;

	result = uac_stream_start(live.stream_pipe);
	if (result < 0) {
		uac_log(LOG_PREFIX "stream start failed: 0x%08x\n", result);
		goto fail;
	}

	/*
	 * Route last.  The endpoint is live from here on, and AVConfig takes ~400 ms
	 * to hand over, so the transport must already be running to cover that with
	 * silence.  It also keeps the route out of every failure path above.
	 */
	result = audio_tap_begin();
	if (result < 0) {
		uac_log(LOG_PREFIX "virtual Sony DataSend start failed: 0x%08x\n",
			result);
		goto fail;
	}
	return;

fail:
	__atomic_store_n(&pending_stop, 1, __ATOMIC_RELEASE);
	signal_work();
}

static int session_worker(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	for (;;) {
		uint32_t matched;
		int device;

		if (__atomic_load_n(&exiting, __ATOMIC_ACQUIRE)) {
			/* One last unwind, so shutdown never leaves a session
			 * holding the route or a pipe. */
			(void)__atomic_exchange_n(&pending_stop, 0, __ATOMIC_ACQ_REL);
			close_session();
			return 0;
		}
		if (__atomic_exchange_n(&pending_stop, 0, __ATOMIC_ACQ_REL)) {
			close_session();
			continue;
		}
		/* Stop before start, always: a queued attach is meant to run after
		 * the teardown it arrived behind, never instead of it. */
		device = __atomic_exchange_n(&pending_start, -1, __ATOMIC_ACQ_REL);
		if (device >= 0) {
			open_session(device);
			continue;
		}
		if (ksceKernelWaitEventFlag(session_event, SESSION_WORK_BIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, NULL) < 0)
			return -1;
	}
}

int session_start(int device_id)
{
	int expected = -1;

	if (device_id < 0 || !__atomic_load_n(&accepting, __ATOMIC_ACQUIRE))
		return 0;
	/*
	 * Refuse a second device, but only when the one we hold is staying.  If a
	 * teardown is already queued this attach belongs behind it -- USBD offers a
	 * device exactly once, so refusing here would lose it until a replug.
	 */
	if (__atomic_load_n(&live.device_id, __ATOMIC_ACQUIRE) >= 0 &&
	    !__atomic_load_n(&pending_stop, __ATOMIC_ACQUIRE)) {
		uac_log(LOG_PREFIX "attach rejected: device %d already active\n",
			__atomic_load_n(&live.device_id, __ATOMIC_ACQUIRE));
		return 0;
	}
	if (!__atomic_compare_exchange_n(&pending_start, &expected, device_id, 0,
		__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		uac_log(LOG_PREFIX "attach rejected: device %d already queued\n",
			expected);
		return 0;
	}
	signal_work();
	return 1;
}

void session_stop(const char *reason)
{
	(void)reason; /* uac_log may compile out. */
	if (__atomic_load_n(&live.device_id, __ATOMIC_ACQUIRE) < 0 &&
	    __atomic_load_n(&pending_start, __ATOMIC_ACQUIRE) < 0)
		return;
	if (!__atomic_exchange_n(&pending_stop, 1, __ATOMIC_ACQ_REL))
		uac_log(LOG_PREFIX "session stop: %s\n",
			reason != NULL ? reason : "unspecified");
	signal_work();
}

void session_stop_detached(int device_id)
{
	int queued = __atomic_load_n(&pending_start, __ATOMIC_ACQUIRE);

	/* Drop a queued attach for a device that has since gone away. */
	if (queued == device_id)
		(void)__atomic_compare_exchange_n(&pending_start, &queued, -1, 0,
			__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
	if (__atomic_load_n(&live.device_id, __ATOMIC_ACQUIRE) != device_id)
		return;
	/* Skips alt 0 in close_session(): there is nothing left to tell. */
	__atomic_store_n(&pending_detached, 1, __ATOMIC_RELEASE);
	session_stop("device detached");
}

void session_accept(void)
{
	__atomic_store_n(&accepting, 1, __ATOMIC_RELEASE);
	signal_work();
}

void session_quiesce(void)
{
	__atomic_store_n(&accepting, 0, __ATOMIC_RELEASE);
	session_stop("lifecycle quiesce");
}

int session_init(void)
{
	int result;

	if (session_event < 0) {
		result = ksceKernelCreateEventFlag("uac_session", 0, 0, NULL);
		if (result < 0)
			return result;
		session_event = result;
	}
	if (control_event < 0) {
		result = ksceKernelCreateEventFlag("uac_control", 0, 0, NULL);
		if (result < 0)
			goto fail;
		control_event = result;
	}
	if (session_thread >= 0 && session_started) {
		session_accept();
		return 0;
	}
	if (session_thread >= 0) {
		result = ksceKernelDeleteThread(session_thread);
		if (result < 0)
			return result;
		session_thread = -1;
	}
	__atomic_store_n(&exiting, 0, __ATOMIC_RELEASE);
	result = ksceKernelCreateThread("uac_session", session_worker,
		SESSION_PRIORITY, SESSION_STACK, 0, 0, NULL);
	if (result < 0)
		goto fail;
	session_thread = result;
	result = ksceKernelStartThread(session_thread, 0, NULL);
	if (result < 0) {
		if (ksceKernelDeleteThread(session_thread) >= 0)
			session_thread = -1;
		else
			return result;
		goto fail;
	}
	session_started = 1;
	session_accept();
	return 0;

fail:
	if (control_event >= 0 && ksceKernelDeleteEventFlag(control_event) >= 0)
		control_event = -1;
	if (session_event >= 0 && ksceKernelDeleteEventFlag(session_event) >= 0)
		session_event = -1;
	return result;
}

/* Call only after the USB driver is unregistered, so nothing can post again. */
int session_shutdown(void)
{
	SceUInt timeout = SESSION_JOIN_TIMEOUT_US;
	int status;
	int result;

	__atomic_store_n(&accepting, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&pending_start, -1, __ATOMIC_RELEASE);

	if (session_thread >= 0) {
		if (session_started) {
			__atomic_store_n(&exiting, 1, __ATOMIC_RELEASE);
			signal_work();
			result = ksceKernelWaitThreadEnd(session_thread, &status,
				&timeout);
			if (result < 0)
				return result;
			session_started = 0;
		}
		result = ksceKernelDeleteThread(session_thread);
		if (result < 0)
			return result;
		session_thread = -1;
	}
	/* Nothing can be open without a worker: open_session only runs on one. */
	if (live.device_id >= 0 || live.control_pipe >= 0 || live.stream_pipe >= 0)
		return -1;

	if (control_event >= 0) {
		result = ksceKernelDeleteEventFlag(control_event);
		if (result < 0)
			return result;
		control_event = -1;
	}
	if (session_event >= 0) {
		result = ksceKernelDeleteEventFlag(session_event);
		if (result < 0)
			return result;
		session_event = -1;
	}
	return 0;
}
