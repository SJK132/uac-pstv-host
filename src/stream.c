#include "stream.h"
#include "log.h"

#include <psp2kern/usbd.h>
#include <psp2kern/kernel/cpu/cache.h>
#include <psp2kern/kernel/threadmgr.h>
#include <string.h>

#define LOG_PREFIX "[uac-pstv-boot] "

/* One UAC1 OUT packet: 48 stereo 16-bit frames, 1 ms at 48 kHz. */
#define STREAM_BUFFER_BYTES 192

/* uac_stream_stop waits this long for an in-flight completion callback to
 * retire before the caller closes the pipe underneath it. */
#define STREAM_STOP_POLL_US 1000
#define STREAM_STOP_POLLS 20
#define CALLBACK_DRAIN_POLLS 20

typedef struct {
	int16_t buffer[STREAM_BUFFER_BYTES / sizeof(int16_t)]
		__attribute__((aligned(64)));
	ksceUsbdIsochTransfer transfer __attribute__((aligned(64)));
} StreamRequest;

/* SceUsbd accepts one isochronous request per pipe. */
static StreamRequest stream_request __attribute__((aligned(64)));
static int stream_running;
static int stream_pipe;
static uint32_t callback_active;
#ifdef UAC_PSTV_ENABLE_LOGGING
static int stream_error;
#endif
static UacCoreFillCallback audio_fill;
static UacCoreStateCallback audio_state;
static uint32_t audio_callback_active;
static int audio_callbacks_enabled;

static void fill_audio(int16_t *output)
{
	UacCoreFillCallback fill;

	memset(output, 0, STREAM_BUFFER_BYTES);
	if (!__atomic_load_n(&audio_callbacks_enabled, __ATOMIC_ACQUIRE))
		return;
	__atomic_fetch_add(&audio_callback_active, 1, __ATOMIC_ACQUIRE);
	if (__atomic_load_n(&audio_callbacks_enabled, __ATOMIC_ACQUIRE)) {
		fill = __atomic_load_n(&audio_fill, __ATOMIC_ACQUIRE);
		if (fill != NULL)
			fill(output);
	}
	__atomic_fetch_sub(&audio_callback_active, 1, __ATOMIC_RELEASE);
}

static void notify_audio_state(int running)
{
	UacCoreStateCallback state;

	if (!__atomic_load_n(&audio_callbacks_enabled, __ATOMIC_ACQUIRE))
		return;
	__atomic_fetch_add(&audio_callback_active, 1, __ATOMIC_ACQUIRE);
	if (__atomic_load_n(&audio_callbacks_enabled, __ATOMIC_ACQUIRE)) {
		state = __atomic_load_n(&audio_state, __ATOMIC_ACQUIRE);
		if (state != NULL)
			state(running);
	}
	__atomic_fetch_sub(&audio_callback_active, 1, __ATOMIC_RELEASE);
}

int uac_core_set_audio_callbacks(UacCoreFillCallback fill,
	UacCoreStateCallback state)
{
	int wait;
	UacCoreStateCallback old_state;

	if ((fill == NULL) != (state == NULL))
		return -1;
	if (fill != NULL) {
		if (__atomic_load_n(&audio_callbacks_enabled, __ATOMIC_ACQUIRE))
			return -1;
		__atomic_store_n(&audio_fill, fill, __ATOMIC_RELEASE);
		__atomic_store_n(&audio_state, state, __ATOMIC_RELEASE);
		__atomic_store_n(&audio_callbacks_enabled, 1, __ATOMIC_RELEASE);
		if (__atomic_load_n(&stream_running, __ATOMIC_ACQUIRE)) {
			notify_audio_state(1);
			/* If stop raced the notification, it normally sends state 0.
			 * This recheck also covers stop just before callbacks enabled. */
			if (!__atomic_load_n(&stream_running, __ATOMIC_ACQUIRE))
				notify_audio_state(0);
		}
		return 0;
	}

	old_state = __atomic_load_n(&audio_state, __ATOMIC_ACQUIRE);
	__atomic_store_n(&audio_callbacks_enabled, 0, __ATOMIC_RELEASE);
	for (wait = 0; wait < CALLBACK_DRAIN_POLLS &&
		__atomic_load_n(&audio_callback_active, __ATOMIC_ACQUIRE) != 0;
		++wait)
		ksceKernelDelayThread(STREAM_STOP_POLL_US);
	if (__atomic_load_n(&audio_callback_active, __ATOMIC_ACQUIRE) != 0)
		return -1;
	if (old_state != NULL)
		old_state(0);
	__atomic_store_n(&audio_fill, NULL, __ATOMIC_RELEASE);
	__atomic_store_n(&audio_state, NULL, __ATOMIC_RELEASE);
	return 0;
}

static void prepare_request(StreamRequest *request)
{
	request->transfer.packets[0].len = STREAM_BUFFER_BYTES;
	request->transfer.packets[0].PSW = 0;
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *request,
	void *arg)
{
	StreamRequest *slot = (StreamRequest *)arg;
	int submit;
	(void)request;

	__atomic_fetch_add(&callback_active, 1, __ATOMIC_ACQUIRE);
	if (result < 0) {
#ifdef UAC_PSTV_ENABLE_LOGGING
		__atomic_store_n(&stream_error, result, __ATOMIC_RELEASE);
#endif
		__atomic_store_n(&stream_running, 0, __ATOMIC_RELEASE);
		notify_audio_state(0);
		goto done;
	}
	if (!__atomic_load_n(&stream_running, __ATOMIC_ACQUIRE))
		goto done;
	prepare_request(slot);
	fill_audio(slot->buffer);
	ksceKernelDcacheCleanRange(slot->buffer, STREAM_BUFFER_BYTES);
	if (!__atomic_load_n(&stream_running, __ATOMIC_ACQUIRE))
		goto done;
	/* Frame zero is rejected; frame one schedules the next request. */
	slot->transfer.relative_start_frame = 1;
	submit = ksceUsbdIsochronousTransfer(
		__atomic_load_n(&stream_pipe, __ATOMIC_RELAXED), &slot->transfer,
		transfer_done, slot);
	if (submit < 0) {
#ifdef UAC_PSTV_ENABLE_LOGGING
		__atomic_store_n(&stream_error, submit, __ATOMIC_RELEASE);
#endif
		__atomic_store_n(&stream_running, 0, __ATOMIC_RELEASE);
		notify_audio_state(0);
	}
done:
	__atomic_fetch_sub(&callback_active, 1, __ATOMIC_RELEASE);
}

int uac_stream_start(int pipe_id, uint16_t packet_bytes, uint8_t speed,
	uint8_t interval)
{
	int result;

	if (__atomic_load_n(&stream_running, __ATOMIC_ACQUIRE) ||
		packet_bytes != STREAM_BUFFER_BYTES)
		return -1;
	if (speed == SCE_USBD_DEVICE_SPEED_HS) {
		if (interval != 4)
			return -1;
	} else if (speed != SCE_USBD_DEVICE_SPEED_FS || interval != 1)
		return -1;

	__atomic_store_n(&stream_pipe, pipe_id, __ATOMIC_RELAXED);
#ifdef UAC_PSTV_ENABLE_LOGGING
	__atomic_store_n(&stream_error, 0, __ATOMIC_RELEASE);
#endif
	__atomic_store_n(&stream_running, 1, __ATOMIC_RELEASE);
	notify_audio_state(1);
	memset(&stream_request, 0, sizeof(stream_request));
	stream_request.transfer.buffer_base = stream_request.buffer;
	stream_request.transfer.relative_start_frame = 1;
	/* This field is the OUT byte count, despite VitaSDK's guessed name. */
	stream_request.transfer.num_packets = STREAM_BUFFER_BYTES;
	prepare_request(&stream_request);
	fill_audio(stream_request.buffer);
	ksceKernelDcacheCleanRange(stream_request.buffer, STREAM_BUFFER_BYTES);
	result = ksceUsbdIsochronousTransfer(pipe_id, &stream_request.transfer,
		transfer_done, &stream_request);
	if (result < 0) {
		__atomic_store_n(&stream_running, 0, __ATOMIC_RELEASE);
		notify_audio_state(0);
		uac_log(LOG_PREFIX "stream queue failed: 0x%08x\n", result);
		return result;
	}
	uac_log(LOG_PREFIX "stream queued: %d-byte OUT request\n",
		STREAM_BUFFER_BYTES);
	return 0;
}

void uac_stream_stop(void)
{
	int wait;
#ifdef UAC_PSTV_ENABLE_LOGGING
	int error;
#endif
	int was_running;

	was_running = __atomic_exchange_n(&stream_running, 0,
		__ATOMIC_ACQ_REL);
	if (was_running)
		notify_audio_state(0);
	for (wait = 0; wait < STREAM_STOP_POLLS &&
		__atomic_load_n(&callback_active, __ATOMIC_ACQUIRE) != 0; ++wait)
		ksceKernelDelayThread(STREAM_STOP_POLL_US);
	/* The caller closes the pipe next, so a callback still running here
	 * would be touching a pipe that is about to go away. */
	if (__atomic_load_n(&callback_active, __ATOMIC_ACQUIRE) != 0)
		uac_log(LOG_PREFIX "stream stop: callback still active after %d ms\n",
			(STREAM_STOP_POLLS * STREAM_STOP_POLL_US) / 1000);
#ifdef UAC_PSTV_ENABLE_LOGGING
	error = __atomic_exchange_n(&stream_error, 0, __ATOMIC_ACQ_REL);
	if (error < 0)
		uac_log(LOG_PREFIX "stream stopped after error: 0x%08x\n", error);
#endif
}
