/*
 * AVConfig route ownership.
 *
 * Sony's audio path normally hands decoded PCM to the HDMI transport.  For the
 * duration of one USB session this file takes that route over: it hooks the two
 * AVConfig entry points that start and stop the hardware DataSend worker, flips
 * the route word to RAM output, and runs a stand-in worker that passes each
 * 480-frame page to stream.c instead.
 *
 * The whole file lives between audio_tap_begin() and audio_tap_end(), both
 * called from the feeder thread in stream.c.  Nothing here runs on a USBD
 * callback, which is why it is allowed to block on the route settling.
 */

#include "audio_tap.h"

#include "log.h"
#include "resolver.h"
#include "stream.h"

#include <stdint.h>
#include <string.h>

#include <psp2kern/kernel/threadmgr.h>
#include <taihen.h>

#define LOG_PREFIX "[uac-pstv-tap] "
#define ROUTE_RAM 0x00000010u
#define ROUTE_WAKE 0x00000010u
#define ROUTE_TIMEOUT_US 1000000u
#define ROUTE_POLL_US 1000u
#define STOP_SETTLE_US 50000u
#define CAPTURE_TARGET 0
#define CAPTURE_RATE 48000
#define CAPTURE_CHANNELS 2
#define CAPTURE_FRAMES UAC_STREAM_CAPTURE_FRAMES
#define CAPTURE_BYTES UAC_STREAM_CAPTURE_BYTES
/*
 * Priority and affinity are copied from Sony's own DataSend worker, which this
 * thread stands in for -- it feeds the same 480-frame RAM-output pages on the
 * same cadence, so it needs the same scheduling treatment.  0x12 is high, and
 * deliberately so; do not lower it without measuring for dropouts first.
 */
#define CAPTURE_PRIORITY 0x12
#define CAPTURE_STACK 0x2000u
#define CAPTURE_CPU_MASK 0x80000u
#define WORKER_TIMEOUT_US 1000000u

/*
 * Distinct codes rather than a bare -1: this is the subsystem most likely to
 * fail in the field and the hardest to reason about from a log, and
 * "0xffffffff" tells you only that some stage failed.
 */
#define UAC_TAP_ROUTE_BUSY ((int)0x80A10001)
#define UAC_TAP_ROUTE_WAKE_FAILED ((int)0x80A10002)
#define UAC_TAP_ACQUIRE_TIMEOUT ((int)0x80A10003)
#define UAC_TAP_RELEASE_TIMEOUT ((int)0x80A10004)
#define UAC_TAP_WORKER_START_TIMEOUT ((int)0x80A10005)
#define UAC_TAP_WORKER_BAD_STATE ((int)0x80A10006)
#define UAC_TAP_WORKER_NO_THREAD ((int)0x80A10007)
#define UAC_TAP_START_REFUSED ((int)0x80A10008)
#define UAC_TAP_HOOK_RELEASE_FAILED ((int)0x80A10009)

enum {
	WORKER_IDLE = 0,
	WORKER_STARTING,
	WORKER_RUNNING,
	WORKER_EXITED,
};

static AudioLayout audio;
static tai_hook_ref_t send_start_ref;
static tai_hook_ref_t device_stop_ref;
static SceUID send_start_hook = -1;
static SceUID device_stop_hook = -1;
static volatile int virtual_owner;
static volatile int accept_start;
static volatile int stop_requested;
static volatile int worker_state;
static volatile SceUID worker_thread = -1;
static volatile uint32_t physical_stop_count;

static int owns_route(void)
{
	return __atomic_load_n(&virtual_owner, __ATOMIC_ACQUIRE) != 0;
}

static int physical_stop(void *state)
{
	if (owns_route()) {
		__atomic_add_fetch(&physical_stop_count, 1u, __ATOMIC_RELAXED);
		return 0;
	}
	return TAI_CONTINUE(int, device_stop_ref, state);
}

static int capture_worker(SceSize args, void *argp)
{
	void *previous = audio.page[0];
	void *next = audio.page[1];
	int expected = WORKER_STARTING;
	int result;

	(void)args;
	(void)argp;
	if (!__atomic_compare_exchange_n(&worker_state, &expected,
		WORKER_RUNNING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return UAC_TAP_WORKER_BAD_STATE;

	result = audio.ram_rate(CAPTURE_RATE);
	if (result >= 0)
		result = audio.ram_channels(CAPTURE_CHANNELS);
	while (result >= 0 &&
	       !__atomic_load_n(&stop_requested, __ATOMIC_ACQUIRE)) {
		void *swap;

		result = audio.ram_submit(CAPTURE_TARGET, next, CAPTURE_FRAMES);
		if (result < 0)
			break;
		(void)uac_stream_publish(previous);
		swap = previous;
		previous = next;
		next = swap;
	}
	if (result >= 0) {
		(void)audio.ram_rate(CAPTURE_RATE);
		(void)audio.ram_channels(CAPTURE_CHANNELS);
	}
	__atomic_store_n(&worker_state, WORKER_EXITED, __ATOMIC_RELEASE);
	return result;
}

static int start_capture(uint32_t mac0, uint32_t mac1, uint32_t flags)
{
	int expected = WORKER_IDLE;
	SceUID thread;
	int result;

	if (!owns_route())
		return TAI_CONTINUE(int, send_start_ref, mac0, mac1, flags);
	if (!__atomic_load_n(&accept_start, __ATOMIC_ACQUIRE))
		return UAC_TAP_START_REFUSED;
	__atomic_store_n(&stop_requested, 0, __ATOMIC_RELEASE);
	if (!__atomic_compare_exchange_n(&worker_state, &expected,
		WORKER_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE) ||
	    !__atomic_load_n(&accept_start, __ATOMIC_ACQUIRE)) {
		if (expected == WORKER_IDLE)
			__atomic_store_n(&worker_state, WORKER_IDLE,
				__ATOMIC_RELEASE);
		return UAC_TAP_START_REFUSED;
	}

	memset(audio.page[0], 0, CAPTURE_BYTES);
	thread = ksceKernelCreateThread("uac_audio_tap", capture_worker,
		CAPTURE_PRIORITY, CAPTURE_STACK, 0, CAPTURE_CPU_MASK, NULL);
	if (thread < 0) {
		__atomic_store_n(&worker_state, WORKER_IDLE, __ATOMIC_RELEASE);
		return thread;
	}
	__atomic_store_n(&worker_thread, thread, __ATOMIC_RELEASE);
	*audio.send_worker_active = 1u;
	result = ksceKernelStartThread(thread, 0, NULL);
	if (result < 0) {
		*audio.send_worker_active = 0u;
		(void)ksceKernelDeleteThread(thread);
		__atomic_store_n(&worker_thread, -1, __ATOMIC_RELEASE);
		__atomic_store_n(&worker_state, WORKER_IDLE, __ATOMIC_RELEASE);
	}
	return result;
}

static int stop_capture(void)
{
	uint32_t start = ksceKernelGetSystemTimeLow();
	SceUID thread;
	SceUInt timeout = WORKER_TIMEOUT_US;
	int state;
	int status;
	int result;

	__atomic_store_n(&accept_start, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&stop_requested, 1, __ATOMIC_RELEASE);
	while ((state = __atomic_load_n(&worker_state, __ATOMIC_ACQUIRE)) ==
	       WORKER_STARTING) {
		if ((uint32_t)(ksceKernelGetSystemTimeLow() - start) >
		    WORKER_TIMEOUT_US)
			return UAC_TAP_WORKER_START_TIMEOUT;
		ksceKernelDelayThread(ROUTE_POLL_US);
	}
	if (state == WORKER_IDLE) {
		*audio.send_worker_active = 0u;
		return 0;
	}
	if (state != WORKER_RUNNING && state != WORKER_EXITED)
		return UAC_TAP_WORKER_BAD_STATE;

	thread = __atomic_load_n(&worker_thread, __ATOMIC_ACQUIRE);
	if (thread < 0)
		return UAC_TAP_WORKER_NO_THREAD;
	result = ksceKernelWaitThreadEnd(thread, &status, &timeout);
	if (result < 0)
		return result;
	result = ksceKernelDeleteThread(thread);
	if (result < 0)
		return result;
	__atomic_store_n(&worker_thread, -1, __ATOMIC_RELEASE);
	__atomic_store_n(&worker_state, WORKER_IDLE, __ATOMIC_RELEASE);
	*audio.send_worker_active = 0u;
	return 0;
}

static int release_hook(SceUID *id, tai_hook_ref_t ref)
{
	int result;

	if (*id < 0)
		return 0;
	result = taiHookReleaseForKernel(*id, ref);
	if (result >= 0)
		*id = -1;
	return result;
}

static int remove_hooks(void)
{
	int stop_result = release_hook(&device_stop_hook, device_stop_ref);
	int start_result = release_hook(&send_start_hook, send_start_ref);

	return stop_result < 0 ? stop_result : start_result;
}

static int install_hooks(void)
{
	int result;

	send_start_hook = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&send_start_ref, audio.module_id, 0, audio.send_start_offset, 1,
		start_capture);
	if (send_start_hook < 0)
		return send_start_hook;
	device_stop_hook = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&device_stop_ref, audio.module_id, 0, audio.device_stop_offset, 1,
		physical_stop);
	if (device_stop_hook >= 0)
		return 0;

	result = device_stop_hook;
	device_stop_hook = -1;
	if (release_hook(&send_start_hook, send_start_ref) < 0)
		return UAC_TAP_HOOK_RELEASE_FAILED;
	return result;
}

static int release_route(void)
{
	uint32_t token;
	uint32_t start;
	int result = 0;

	if (!owns_route())
		return 0;
	result = stop_capture();
	if (result < 0)
		return result;
	result = 0;
	ksceKernelDelayThread(STOP_SETTLE_US);

	token = audio.cpu_lock(audio.route_lock);
	*audio.route_word &= ~ROUTE_RAM;
	*audio.transport_ready = 0u;
	*audio.route_dirty = 1u;
	audio.cpu_unlock(audio.route_lock, token);
	if (audio.route_wake(ROUTE_WAKE) < 0)
		result = UAC_TAP_ROUTE_WAKE_FAILED;

	/*
	 * The release is complete when AVConfig's own four state fields say so,
	 * and on nothing else.
	 *
	 * Waiting on physical_stop_count as a fifth condition is tempting -- it
	 * would mean Sony had acknowledged the teardown through the device-stop
	 * wrapper we hook.  Do not add it.  stop_capture() above has already
	 * cleared send_worker_active, so Sony is entitled to conclude there is
	 * nothing running to stop and never call that wrapper at all.  When it
	 * makes that choice the condition can never be satisfied and the loop
	 * spends the entire ROUTE_TIMEOUT_US before giving up -- a second during
	 * which any incoming attach has to be deferred.  The count is maintained
	 * and logged because a zero is worth knowing about, not because the
	 * release depends on it.
	 *
	 * Ownership is dropped whether or not the wait succeeded.  The route word
	 * is already cleared by this point, so keeping ownership would only leave
	 * physical_stop() swallowing Sony's device stop and stop cleanup() from
	 * removing the hooks.
	 */
	start = ksceKernelGetSystemTimeLow();
	while (*audio.selected_target == ROUTE_RAM ||
	       *audio.transport_ready != 0u || *audio.route_dirty != 0u ||
	       *audio.send_worker_active != 0u) {
		if ((uint32_t)(ksceKernelGetSystemTimeLow() - start) >
		    ROUTE_TIMEOUT_US) {
			result = UAC_TAP_RELEASE_TIMEOUT;
			break;
		}
		ksceKernelDelayThread(ROUTE_POLL_US);
	}
	uac_log(LOG_PREFIX "route released in %u us, %u physical stops\n",
		(unsigned int)(ksceKernelGetSystemTimeLow() - start),
		__atomic_load_n(&physical_stop_count, __ATOMIC_ACQUIRE));
	__atomic_store_n(&virtual_owner, 0, __ATOMIC_RELEASE);
	return result;
}

static int acquire_route(void)
{
	uint32_t token;
	uint32_t start;
	int result;

	token = audio.cpu_lock(audio.route_lock);
	if ((*audio.route_word & ROUTE_RAM) != 0u ||
	    *audio.selected_target == ROUTE_RAM || *audio.transport_ready != 0u ||
	    *audio.send_worker_active != 0u || *audio.recv_worker_active != 0u) {
		audio.cpu_unlock(audio.route_lock, token);
		return UAC_TAP_ROUTE_BUSY;
	}
	__atomic_store_n(&physical_stop_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&virtual_owner, 1, __ATOMIC_RELEASE);
	__atomic_store_n(&accept_start, 1, __ATOMIC_RELEASE);
	*audio.route_word |= ROUTE_RAM;
	*audio.transport_ready = 1u;
	*audio.route_dirty = 1u;
	audio.cpu_unlock(audio.route_lock, token);

	result = audio.route_wake(ROUTE_WAKE);
	if (result < 0)
		goto fail;
	start = ksceKernelGetSystemTimeLow();
	while ((*audio.route_word & ROUTE_RAM) != ROUTE_RAM ||
	       *audio.selected_target != ROUTE_RAM ||
	       *audio.transport_ready != 1u || *audio.route_dirty != 0u ||
	       *audio.send_worker_active != 1u ||
	       __atomic_load_n(&worker_state, __ATOMIC_ACQUIRE) != WORKER_RUNNING) {
		if ((uint32_t)(ksceKernelGetSystemTimeLow() - start) >
		    ROUTE_TIMEOUT_US) {
			result = UAC_TAP_ACQUIRE_TIMEOUT;
			goto fail;
		}
		ksceKernelDelayThread(ROUTE_POLL_US);
	}
	return 0;

fail:
	if (release_route() < 0)
		return UAC_TAP_RELEASE_TIMEOUT;
	return result;
}

static int cleanup(void)
{
	int result = release_route();

	if (result < 0)
		return result;
	result = remove_hooks();
	if (result < 0)
		return result;
	resolver_close(&audio);
	return 0;
}

/*
 * Take ownership of the AVConfig route for one USB session.
 *
 * Unwinding on failure is deliberately partial.  The layout is only dropped by
 * a session that ends up owning nothing; if the route is still ours the hooks
 * must stay installed, because releasing it later needs the physical-stop hook
 * to observe Sony's acknowledgement.  audio_tap_shutdown() retries that.
 */
int audio_tap_begin(void)
{
	int result;

	/* An earlier session may have failed to unwind. Retry before rebuilding. */
	if (owns_route()) {
		result = cleanup();
		if (result < 0)
			return result;
	}
	if (send_start_hook >= 0 || device_stop_hook >= 0) {
		result = remove_hooks();
		if (result < 0)
			return result;
	}

	resolver_close(&audio);
	result = resolver_open(&audio);
	if (result < 0)
		return result;

	result = install_hooks();
	if (result < 0) {
		/* install_hooks() rolls itself back; drop the layout if it managed to. */
		if (send_start_hook < 0 && device_stop_hook < 0)
			resolver_close(&audio);
		return result;
	}

	result = acquire_route();
	if (result >= 0) {
		uac_log(LOG_PREFIX "native 480-frame A/B capture active\n");
		return 0;
	}

	/* Still ours: leave the hooks for shutdown to finish the release. */
	if (owns_route())
		return result;
	if (remove_hooks() < 0)
		return UAC_TAP_HOOK_RELEASE_FAILED;
	resolver_close(&audio);
	return result;
}

int audio_tap_end(void)
{
	return cleanup();
}

int audio_tap_shutdown(void)
{
	return cleanup();
}
