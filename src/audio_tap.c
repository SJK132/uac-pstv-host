/*
 * AVConfig route ownership.
 *
 * Sony's audio path normally hands decoded PCM to the HDMI transport.  For the
 * duration of one USB session this file takes that route over: it hooks
 * AVConfig's DataSend start, acknowledges its physical-stop wrapper locally,
 * flips the route word to RAM output, and runs a stand-in worker that passes
 * each captured page to stream.c instead.
 *
 * The whole file lives between audio_tap_begin() and audio_tap_end().  Route
 * acquisition is polled so the feeder can keep USB alive with startup silence;
 * release may block on the capture worker and AVConfig state convergence.
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
_Static_assert(CAPTURE_FRAMES <= 512u,
	"capture block exceeds Sony's 0x800-byte RAM-output page");
/*
 * Priority and affinity are copied from Sony's own DataSend worker, which this
 * thread stands in for -- it feeds the same RAM-output pages, so it needs the
 * same scheduling treatment.  0x12 is high, and deliberately so; do not lower it
 * without measuring for dropouts first.  Note that shrinking CAPTURE_FRAMES
 * raises this thread's wakeup rate proportionally, which is the cost side of
 * buying lower latency that way.
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
	WORKER_DORMANT,
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
static volatile int worker_result;
static volatile int acquire_pending;
static volatile uint32_t acquire_started;
#ifdef UAC_PSTV_ENABLE_LOGGING
static volatile uint32_t physical_stop_count;
#endif

static int owns_route(void)
{
	return __atomic_load_n(&virtual_owner, __ATOMIC_ACQUIRE) != 0;
}

static int physical_stop(void *state)
{
	if (owns_route()) {
#ifdef UAC_PSTV_ENABLE_LOGGING
		__atomic_add_fetch(&physical_stop_count, 1u, __ATOMIC_RELAXED);
#endif
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
		WORKER_RUNNING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		result = UAC_TAP_WORKER_BAD_STATE;
		__atomic_store_n(&worker_result, result, __ATOMIC_RELEASE);
		if (!__atomic_load_n(&stop_requested, __ATOMIC_ACQUIRE))
			uac_stream_source_failed(result);
		__atomic_store_n(&worker_state, WORKER_EXITED, __ATOMIC_RELEASE);
		return result;
	}

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
	__atomic_store_n(&worker_result, result, __ATOMIC_RELEASE);
	if (result < 0 &&
	    !__atomic_load_n(&stop_requested, __ATOMIC_ACQUIRE))
		uac_stream_source_failed(result);
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
	if (!__atomic_compare_exchange_n(&worker_state, &expected,
		WORKER_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return UAC_TAP_START_REFUSED;

	/*
	 * The no-op RMW orders this start against stop_capture()'s exchange.
	 * If stop wins, we restore its stop request; if start wins, a later stop
	 * necessarily writes after us.  A plain second load cannot prove that.
	 */
	__atomic_store_n(&stop_requested, 0, __ATOMIC_RELEASE);
	expected = 1;
	if (!__atomic_compare_exchange_n(&accept_start, &expected, 1, 0,
		__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		__atomic_store_n(&stop_requested, 1, __ATOMIC_RELEASE);
		__atomic_store_n(&worker_state, WORKER_IDLE, __ATOMIC_RELEASE);
		return UAC_TAP_START_REFUSED;
	}
	__atomic_store_n(&worker_result, 0, __ATOMIC_RELEASE);

	memset(audio.page[0], 0, CAPTURE_BYTES);
	thread = ksceKernelCreateThread("uac_audio_tap", capture_worker,
		CAPTURE_PRIORITY, CAPTURE_STACK, 0, CAPTURE_CPU_MASK, NULL);
	if (thread < 0) {
		__atomic_store_n(&worker_result, thread, __ATOMIC_RELEASE);
		__atomic_store_n(&worker_state, WORKER_IDLE, __ATOMIC_RELEASE);
		return thread;
	}
	__atomic_store_n(&worker_thread, thread, __ATOMIC_RELEASE);
	*audio.send_worker_active = 1u;
	result = ksceKernelStartThread(thread, 0, NULL);
	if (result < 0) {
		*audio.send_worker_active = 0u;
		__atomic_store_n(&worker_result, result, __ATOMIC_RELEASE);
		/* Keep the UID until cleanup confirms that deletion succeeded. */
		__atomic_store_n(&worker_state, WORKER_DORMANT, __ATOMIC_RELEASE);
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

	(void)__atomic_exchange_n(&accept_start, 0, __ATOMIC_ACQ_REL);
	__atomic_store_n(&stop_requested, 1, __ATOMIC_RELEASE);
	while ((state = __atomic_load_n(&worker_state, __ATOMIC_ACQUIRE)) ==
	       WORKER_STARTING) {
		if ((uint32_t)(ksceKernelGetSystemTimeLow() - start) >
		    WORKER_TIMEOUT_US)
			return UAC_TAP_WORKER_START_TIMEOUT;
		ksceKernelDelayThread(ROUTE_POLL_US);
	}
	thread = __atomic_load_n(&worker_thread, __ATOMIC_ACQUIRE);
	if (state == WORKER_IDLE || state == WORKER_DORMANT) {
		if (thread >= 0) {
			result = ksceKernelDeleteThread(thread);
			if (result < 0)
				return result;
			__atomic_store_n(&worker_thread, -1, __ATOMIC_RELEASE);
		} else if (state == WORKER_DORMANT) {
			return UAC_TAP_WORKER_NO_THREAD;
		}
		__atomic_store_n(&worker_state, WORKER_IDLE, __ATOMIC_RELEASE);
		*audio.send_worker_active = 0u;
		return 0;
	}
	if (state != WORKER_RUNNING && state != WORKER_EXITED)
		return UAC_TAP_WORKER_BAD_STATE;
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
	ksceKernelDelayThread(STOP_SETTLE_US);

	token = audio.cpu_lock(audio.route_lock);
	*audio.route_word &= ~ROUTE_RAM;
	*audio.transport_ready = 0u;
	*audio.route_dirty = 1u;
	audio.cpu_unlock(audio.route_lock, token);
	if (audio.route_wake(ROUTE_WAKE) < 0)
		return UAC_TAP_ROUTE_WAKE_FAILED;

	/*
	 * The release is complete when the route and the send worker we own are
	 * neutral again.  recv_worker_active belongs to an independent Sony path:
	 * it is a valid acquire-time conflict, but cannot be a release condition.
	 *
	 * Waiting on physical_stop_count as a fifth condition is tempting -- it
	 * would mean Sony had acknowledged the teardown through the device-stop
	 * wrapper we hook.  Do not add it.  stop_capture() above has already
	 * cleared send_worker_active, so Sony is entitled to conclude there is
	 * nothing running to stop and never call that wrapper at all.  When it
	 * makes that choice the condition can never be satisfied and the loop
	 * spends the entire ROUTE_TIMEOUT_US before giving up -- a second during
	 * which any incoming attach has to be deferred.  Logging builds maintain
	 * the count for diagnosis, but release never pays for it.
	 *
	 * A timeout retains ownership, hooks and the resolved layout so shutdown
	 * can retry without unloading code or pointers still needed by AVConfig.
	 */
	start = ksceKernelGetSystemTimeLow();
	while (*audio.selected_target == ROUTE_RAM ||
	       (*audio.route_word & ROUTE_RAM) != 0u ||
	       *audio.transport_ready != 0u || *audio.route_dirty != 0u ||
	       *audio.send_worker_active != 0u) {
		if ((uint32_t)(ksceKernelGetSystemTimeLow() - start) >
		    ROUTE_TIMEOUT_US)
			return UAC_TAP_RELEASE_TIMEOUT;
		ksceKernelDelayThread(ROUTE_POLL_US);
	}
#ifdef UAC_PSTV_ENABLE_LOGGING
	uac_log(LOG_PREFIX "route released in %u us, %u physical stops\n",
		(unsigned int)(ksceKernelGetSystemTimeLow() - start),
		__atomic_load_n(&physical_stop_count, __ATOMIC_ACQUIRE));
#endif
	__atomic_store_n(&acquire_pending, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&virtual_owner, 0, __ATOMIC_RELEASE);
	return result;
}

static int begin_route(void)
{
	uint32_t token;

	if (__atomic_load_n(&worker_state, __ATOMIC_ACQUIRE) != WORKER_IDLE ||
	    __atomic_load_n(&worker_thread, __ATOMIC_ACQUIRE) >= 0)
		return UAC_TAP_WORKER_BAD_STATE;
	token = audio.cpu_lock(audio.route_lock);
	if ((*audio.route_word & ROUTE_RAM) != 0u ||
	    *audio.selected_target == ROUTE_RAM || *audio.transport_ready != 0u ||
	    *audio.route_dirty != 0u || *audio.send_worker_active != 0u ||
	    *audio.recv_worker_active != 0u) {
		audio.cpu_unlock(audio.route_lock, token);
		return UAC_TAP_ROUTE_BUSY;
	}
#ifdef UAC_PSTV_ENABLE_LOGGING
	__atomic_store_n(&physical_stop_count, 0u, __ATOMIC_RELAXED);
#endif
	__atomic_store_n(&worker_result, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&stop_requested, 1, __ATOMIC_RELEASE);
	*audio.route_word |= ROUTE_RAM;
	*audio.transport_ready = 1u;
	*audio.route_dirty = 1u;
	__atomic_store_n(&accept_start, 1, __ATOMIC_RELEASE);
	__atomic_store_n(&virtual_owner, 1, __ATOMIC_RELEASE);
	audio.cpu_unlock(audio.route_lock, token);

	__atomic_store_n(&acquire_started, ksceKernelGetSystemTimeLow(),
		__ATOMIC_RELEASE);
	__atomic_store_n(&acquire_pending, 1, __ATOMIC_RELEASE);
	return audio.route_wake(ROUTE_WAKE) < 0 ? UAC_TAP_ROUTE_WAKE_FAILED : 0;
}

static int worker_present(void)
{
	return __atomic_load_n(&worker_state, __ATOMIC_ACQUIRE) != WORKER_IDLE ||
	       __atomic_load_n(&worker_thread, __ATOMIC_ACQUIRE) >= 0;
}

static int cleanup(void)
{
	int result;

	if (owns_route()) {
		result = release_route();
		if (result < 0)
			return result;
	} else if (worker_present()) {
		result = stop_capture();
		if (result < 0)
			return result;
	}
	result = remove_hooks();
	if (result < 0)
		return result;
	resolver_close(&audio);
	return 0;
}

/*
 * Take ownership of the AVConfig route for one USB session.
 *
 * Unwinding is fail-closed.  While the route is still ours, AVConfig may enter
 * either wrapper and a later retry still needs the resolved state, so hooks and
 * layout remain installed.  audio_tap_shutdown() retries that cleanup.
 */
int audio_tap_begin(void)
{
	int result;
	int cleanup_result;

	/* Finish any retained fail-closed state before rebuilding the layout. */
	result = cleanup();
	if (result < 0)
		return result;
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

	result = begin_route();
	if (result >= 0)
		return 0;

	cleanup_result = cleanup();
	if (cleanup_result < 0)
		return cleanup_result;
	return result;
}

int audio_tap_poll(void)
{
	int state;
	int result;

	if (!owns_route())
		return UAC_TAP_WORKER_BAD_STATE;
	state = __atomic_load_n(&worker_state, __ATOMIC_ACQUIRE);
	result = __atomic_load_n(&worker_result, __ATOMIC_ACQUIRE);
	if (result < 0)
		return result;
	if ((*audio.route_word & ROUTE_RAM) == ROUTE_RAM &&
	    *audio.selected_target == ROUTE_RAM &&
	    *audio.transport_ready == 1u && *audio.route_dirty == 0u &&
	    *audio.send_worker_active == 1u &&
	    *audio.recv_worker_active == 0u && state == WORKER_RUNNING &&
	    __atomic_load_n(&worker_thread, __ATOMIC_ACQUIRE) >= 0 &&
	    __atomic_load_n(&accept_start, __ATOMIC_ACQUIRE) != 0 &&
	    __atomic_load_n(&stop_requested, __ATOMIC_ACQUIRE) == 0) {
		if (__atomic_exchange_n(&acquire_pending, 0,
			__ATOMIC_ACQ_REL) != 0)
			uac_log(LOG_PREFIX "native %u-frame A/B capture active\n",
				(unsigned int)CAPTURE_FRAMES);
		return 0;
	}
	if (state == WORKER_EXITED ||
	    (state != WORKER_IDLE && state != WORKER_STARTING &&
	     state != WORKER_RUNNING))
		return UAC_TAP_WORKER_BAD_STATE;
	if (__atomic_load_n(&acquire_pending, __ATOMIC_ACQUIRE) == 0)
		return UAC_TAP_WORKER_BAD_STATE;
	if ((uint32_t)(ksceKernelGetSystemTimeLow() -
	    __atomic_load_n(&acquire_started, __ATOMIC_ACQUIRE)) >
	    ROUTE_TIMEOUT_US)
		return UAC_TAP_ACQUIRE_TIMEOUT;
	return 1;
}

int audio_tap_end(void)
{
	return cleanup();
}

int audio_tap_shutdown(void)
{
	return cleanup();
}
