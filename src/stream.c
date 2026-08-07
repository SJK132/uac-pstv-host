#include "stream.h"
#include "log.h"
#include "mixer.h"

#include <stdint.h>
#include <string.h>

#include <psp2kern/kernel/cpu/cache.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv] "
#define PACKET_BYTES 192u
#define STOP_POLL_US 1000u
#define STOP_POLLS 50u
#define CALLBACK_CLOSING 0x80000000u
#define CALLBACK_REFS 0x7fffffffu

enum { STREAM_IDLE, STREAM_STARTING, STREAM_RUNNING, STREAM_STOPPING };

typedef struct {
	int16_t buffer[PACKET_BYTES / sizeof(int16_t)] __attribute__((aligned(64)));
	ksceUsbdIsochTransfer transfer __attribute__((aligned(64)));
} StreamRequest;

static StreamRequest request __attribute__((aligned(64)));
static int stream_state;
static int stream_pipe = -1;
static uint32_t stream_generation;
static uint32_t callback_guard;

static uint32_t next_generation(void)
{
	uint32_t value = __atomic_add_fetch(&stream_generation, 1, __ATOMIC_ACQ_REL);
	return value ? value : __atomic_add_fetch(&stream_generation, 1,
		__ATOMIC_ACQ_REL);
}

static void finish_retire(void)
{
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) == STREAM_STOPPING &&
		__atomic_load_n(&stream_pipe, __ATOMIC_ACQUIRE) < 0 &&
		__atomic_load_n(&callback_guard, __ATOMIC_ACQUIRE) == CALLBACK_CLOSING) {
		__atomic_store_n(&callback_guard, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
	}
}

static void callback_leave(void)
{
	if (__atomic_fetch_sub(&callback_guard, 1, __ATOMIC_RELEASE) ==
		(CALLBACK_CLOSING | 1u))
		finish_retire();
}

static int callback_enter(uint32_t generation)
{
	uint32_t guard;

	for (;;) {
		guard = __atomic_load_n(&callback_guard, __ATOMIC_ACQUIRE);
		if (guard & CALLBACK_CLOSING)
			return 0;
		if (__atomic_compare_exchange_n(&callback_guard, &guard, guard + 1, 0,
			__ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
			break;
	}
	if (__atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE) == generation)
		return 1;
	callback_leave();
	return 0;
}

static void stop_stream(int result, uint8_t psw)
{
	int previous = __atomic_exchange_n(&stream_state, STREAM_STOPPING,
		__ATOMIC_ACQ_REL);

	__atomic_fetch_or(&callback_guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	if (previous == STREAM_RUNNING || previous == STREAM_STARTING)
		uac_mixer_stop();
	if (result < 0)
		uac_log(LOG_PREFIX "stream transfer failed: 0x%08x\n", result);
	else if (psw != USBD_CC_NOERR)
		uac_log(LOG_PREFIX "stream packet failed: PSW 0x%02x\n", psw);
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *transfer,
	void *arg);

static int queue_request(uint32_t generation)
{
	int pipe;

	request.transfer.packets[0].len = PACKET_BYTES;
	request.transfer.packets[0].PSW = 0;
	uac_mixer_fill(request.buffer);
	ksceKernelDcacheCleanRange(request.buffer, PACKET_BYTES);
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
		__atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE) != generation)
		return 1;
	pipe = __atomic_load_n(&stream_pipe, __ATOMIC_ACQUIRE);
	if (pipe < 0)
		return 1;
	request.transfer.relative_start_frame = 1;
	return ksceUsbdIsochronousTransfer(pipe, &request.transfer, transfer_done,
		(void *)(uintptr_t)generation);
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *transfer,
	void *arg)
{
	uint32_t generation = (uint32_t)(uintptr_t)arg;
	uint8_t psw;

	if (!callback_enter(generation))
		return;
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING)
		goto out;
	psw = transfer->packets[0].PSW;
	if (result < 0 || psw != USBD_CC_NOERR)
		stop_stream(result, psw);
	else if ((result = queue_request(generation)) != 0)
		stop_stream(result < 0 ? result : 0, USBD_CC_NOERR);
out:
	callback_leave();
}

int uac_stream_start(int pipe_id)
{
	int expected = STREAM_IDLE;
	int result;
	uint32_t generation;

	if (pipe_id < 0 || !__atomic_compare_exchange_n(&stream_state, &expected,
		STREAM_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;
	generation = next_generation();
	__atomic_store_n(&stream_pipe, pipe_id, __ATOMIC_RELEASE);
	memset(&request, 0, sizeof(request));
	request.transfer.buffer_base = request.buffer;
	/* On PSTV this is the OUT byte count, despite its VitaSDK name. */
	request.transfer.num_packets = PACKET_BYTES;
	uac_mixer_start();

	expected = STREAM_STARTING;
	if (!__atomic_compare_exchange_n(&stream_state, &expected, STREAM_RUNNING, 0,
		__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;
	result = queue_request(generation);
	if (result != 0) {
		stop_stream(result < 0 ? result : 0, USBD_CC_NOERR);
		return result < 0 ? result : -1;
	}
	uac_log(LOG_PREFIX "stream queued: %u-byte OUT request\n", PACKET_BYTES);
	return 0;
}

void uac_stream_stop(void)
{
	int state, expected;
	uint32_t wait;

	for (;;) {
		state = __atomic_load_n(&stream_state, __ATOMIC_ACQUIRE);
		if (state == STREAM_IDLE)
			return;
		if (state == STREAM_STOPPING)
			break;
		expected = state;
		if (__atomic_compare_exchange_n(&stream_state, &expected,
			STREAM_STOPPING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
			if (state == STREAM_RUNNING || state == STREAM_STARTING)
				uac_mixer_stop();
			break;
		}
	}

	/* Wait for callback code, not for the request cancelled by pipe close. */
	__atomic_fetch_or(&callback_guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	for (wait = 0; wait < STOP_POLLS; ++wait) {
		if (!(__atomic_load_n(&callback_guard, __ATOMIC_ACQUIRE) & CALLBACK_REFS))
			return;
		ksceKernelDelayThread(STOP_POLL_US);
	}
	uac_log(LOG_PREFIX "stream callback still active after %u ms\n",
		(STOP_POLLS * STOP_POLL_US) / 1000u);
}

void uac_stream_pipe_closed(int pipe_id)
{
	int previous;

	if (pipe_id < 0 ||
		__atomic_load_n(&stream_pipe, __ATOMIC_ACQUIRE) != pipe_id)
		return;
	previous = __atomic_exchange_n(&stream_state, STREAM_STOPPING,
		__ATOMIC_ACQ_REL);
	if (previous == STREAM_RUNNING || previous == STREAM_STARTING)
		uac_mixer_stop();
	__atomic_fetch_or(&callback_guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	(void)next_generation(); /* invalidate completions from the closed pipe */
	__atomic_store_n(&stream_pipe, -1, __ATOMIC_RELEASE);
	finish_retire();
}
