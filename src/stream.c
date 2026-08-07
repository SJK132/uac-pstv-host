#include "stream.h"
#include "log.h"
#include "mixer.h"

#include <psp2kern/kernel/cpu/cache.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/usbd.h>
#include <stdint.h>
#include <string.h>

#define LOG_PREFIX "[uac-pstv] "
#define PACKET_BYTES 192u
#define STOP_POLL_US 1000u
#define STOP_POLLS 50u

enum { STREAM_IDLE, STREAM_STARTING, STREAM_RUNNING, STREAM_STOPPING };

typedef struct {
	int16_t buffer[PACKET_BYTES / sizeof(int16_t)] __attribute__((aligned(64)));
	ksceUsbdIsochTransfer transfer __attribute__((aligned(64)));
} StreamRequest;

static StreamRequest stream_request __attribute__((aligned(64)));
static int stream_state;
static int stream_pipe = -1;

/* IDLE means the static request is safe to initialize for another stream. */
static void retire_stream(int result, uint8_t psw)
{
	int previous = __atomic_exchange_n(&stream_state, STREAM_STOPPING,
		__ATOMIC_ACQ_REL);

	if (previous == STREAM_RUNNING)
		uac_mixer_stop();
	if (result < 0)
		uac_log(LOG_PREFIX "stream transfer failed: 0x%08x\n", result);
	else if (psw != USBD_CC_NOERR)
		uac_log(LOG_PREFIX "stream packet failed: PSW 0x%02x\n", psw);
	stream_pipe = -1;
	__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *transfer,
	void *arg);

/* Returns 0 when queued, 1 when stopping, or the USB submission error. */
static int queue_request(StreamRequest *request)
{
	request->transfer.packets[0].len = PACKET_BYTES;
	request->transfer.packets[0].PSW = 0;
	uac_mixer_fill(request->buffer);
	ksceKernelDcacheCleanRange(request->buffer, PACKET_BYTES);
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING)
		return 1;
	request->transfer.relative_start_frame = 1;
	return ksceUsbdIsochronousTransfer(stream_pipe, &request->transfer,
		transfer_done, request);
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *transfer,
	void *arg)
{
	uint8_t psw = transfer->packets[0].PSW;

	if (result < 0 || psw != USBD_CC_NOERR) {
		retire_stream(result, psw);
		return;
	}
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING) {
		retire_stream(0, USBD_CC_NOERR);
		return;
	}
	result = queue_request(arg);
	if (result != 0)
		retire_stream(result < 0 ? result : 0, USBD_CC_NOERR);
}

int uac_stream_start(int pipe_id)
{
	int expected = STREAM_IDLE;
	int result;

	if (pipe_id < 0 || !__atomic_compare_exchange_n(&stream_state, &expected,
		STREAM_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;

	stream_pipe = pipe_id;
	memset(&stream_request, 0, sizeof(stream_request));
	stream_request.transfer.buffer_base = stream_request.buffer;
	/* On PSTV this is the OUT byte count, despite its VitaSDK name. */
	stream_request.transfer.num_packets = PACKET_BYTES;
	uac_mixer_start();

	expected = STREAM_STARTING;
	if (!__atomic_compare_exchange_n(&stream_state, &expected, STREAM_RUNNING,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		uac_mixer_stop();
		stream_pipe = -1;
		__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
		return -1;
	}

	result = queue_request(&stream_request);
	if (result != 0) {
		retire_stream(result < 0 ? result : 0, USBD_CC_NOERR);
		return result < 0 ? result : -1;
	}
	uac_log(LOG_PREFIX "stream queued: %u-byte OUT request\n", PACKET_BYTES);
	return 0;
}

void uac_stream_stop(void)
{
	int state;
	int expected;
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
			if (state == STREAM_RUNNING)
				uac_mixer_stop();
			break;
		}
	}

	/* The owner closes the pipe after the request or callback retires. */
	for (wait = 0; wait < STOP_POLLS; ++wait) {
		if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) == STREAM_IDLE)
			return;
		ksceKernelDelayThread(STOP_POLL_US);
	}
	uac_log(LOG_PREFIX "stream stop timed out after %u ms\n",
		(STOP_POLLS * STOP_POLL_US) / 1000u);
}
