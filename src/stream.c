/*
 * PCM handoff and USB transport.
 *
 * Two halves that meet in the middle:
 *
 *   - the source side, driven by audio_tap's capture worker, publishes whole
 *     480-frame blocks into a two-slot seqlock (uac_stream_publish);
 *   - the transport side runs one feeder thread that cuts those blocks into
 *     48-frame (1 ms) packets and keeps exactly one isochronous request in
 *     flight, with a second context staged behind it.
 *
 * The seqlock is latest-wins, and deliberately not a FIFO.  The capture worker
 * is clocked by Sony's audio hardware rather than by us, so producer and
 * consumer are never rate-matched; a queue between them grows without bound and
 * turns straight into latency, while dropping a stale block costs 10 ms once
 * and self-corrects.  Anyone reaching for a ring buffer here should measure the
 * producer's lead first -- it has been observed a thousand packets ahead.
 */

#include "stream.h"
#include "audio_tap.h"
#include "log.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <psp2kern/kernel/cpu/cache.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv-usb] "

/*
 * Sony-shaped shallow transport for the tested HS bInterval=4 adapter.
 * Two fixed request contexts: one submitted, one READY/preparing.
 */
#define PACKET_FRAMES 48u
#define PACKET_BYTES (PACKET_FRAMES * 4u)
#define PCM_SLOT_COUNT 2u
#define SONY_ISO_PACKET_SLOTS 8u
#define TRANSFER_BYTES PACKET_BYTES
#define CONTEXT_COUNT 2u

#define FREE_EVENT_BIT 0x00000001u
#define PCM_EVENT_BIT 0x00000002u
#define STOP_POLL_US 1000u
#define STOP_POLLS 50u
/*
 * The feeder exits through audio_tap_end(), whose own worst case is
 * WORKER_TIMEOUT_US (thread stop) + STOP_SETTLE_US + ROUTE_TIMEOUT_US (route
 * convergence).  This must exceed that sum or teardown returns while the feeder
 * still owns the route and the hooks.  Since uac1 runs teardown on its own
 * worker rather than the USBD callback thread, a generous bound costs nothing
 * in the common case, which converges in tens of milliseconds.
 */
#define FEEDER_STOP_TIMEOUT_US 4000000u
#define CALLBACK_CLOSING 0x80000000u
#define CALLBACK_REFS 0x7fffffffu

/*
 * Feeder step results.  Any negative value is a transport error carrying its
 * own code straight through to stop_stream(); these are the non-negative ones.
 * FEED_NEED_PCM is also > 0, so callers must test it before testing "> 0".
 */
#define FEED_QUEUED 0		/* packet staged, transport pumped */
#define FEED_STREAM_ENDED 1	/* no longer RUNNING, or generation changed */
#define FEED_NEED_PCM 2		/* no PCM staged yet; wait on the source */

#if TRANSFER_BYTES != 192u
#error Tested transport must remain one 192-byte packet per request
#endif

#if UAC_STREAM_CAPTURE_FRAMES % PACKET_FRAMES != 0u
#error Native blocks must contain a whole number of USB packets
#endif

enum {
	STREAM_IDLE = 0,
	STREAM_STARTING,
	STREAM_RUNNING,
	STREAM_STOPPING,
};

enum {
	CONTEXT_FREE = 0,
	CONTEXT_WRITING,
	CONTEXT_READY,
};

/* Native SceUsbAudio isoch request ABI on the reversed PSTV firmware. */
typedef struct {
	uint16_t len;
	uint16_t status;
} SonyIsoPacket;

typedef struct {
	void *buffer_base;
	uint32_t num_packets;
	SonyIsoPacket packets[SONY_ISO_PACKET_SLOTS];
} SonyIsoTransfer;

typedef char sony_iso_packet_size_must_be_4[
	(sizeof(SonyIsoPacket) == 4u) ? 1 : -1];

#if UINTPTR_MAX == 0xffffffffu
typedef char sony_iso_transfer_size_must_be_0x28[
	(sizeof(SonyIsoTransfer) == 0x28u) ? 1 : -1];
typedef char sony_iso_count_offset_must_be_4[
	(offsetof(SonyIsoTransfer, num_packets) == 0x04u) ? 1 : -1];
typedef char sony_iso_packets_offset_must_be_8[
	(offsetof(SonyIsoTransfer, packets) == 0x08u) ? 1 : -1];
#endif

typedef struct {
	uint8_t buffer[TRANSFER_BYTES] __attribute__((aligned(64)));
	SonyIsoTransfer transfer __attribute__((aligned(64)));
	uint32_t generation;
	int state;
} StreamContext;

typedef struct {
	uint32_t guard;
	uint32_t sequence;
	int16_t pcm[UAC_STREAM_CAPTURE_FRAMES * 2u]
		__attribute__((aligned(64)));
} PcmSlot;

static StreamContext contexts[CONTEXT_COUNT] __attribute__((aligned(64)));
static PcmSlot pcm_slots[PCM_SLOT_COUNT] __attribute__((aligned(64)));

static int stream_state;
static int stream_pipe = -1;
static uint32_t stream_generation;
static uint32_t callback_guard;
static uint32_t transfer_active;
static uint32_t submit_index;
static uint32_t prime_count;
static int primed;
/*
 * Two event flags, one bit each.  Never merge them into one flag.
 *
 * Both waiters use SCE_EVENT_WAITCLEAR, which clears the entire flag rather
 * than only the matched pattern, so a shared flag lets either wake destroy the
 * other subsystem's pending wakeup.  That is unrecoverable during priming:
 * there is no transfer in flight yet to re-post FREE_EVENT_BIT, so the feeder
 * waits forever and the stream comes up silent with every setup step in the log
 * reporting success.  Use SCE_EVENT_WAITCLEAR_PAT if you ever do need to share.
 */
static SceUID free_event = -1;
static SceUID pcm_event = -1;
static SceUID feeder_thread = -1;

static int pcm_enabled;
static int pcm_producer_busy;
static uint32_t pcm_latest;

/* Consumer state belongs exclusively to the feeder thread. */
static uint32_t pcm_sequence;
static uint32_t pcm_offset;
static int pcm_valid;

static uint32_t next_generation(void)
{
	uint32_t value = __atomic_add_fetch(&stream_generation, 1,
		__ATOMIC_ACQ_REL);

	return value ? value : __atomic_add_fetch(&stream_generation, 1,
		__ATOMIC_ACQ_REL);
}

static void publish_free_context(void)
{
	if (free_event >= 0)
		(void)ksceKernelSetEventFlag(free_event, FREE_EVENT_BIT);
}

static void signal_pcm(void)
{
	if (pcm_event >= 0)
		(void)ksceKernelSetEventFlag(pcm_event, PCM_EVENT_BIT);
}

/* ksceKernelClearEventFlag() takes the mask of bits to KEEP, so zero resets. */
static void reset_event(SceUID event)
{
	if (event >= 0)
		(void)ksceKernelClearEventFlag(event, 0);
}

static uint32_t next_pcm_sequence(uint32_t sequence)
{
	sequence++;
	return sequence ? sequence : 1u;
}

static void pcm_write(const void *pcm)
{
	PcmSlot *slot;
	uint32_t sequence = next_pcm_sequence(
		__atomic_load_n(&pcm_latest, __ATOMIC_RELAXED));

	slot = &pcm_slots[sequence & 1u];
	__atomic_store_n(&slot->guard, (sequence << 1) | 1u,
		__ATOMIC_SEQ_CST);
	memcpy(slot->pcm, pcm, UAC_STREAM_CAPTURE_BYTES);
	__atomic_store_n(&slot->sequence, sequence, __ATOMIC_RELAXED);
	__atomic_store_n(&slot->guard, sequence << 1, __ATOMIC_RELEASE);
	__atomic_store_n(&pcm_latest, sequence, __ATOMIC_RELEASE);
	signal_pcm();
}

int uac_stream_publish(const void *pcm)
{
	if (pcm == NULL || !__atomic_load_n(&pcm_enabled, __ATOMIC_ACQUIRE))
		return 0;
	if (__atomic_exchange_n(&pcm_producer_busy, 1, __ATOMIC_ACQUIRE))
		return 0;
	if (__atomic_load_n(&pcm_enabled, __ATOMIC_ACQUIRE))
		pcm_write(pcm);
	__atomic_store_n(&pcm_producer_busy, 0, __ATOMIC_RELEASE);
	return 1;
}

static int has_free_context(void)
{
	return __atomic_load_n(&contexts[0].state, __ATOMIC_ACQUIRE) == CONTEXT_FREE ||
	       __atomic_load_n(&contexts[1].state, __ATOMIC_ACQUIRE) == CONTEXT_FREE;
}

static void finish_retire(void)
{
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) == STREAM_STOPPING &&
	    __atomic_load_n(&stream_pipe, __ATOMIC_ACQUIRE) < 0 &&
	    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) < 0 &&
	    __atomic_load_n(&callback_guard, __ATOMIC_ACQUIRE) == CALLBACK_CLOSING) {
		__atomic_store_n(&callback_guard, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&transfer_active, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&primed, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
		publish_free_context();
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
		if (__atomic_compare_exchange_n(&callback_guard, &guard, guard + 1u,
			0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
			break;
	}
	if (__atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE) == generation)
		return 1;
	callback_leave();
	return 0;
}

static void stop_stream(int result, uint16_t status)
{
	int previous = __atomic_exchange_n(&stream_state, STREAM_STOPPING,
		__ATOMIC_ACQ_REL);

	__atomic_fetch_or(&callback_guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	publish_free_context();
	signal_pcm();
	if (previous != STREAM_RUNNING && previous != STREAM_STARTING)
		return;
	if (result < 0)
		uac_log(LOG_PREFIX "transfer failed: 0x%08x\n", result);
	else if (status != USBD_CC_NOERR)
		uac_log(LOG_PREFIX "packet failed: status 0x%04x\n", status);
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *transfer,
	void *arg);

static int submit_context(StreamContext *context)
{
	int pipe;

	context->transfer.packets[0].len = PACKET_BYTES;
	context->transfer.packets[0].status = 0;
	ksceKernelDcacheCleanRange(context->buffer, TRANSFER_BYTES);

	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
	    __atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE) !=
		context->generation)
		return 1;
	pipe = __atomic_load_n(&stream_pipe, __ATOMIC_ACQUIRE);
	if (pipe < 0)
		return 1;

	return ksceUsbdIsochronousTransfer(
		pipe,
		(ksceUsbdIsochTransfer *)(void *)&context->transfer,
		transfer_done,
		context);
}

static int pump_ready(void)
{
	StreamContext *context;
	uint32_t expected_active = 0;
	int result;

	/* Exactly one request may be in flight, so losing this CAS is normal. */
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
	    !__atomic_load_n(&primed, __ATOMIC_ACQUIRE) ||
	    !__atomic_compare_exchange_n(&transfer_active, &expected_active, 1u,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return FEED_QUEUED;

	context = &contexts[submit_index];
	if (__atomic_load_n(&context->state, __ATOMIC_ACQUIRE) != CONTEXT_READY) {
		__atomic_store_n(&transfer_active, 0, __ATOMIC_RELEASE);
		return FEED_QUEUED;
	}
	submit_index ^= 1u;

	/* Zero means the request is now owned by USBD and will call back. */
	result = submit_context(context);
	if (result == 0)
		return FEED_QUEUED;

	/* Declined or failed: give the storage back and retire the stream. */
	__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	__atomic_store_n(&transfer_active, 0, __ATOMIC_RELEASE);
	publish_free_context();
	stop_stream(result < 0 ? result : 0, USBD_CC_NOERR);
	return result < 0 ? result : FEED_STREAM_ENDED;
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *sdk_transfer,
	void *arg)
{
	StreamContext *context = (StreamContext *)arg;
	SonyIsoTransfer *transfer = (SonyIsoTransfer *)(void *)sdk_transfer;
	uint16_t status;

	if (!callback_enter(context->generation))
		return;
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING)
		goto out;

	status = transfer->packets[0].status;
	if (result < 0 || status != USBD_CC_NOERR) {
		stop_stream(result, status);
		goto out;
	}

	__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	__atomic_store_n(&transfer_active, 0, __ATOMIC_RELEASE);
	publish_free_context();
	(void)pump_ready();

out:
	callback_leave();
}

static int wait_for_free_context(uint32_t generation, StreamContext **out)
{
	*out = NULL;

	for (;;) {
		uint32_t matched_bits;
		int result;
		int expected;

		if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
		    __atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE) != generation)
			return FEED_STREAM_ENDED;

		result = ksceKernelWaitEventFlag(free_event, FREE_EVENT_BIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched_bits, NULL);
		if (result < 0)
			return result;

		if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
		    __atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE) != generation)
			return FEED_STREAM_ENDED;

		expected = CONTEXT_FREE;
		if (__atomic_compare_exchange_n(&contexts[0].state, &expected,
			CONTEXT_WRITING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			*out = &contexts[0];
		else {
			expected = CONTEXT_FREE;
			if (__atomic_compare_exchange_n(&contexts[1].state, &expected,
				CONTEXT_WRITING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
				*out = &contexts[1];
		}
		if (*out != NULL) {
			if (has_free_context())
				publish_free_context();
			return FEED_QUEUED;
		}
	}
}

static void pcm_begin(void)
{
	__atomic_store_n(&pcm_enabled, 0, __ATOMIC_RELEASE);
	while (__atomic_load_n(&pcm_producer_busy, __ATOMIC_ACQUIRE))
		ksceKernelDelayThread(50);

	memset(pcm_slots, 0, sizeof(pcm_slots));
	__atomic_store_n(&pcm_latest, 0, __ATOMIC_RELAXED);
	pcm_sequence = 0;
	pcm_offset = 0;
	pcm_valid = 0;
	reset_event(pcm_event);
	__atomic_store_n(&pcm_enabled, 1, __ATOMIC_RELEASE);
	uac_log("[uac-pstv-source] armed: 2x480 PCM staging; "
		"48-frame packetizer\n");
}

static void pcm_end(void)
{
	__atomic_store_n(&pcm_enabled, 0, __ATOMIC_RELEASE);
	signal_pcm();
	while (__atomic_load_n(&pcm_producer_busy, __ATOMIC_ACQUIRE))
		ksceKernelDelayThread(50);
}

static int pcm_copy(uint32_t sequence, uint32_t offset, uint8_t *packet)
{
	PcmSlot *slot = &pcm_slots[sequence & 1u];
	uint32_t expected = sequence << 1;
	uint32_t before;
	uint32_t attempt;

	for (attempt = 0; attempt < 3u; ++attempt) {
		before = __atomic_load_n(&slot->guard, __ATOMIC_ACQUIRE);
		if (before != expected ||
		    __atomic_load_n(&slot->sequence, __ATOMIC_RELAXED) != sequence)
			continue;
		memcpy(packet, (const uint8_t *)slot->pcm + offset * 4u,
			PACKET_BYTES);
		if (__atomic_load_n(&slot->guard, __ATOMIC_ACQUIRE) == before)
			return 1;
	}
	return 0;
}

static void pcm_resync(uint32_t latest)
{
	pcm_sequence = latest >= 2u ? latest - 1u : 1u;
	pcm_offset = 0;
	pcm_valid = latest != 0;
}

/* Return zero for a packet, one when no packet is ready, or a negative error. */
static int pcm_next(uint8_t packet[PACKET_BYTES])
{
	uint32_t latest = __atomic_load_n(&pcm_latest, __ATOMIC_ACQUIRE);

	/*
	 * Priming.  uac1 has already selected the alternate setting, so the device
	 * is live and expecting a packet every millisecond well before capture has
	 * produced anything -- the tap still has to acquire the route, and two
	 * blocks have to exist before the consumer can trail the producer.  Send
	 * silence across that window rather than nothing: a gap on an active
	 * isochronous endpoint reads as an invalid stream to the device, while
	 * continuous silence lets it lock cleanly and audio simply fades in.
	 * Only ever taken before the first real block; a mid-stream stall still
	 * falls through to the wait below.
	 */
	if (!pcm_valid) {
		if (latest < 2u) {
			memset(packet, 0, PACKET_BYTES);
			return 0;
		}
		pcm_resync(latest);
	}
	if (latest >= pcm_sequence + PCM_SLOT_COUNT)
		pcm_resync(latest);

	if (!pcm_copy(pcm_sequence, pcm_offset, packet)) {
		latest = __atomic_load_n(&pcm_latest, __ATOMIC_ACQUIRE);
		if (latest >= pcm_sequence + PCM_SLOT_COUNT)
			pcm_resync(latest);
		if (!pcm_copy(pcm_sequence, pcm_offset, packet))
			return 1;
	}

	pcm_offset += PACKET_FRAMES;
	if (pcm_offset == UAC_STREAM_CAPTURE_FRAMES) {
		pcm_sequence = next_pcm_sequence(pcm_sequence);
		pcm_offset = 0;
	}
	return 0;
}

static int pcm_wait(void)
{
	uint32_t matched;
	int result;

	if (!__atomic_load_n(&pcm_enabled, __ATOMIC_ACQUIRE))
		return 1;
	result = ksceKernelWaitEventFlag(pcm_event, PCM_EVENT_BIT,
		SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, NULL);
	if (result < 0)
		return result;
	return __atomic_load_n(&pcm_enabled, __ATOMIC_ACQUIRE) ? 0 : 1;
}

/*
 * Order matters: take USB-owned storage first, then sample the source.
 *
 * The reverse -- copy 1 ms of PCM, then wait for a free context -- reads as
 * equivalent and is not.  That wait is unbounded if a completion is delayed,
 * and the packet copied before it goes stale while we sit in it.  A completion
 * only makes transfer storage reusable; choosing what to put in that storage
 * belongs after ownership is held, so pcm_next() always resyncs against
 * whatever Sony has produced at submission time.
 */
static int queue_next_source_packet(void)
{
	StreamContext *context;
	uint32_t generation;
	int result;

	generation = __atomic_load_n(&stream_generation, __ATOMIC_ACQUIRE);
	if (!uac_stream_is_active())
		return FEED_STREAM_ENDED;

	result = wait_for_free_context(generation, &context);
	if (result != FEED_QUEUED)
		return result;

	/* pcm_next() never fails; it either stages a packet or has none yet. */
	if (pcm_next(context->buffer) != 0) {
		__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
		publish_free_context();
		return FEED_NEED_PCM;
	}

	__atomic_store_n(&context->state, CONTEXT_READY, __ATOMIC_RELEASE);

	/* The transport stays idle until both contexts hold a packet. */
	if (prime_count < CONTEXT_COUNT && ++prime_count == CONTEXT_COUNT)
		__atomic_store_n(&primed, 1, __ATOMIC_RELEASE);

	if (__atomic_load_n(&primed, __ATOMIC_ACQUIRE))
		return pump_ready();
	return FEED_QUEUED;
}

static int usb_feeder_thread(SceSize args, void *argp)
{
	int tap_started = 0;
	int result;

	(void)args;
	(void)argp;
	pcm_begin();
	result = audio_tap_begin();
	if (result < 0) {
		uac_log(LOG_PREFIX "virtual Sony DataSend start failed: 0x%08x\n",
			result);
		stop_stream(result, USBD_CC_NOERR);
		pcm_end();
		return result;
	}
	tap_started = 1;

	while (uac_stream_is_active()) {
		result = queue_next_source_packet();
		if (result == FEED_NEED_PCM) {
			result = pcm_wait();
			if (result < 0) {
				stop_stream(result, USBD_CC_NOERR);
				break;
			}
			if (result > 0 && !uac_stream_is_active())
				break;
			continue;
		}
		if (result < 0) {
			stop_stream(result, USBD_CC_NOERR);
			break;
		}
		if (result != FEED_QUEUED)
			break;
	}

	if (tap_started) {
		result = audio_tap_end();
		if (result < 0)
			uac_log(LOG_PREFIX
				"virtual Sony DataSend stop failed: 0x%08x\n", result);
	}
	pcm_end();
	return 0;
}

static int reap_feeder(SceUInt timeout_us)
{
	SceUID thread;
	int status;
	int result;

	thread = __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE);
	if (thread < 0)
		return 0;
	result = ksceKernelWaitThreadEnd(thread, &status, &timeout_us);
	if (result < 0)
		return result;
	result = ksceKernelDeleteThread(thread);
	if (result >= 0) {
		__atomic_store_n(&feeder_thread, -1, __ATOMIC_RELEASE);
		finish_retire();
	}
	return result;
}

int uac_stream_start(int pipe_id)
{
	int expected = STREAM_IDLE;
	uint32_t generation;
	uint32_t context_index;
	int result;

	if (pipe_id < 0)
		return -1;
	if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
		SceUInt no_wait = 0;

		if (reap_feeder(no_wait) < 0)
			return -1;
	}
	if (!__atomic_compare_exchange_n(&stream_state, &expected,
		STREAM_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;

	if (free_event < 0 || pcm_event < 0) {
		__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
		return -1;
	}

	generation = next_generation();
	__atomic_store_n(&stream_pipe, pipe_id, __ATOMIC_RELEASE);
	__atomic_store_n(&callback_guard, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&transfer_active, 0, __ATOMIC_RELEASE);
	submit_index = 0u;
	prime_count = 0u;
	__atomic_store_n(&primed, 0, __ATOMIC_RELEASE);
	memset(contexts, 0, sizeof(contexts));

	for (context_index = 0; context_index < CONTEXT_COUNT; ++context_index) {
		StreamContext *context = &contexts[context_index];

		context->generation = generation;
		context->transfer.buffer_base = context->buffer;
		context->transfer.num_packets = 1u;
		__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	}

	reset_event(free_event);
	publish_free_context();

	expected = STREAM_STARTING;
	if (!__atomic_compare_exchange_n(&stream_state, &expected, STREAM_RUNNING,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;

	result = ksceKernelCreateThread("uac_usb_feeder", usb_feeder_thread,
		0x40, 0x3000, 0, 0, NULL);
	if (result < 0) {
		__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
		__atomic_store_n(&stream_pipe, -1, __ATOMIC_RELEASE);
		(void)next_generation();
		return result;
	}
	__atomic_store_n(&feeder_thread, result, __ATOMIC_RELEASE);
	result = ksceKernelStartThread(
		__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE), 0, NULL);
	if (result < 0) {
		(void)ksceKernelDeleteThread(
			__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE));
		__atomic_store_n(&feeder_thread, -1, __ATOMIC_RELEASE);
		__atomic_store_n(&stream_state, STREAM_IDLE, __ATOMIC_RELEASE);
		__atomic_store_n(&stream_pipe, -1, __ATOMIC_RELEASE);
		(void)next_generation();
		return result;
	}

	uac_log(LOG_PREFIX
		"ready: 2 fixed 1-ms contexts; 1 in flight + 1 READY/preparing; "
		"completion-driven A/B rotation; "
		"1x192 bytes/request; no deep runway; virtual native DataSend\n");
	return 0;
}

int uac_stream_is_active(void)
{
	return __atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) == STREAM_RUNNING;
}

void uac_stream_stop(void)
{
	int state;
	int expected;
	uint32_t wait;
	SceUInt feeder_timeout;
	int feeder_result;

	for (;;) {
		state = __atomic_load_n(&stream_state, __ATOMIC_ACQUIRE);
		if (state == STREAM_IDLE) {
			if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
				feeder_timeout = FEEDER_STOP_TIMEOUT_US;
				(void)reap_feeder(feeder_timeout);
			}
			return;
		}
		if (state == STREAM_STOPPING)
			break;
		expected = state;
		if (__atomic_compare_exchange_n(&stream_state, &expected,
			STREAM_STOPPING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			break;
	}

	__atomic_fetch_or(&callback_guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	publish_free_context();
	signal_pcm();

	if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
		feeder_timeout = FEEDER_STOP_TIMEOUT_US;
		feeder_result = reap_feeder(feeder_timeout);
		if (feeder_result < 0)
			uac_log(LOG_PREFIX "USB feeder stop failed: 0x%08x\n",
				feeder_result);
	}

	for (wait = 0; wait < STOP_POLLS; ++wait) {
		if (!(__atomic_load_n(&callback_guard, __ATOMIC_ACQUIRE) & CALLBACK_REFS)) {
			return;
		}
		ksceKernelDelayThread(STOP_POLL_US);
	}
	uac_log(LOG_PREFIX "callback still active after %u ms\n",
		(STOP_POLLS * STOP_POLL_US) / 1000u);
}

void uac_stream_pipe_closed(int pipe_id)
{
	if (pipe_id < 0 ||
	    __atomic_load_n(&stream_pipe, __ATOMIC_ACQUIRE) != pipe_id)
		return;

	__atomic_exchange_n(&stream_state, STREAM_STOPPING, __ATOMIC_ACQ_REL);
	__atomic_fetch_or(&callback_guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	(void)next_generation();
	__atomic_store_n(&stream_pipe, -1, __ATOMIC_RELEASE);
	publish_free_context();
	signal_pcm();
	finish_retire();
}

int uac_stream_init(void)
{
	int event;

	if (free_event < 0) {
		event = ksceKernelCreateEventFlag("uac_usb_free", 0, 0, NULL);
		if (event < 0)
			return event;
		free_event = event;
	}
	if (pcm_event < 0) {
		event = ksceKernelCreateEventFlag("uac_pcm_source", 0, 0, NULL);
		if (event < 0) {
			if (ksceKernelDeleteEventFlag(free_event) >= 0)
				free_event = -1;
			return event;
		}
		pcm_event = event;
	}
	return 0;
}

int uac_stream_shutdown(void)
{
	int result;

	if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
		SceUInt timeout = FEEDER_STOP_TIMEOUT_US;
		publish_free_context();
		signal_pcm();
		result = reap_feeder(timeout);
		if (result < 0)
			return result;
	}
	if (__atomic_load_n(&stream_state, __ATOMIC_ACQUIRE) != STREAM_IDLE)
		return -1;
	pcm_end();
	if (pcm_event >= 0) {
		result = ksceKernelDeleteEventFlag(pcm_event);
		if (result < 0)
			return result;
		pcm_event = -1;
	}
	if (free_event >= 0) {
		result = ksceKernelDeleteEventFlag(free_event);
		if (result < 0)
			return result;
		free_event = -1;
	}
	return 0;
}
