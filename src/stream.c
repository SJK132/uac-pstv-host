/*
 * PCM handoff and USB transport.
 *
 * Two halves that meet in the middle:
 *
 *   - the source side, driven by audio_tap's capture worker, publishes whole
 *     captured blocks into a small seqlock (uac_stream_publish);
 *   - the transport side runs one feeder thread that cuts those blocks into
 *     48-frame (1 ms) packets and keeps exactly one isochronous request in
 *     flight, with the remaining contexts staged behind it.
 *
 * The seqlock is latest-wins, and deliberately not a FIFO.  Producer and
 * consumer run off unrelated clocks -- Sony's audio hardware and the USB host's
 * frame timer -- so they are never rate-matched, and USB completions have been
 * measured stalling for 250 ms at a stretch.  The producer runs straight through
 * such a stall, ending up ~50 blocks ahead.  Latest-wins throws the backlog away
 * and resumes at current audio for the price of one discontinuity; a FIFO would
 * queue every stalled block faithfully and add that 250 ms to output latency
 * permanently, again on each stall.  Anyone reaching for a ring buffer here
 * should read that sentence twice.
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

/* Sony-shaped shallow transport for the tested HS bInterval=4 adapter. */
#define PACKET_FRAMES 48u
#define PACKET_BYTES (PACKET_FRAMES * 4u)
/*
 * Staging depth, in whole captured blocks.  This is history, not queue: the
 * producer never waits for a free slot, it overwrites the oldest unconditionally
 * every block.  So depth buys stall tolerance -- how far the feeder can fall
 * behind before the block it is reading gets pulled out from under it and
 * pcm_resync() has to jump forward, which is audible -- and costs no latency,
 * since latency is fixed by where resync places the cursor.  Power of two;
 * PCM_SLOT_MASK indexes with it.
 */
#define PCM_SLOT_COUNT 4u
#define PCM_SLOT_MASK (PCM_SLOT_COUNT - 1u)
#define SONY_ISO_PACKET_SLOTS 8u
#define TRANSFER_BYTES PACKET_BYTES
/*
 * Transport depth: one request in flight, the rest staged behind it.  Each
 * context is one 1 ms packet, so this is both the feeder's slack -- how long it
 * may be held off before the transport runs out of prepared packets -- and a
 * direct addend to output latency.  Three buys 2 ms of slack for 1 ms of delay.
 */
#define CONTEXT_COUNT 3u

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

/*
 * Feeder scheduling.
 *
 * 0x40 is SCE_KERNEL_HIGHEST_PRIORITY_USER, the bottom of the kernel band, and a
 * poor place for a thread with a 1 ms deadline.  0x20 lifts it clear while
 * staying below the capture worker's 0x12 -- that ordering is deliberate, since
 * the capture worker is the producer and starving it stops everything
 * downstream.  Raising this is safe: the feeder does one CAS and one 192-byte
 * copy per wakeup and then blocks, so it cannot monopolise a core, it only needs
 * waking promptly.
 *
 * The affinity is the more defensible half, and for a cache reason rather than a
 * scheduling one.  Pinned to the capture worker's core (CAPTURE_CPU_MASK in
 * audio_tap.c -- both are SCE_KERNEL_CPU_MASK_USER_3), a freshly published block
 * is still in L1 when the feeder reads it back out; on separate cores that is
 * fifteen line invalidations per block, a few thousand a second.
 *
 * Stated honestly, because the measurements were noisier than they first looked:
 * the priority change moved the starve rate only slightly and within run-to-run
 * variance, and an A/B against 0x40-unpinned produced identical resync counts,
 * so neither setting is implicated in the USB stalls described at the top of
 * this file.  Compare the session line before and after touching either.
 */
#ifndef FEEDER_PRIORITY
#define FEEDER_PRIORITY 0x20
#endif
#ifndef FEEDER_CPU_MASK
#define FEEDER_CPU_MASK 0x80000u
#endif
#define FEEDER_STACK 0x3000u
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

#if CONTEXT_COUNT < 2u
#error Need at least one request in flight and one staged behind it
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

/*
 * One captured block, plus the seqlock words that publish it.  The PCM starts
 * on a cache line so both the producer's memcpy in and the consumer's copy out
 * begin aligned, and guard sits next to the data it guards rather than in a
 * separate array, so validating a read touches the same lines as the read.
 *
 * At the current block size a slot is exactly 1 KiB and the array exactly 4 KiB.
 * Tidy, but incidental -- the array is only 64-aligned, so it is page-sized, not
 * page-aligned, and aligning it to a page was measured to change no instructions
 * for 3.4 KiB of padding.  What the asserts below enforce is the part that would
 * silently cost performance: every slot landing on a line boundary, not just the
 * first, which holds for any block size that is a multiple of PACKET_FRAMES.
 */
typedef struct {
	uint32_t guard;
	uint32_t sequence;
	int16_t pcm[UAC_STREAM_CAPTURE_FRAMES * 2u]
		__attribute__((aligned(64)));
} PcmSlot;

typedef char pcm_slot_must_tile_cache_lines[
	(sizeof(PcmSlot) % 64u == 0u) ? 1 : -1];
typedef char pcm_slot_count_must_be_power_of_two[
	(PCM_SLOT_COUNT != 0u &&
	 (PCM_SLOT_COUNT & PCM_SLOT_MASK) == 0u) ? 1 : -1];

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

/*
 * Session instrumentation, logging builds only.  What each number means, since
 * only two of the four are audible:
 *
 *   starve   the transport won the right to submit and found the next context
 *            unfilled, so the isochronous stream gets a hole.  Audible.
 *   pcm wait the feeder had a context but no PCM to put in it.  Absorbed by the
 *            staged contexts; not audible by itself.
 *   resync   the producer lapped the block being read and the cursor jumped to
 *            current audio.  Audible, and the one that matters most.
 *   margin   blocks the producer is ahead by when the consumer reads.  One is
 *            the design point; zero means it caught up and is about to wait.
 *
 * These swing by orders of magnitude between runs of the same binary -- pcm
 * waits have gone 3695 and 2 on consecutive sessions -- so read patterns across
 * several sessions and never conclude anything from one number.
 */
#ifdef UAC_PSTV_ENABLE_LOGGING
static uint32_t starve_count;
static uint32_t submit_count;
static uint32_t pcm_wait_count;
static uint32_t resync_count;
static uint32_t margin_min = 0xffffffffu;
#define COUNT_STARVE() __atomic_add_fetch(&starve_count, 1u, __ATOMIC_RELAXED)
#define COUNT_SUBMIT() __atomic_add_fetch(&submit_count, 1u, __ATOMIC_RELAXED)
#define COUNT_PCM_WAIT() __atomic_add_fetch(&pcm_wait_count, 1u, __ATOMIC_RELAXED)
/*
 * Where each resync happened, not just how many.  They come in single digits per
 * session, so each one is worth looking at: the gap tells you what caused it --
 * four or five blocks is ordinary jitter, fifty is a USB stall.  Stored rather
 * than logged inline, because uac_log() writes to the filesystem and doing that
 * from pcm_next() would stall the feeder and manufacture the very problem being
 * measured.
 */
#define RESYNC_LOG_MAX 8u
static uint32_t resync_at[RESYNC_LOG_MAX];
static uint32_t resync_gap[RESYNC_LOG_MAX];
#define COUNT_RESYNC(gap) do { \
	uint32_t n = __atomic_fetch_add(&resync_count, 1u, __ATOMIC_RELAXED); \
	if (n < RESYNC_LOG_MAX) { \
		resync_at[n] = __atomic_load_n(&submit_count, __ATOMIC_RELAXED); \
		resync_gap[n] = (gap); \
	} \
} while (0)
#define TRACK_MARGIN(m) do { \
	uint32_t seen = (m); \
	uint32_t old = __atomic_load_n(&margin_min, __ATOMIC_RELAXED); \
	while (seen < old && \
	       !__atomic_compare_exchange_n(&margin_min, &old, seen, 1, \
			__ATOMIC_RELAXED, __ATOMIC_RELAXED)) \
		; \
} while (0)
#else
#define COUNT_STARVE() ((void)0)
#define COUNT_SUBMIT() ((void)0)
#define COUNT_PCM_WAIT() ((void)0)
#define COUNT_RESYNC(gap) ((void)0)
#define TRACK_MARGIN(m) ((void)0)
#endif

/*
 * Producer state, written by audio_tap's capture worker; the consumer cursor
 * below is the feeder's alone.  Each starts a cache line so neither can
 * invalidate the other.  Note that with FEEDER_CPU_MASK pinning both threads to
 * one core this currently buys nothing -- keep it as insurance against that
 * pinning changing, not as an active optimisation.  Costs padding, not
 * instructions.
 */
static int pcm_enabled __attribute__((aligned(64)));
static int pcm_producer_busy;
static uint32_t pcm_latest;

/* Consumer state belongs exclusively to the feeder thread. */
static uint32_t pcm_sequence __attribute__((aligned(64)));
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

	slot = &pcm_slots[sequence & PCM_SLOT_MASK];
	/*
	 * Mark the slot odd, then fence.  All this needs is for the marker to be
	 * visible before the data writes that follow, which is a release fence
	 * after a relaxed store: one dmb.  A seq_cst store would compile to
	 * dmb-str-dmb and buy nothing -- there is a single producer, so there is
	 * no second writer for total order to arbitrate between.  This is the
	 * write_seqlock() half.
	 */
	__atomic_store_n(&slot->guard, (sequence << 1) | 1u, __ATOMIC_RELAXED);
	__atomic_thread_fence(__ATOMIC_RELEASE);
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
	uint32_t index;

	for (index = 0; index < CONTEXT_COUNT; ++index) {
		if (__atomic_load_n(&contexts[index].state, __ATOMIC_ACQUIRE) ==
		    CONTEXT_FREE)
			return 1;
	}
	return 0;
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
		/* Nothing staged: the feeder fell behind and the stream gaps. */
		COUNT_STARVE();
		__atomic_store_n(&transfer_active, 0, __ATOMIC_RELEASE);
		return FEED_QUEUED;
	}
	COUNT_SUBMIT();
	/* Rotate by compare rather than modulo: CONTEXT_COUNT need not be a
	 * power of two, and a divide has no business on the 1 ms path. */
	if (++submit_index == CONTEXT_COUNT)
		submit_index = 0u;

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
		uint32_t index;
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

		/*
		 * First fit, which is also submit order -- and it has to be,
		 * because filling out of order would transmit packets out of
		 * order.  Only one request is ever in flight, so in steady
		 * state exactly one context is FREE: the one that just
		 * completed, which is precisely the one pump_ready() reaches
		 * next.  During priming every context is FREE and this fills
		 * 0..N-1, matching submit_index starting at zero.
		 */
		for (index = 0; index < CONTEXT_COUNT; ++index) {
			expected = CONTEXT_FREE;
			if (__atomic_compare_exchange_n(&contexts[index].state,
				&expected, CONTEXT_WRITING, 0, __ATOMIC_ACQ_REL,
				__ATOMIC_ACQUIRE)) {
				*out = &contexts[index];
				break;
			}
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
	uac_log("[uac-pstv-source] armed: %ux%u PCM staging; "
		"%u-frame packetizer\n", PCM_SLOT_COUNT,
		UAC_STREAM_CAPTURE_FRAMES, PACKET_FRAMES);
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
	PcmSlot *slot = &pcm_slots[sequence & PCM_SLOT_MASK];
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
		/*
		 * Fence first, then a relaxed read -- not an acquire load.
		 * Acquire orders what comes after it; what has to be ordered
		 * here is the memcpy above, which must be complete before the
		 * guard is sampled again.  An acquire load puts its barrier on
		 * the far side and leaves the copy free to be reordered past
		 * the check, which is how a torn read passes validation.  Same
		 * two instructions either way; only the order differs.  This is
		 * the read_seqretry() half of the pattern.
		 */
		__atomic_thread_fence(__ATOMIC_ACQUIRE);
		if (__atomic_load_n(&slot->guard, __ATOMIC_RELAXED) == before)
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
	/* How far ahead the producer is right now; zero means we caught up. */
	TRACK_MARGIN(latest - pcm_sequence);

	if (latest >= pcm_sequence + PCM_SLOT_COUNT) {
		/* Lapped: the block being read got overwritten. Audible jump. */
		COUNT_RESYNC(latest - pcm_sequence);
		pcm_resync(latest);
	}

	if (!pcm_copy(pcm_sequence, pcm_offset, packet)) {
		latest = __atomic_load_n(&pcm_latest, __ATOMIC_ACQUIRE);
		if (latest >= pcm_sequence + PCM_SLOT_COUNT) {
			COUNT_RESYNC(latest - pcm_sequence);
			pcm_resync(latest);
		}
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

	/* The transport stays idle until every context holds a packet. */
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
			/*
			 * The feeder had a context but no PCM to put in it.  If
			 * this tracks the starve count, the shortfall is on the
			 * producer side and feeder priority is irrelevant.
			 */
			COUNT_PCM_WAIT();
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
#ifdef UAC_PSTV_ENABLE_LOGGING
	uac_log(LOG_PREFIX
		"session: %u packets, %u starved, %u pcm waits, "
		"%u resyncs, min margin %u blocks\n",
		__atomic_load_n(&submit_count, __ATOMIC_RELAXED),
		__atomic_load_n(&starve_count, __ATOMIC_RELAXED),
		__atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED),
		__atomic_load_n(&resync_count, __ATOMIC_RELAXED),
		__atomic_load_n(&margin_min, __ATOMIC_RELAXED));
	__atomic_store_n(&submit_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&starve_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&pcm_wait_count, 0u, __ATOMIC_RELAXED);
	{
		uint32_t total = __atomic_load_n(&resync_count, __ATOMIC_RELAXED);
		uint32_t shown = total < RESYNC_LOG_MAX ? total : RESYNC_LOG_MAX;
		uint32_t i;

		for (i = 0; i < shown; ++i)
			uac_log(LOG_PREFIX
				"  resync %u at packet %u, producer was %u blocks ahead\n",
				i + 1u, resync_at[i], resync_gap[i]);
	}
	__atomic_store_n(&resync_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&margin_min, 0xffffffffu, __ATOMIC_RELAXED);
#endif
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
		FEEDER_PRIORITY, FEEDER_STACK, 0, FEEDER_CPU_MASK, NULL);
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
		"ready: %u fixed 1-ms contexts; 1 in flight + %u staged; "
		"completion-driven rotation; 1x%u bytes/request; "
		"virtual native DataSend\n",
		CONTEXT_COUNT, CONTEXT_COUNT - 1u, TRANSFER_BYTES);
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
