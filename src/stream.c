/*
 * PCM handoff and USB transport.
 *
 * Two halves that meet in the middle:
 *
 *   - the source side, driven by audio_tap's capture worker, publishes whole
 *     captured blocks into a small seqlock (uac_stream_publish);
 *   - the transport side runs one feeder thread that cuts those blocks into
 *     48-frame (1 ms) packets and keeps three isochronous requests in flight,
 *     with one context staged behind them.
 *
 * The seqlock is latest-wins, and deliberately not a FIFO.  Producer and
 * consumer run off unrelated clocks -- Sony's audio hardware and the USB host's
 * frame timer -- so they are never rate-matched, and USB completions have been
 * measured stalling for 250 ms at a stretch.  The producer runs straight through
 * such a stall, ending up ~50 blocks ahead.  Latest-wins throws the backlog away
 * and resumes at current audio for the price of one discontinuity; a FIFO would
 * queue every stalled block faithfully and add that 250 ms to output latency
 * permanently, again on each stall.
 */

#include "stream.h"
#include "log.h"
#include "uac1.h"

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
 * Transport depth.
 *
 * Keep three requests owned by USBD and one READY context.  The fourth context
 * lets the feeder prepare the next millisecond without touching DMA-owned
 * storage, while three queued frames keep SceUsbd's periodic cursor ahead.
 */
#define CONTEXT_COUNT 4u
#define MAX_IN_FLIGHT 3u

/*
 * USBD may deliver a completion after its pipe has been closed and the static
 * contexts have been reused by a later session.  Never pass a context pointer
 * as the callback identity: its generation is mutable.  The opaque callback
 * argument instead carries the session generation in its upper bits and the
 * context index in its lower two bits, so the submitted identity is immutable.
 */
#define CALLBACK_CONTEXT_MASK (CONTEXT_COUNT - 1u)
#define CALLBACK_GENERATION_MASK (~CALLBACK_CONTEXT_MASK)

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
 * downstream.  Raising this is safe: per wakeup the feeder claims a context,
 * copies 192 bytes, and blocks again -- bounded work either way, so it cannot
 * monopolise a core.  It only needs waking promptly.
 *
 * The affinity is the more defensible half, and for a cache reason rather than a
 * scheduling one.  Pinned to the capture worker's core (CAPTURE_CPU_MASK in
 * audio_tap.c -- both are SCE_KERNEL_CPU_MASK_USER_3), a freshly published block
 * is still in L1 when the feeder reads it back out; on separate cores a 960-byte
 * block is thirty line invalidations, six thousand a second.  (Thirty, not
 * fifteen: this core's line is 32 bytes.  See UAC_ERG.)
 */
#define FEEDER_PRIORITY 0x20
#define FEEDER_CPU_MASK 0x80000u
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

/*
 * UAC_ERG is the Cortex-A9 exclusive-reservation granule, which on this core is
 * also the L1 line length.  Both are 32 bytes; do not carry a 64 over from a
 * later Cortex, and do not read "cache line" anywhere below as anything but 32.
 *
 * It matters because a reservation covers the granule, not the word: a plain
 * store by any core into a granule clears an ldrex another core holds on it.
 * So the two things that must never share one are a word written every
 * millisecond by one thread and a word ldrex/strex'd every millisecond by
 * another -- the second one's CAS loop then retries for nothing.
 */
#define UAC_ERG 32u

typedef char erg_must_be_a_power_of_two[
	(UAC_ERG != 0u && (UAC_ERG & (UAC_ERG - 1u)) == 0u) ? 1 : -1];

#if TRANSFER_BYTES != 192u
#error Tested transport must remain one 192-byte packet per request
#endif

#if CONTEXT_COUNT < 2u
#error Need at least one request in flight and one staged behind it
#endif

#if CONTEXT_COUNT != 4u
#error Callback token encoding requires four contexts
#endif

#if UAC_STREAM_CAPTURE_FRAMES % PACKET_FRAMES != 0u
#error Native blocks must contain a whole number of USB packets
#endif

#if UAC_STREAM_CAPTURE_BYTES > 0x800u
#error Native blocks must fit the Sony RAM-output page
#endif

enum {
	STREAM_IDLE = 0,
	STREAM_STARTING,
	STREAM_RUNNING,
	STREAM_STOPPING,
};

/*
 * IN_FLIGHT exists because READY cannot mean two things at once.  While only one
 * request was outstanding a submitted context kept READY until its completion
 * set it FREE, and nothing noticed because the single-slot guard blocked every
 * other submit.  With several outstanding, READY must remain distinct from a
 * buffer that USBD still owns, or DMA storage can be submitted twice.
 */
enum {
	CONTEXT_FREE = 0,
	CONTEXT_WRITING,
	CONTEXT_READY,
	CONTEXT_IN_FLIGHT,
};

enum {
	FEEDER_NONE = 0,
	FEEDER_STARTING,
	FEEDER_RUNNING,
	FEEDER_DORMANT,
	FEEDER_REAPING,
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

/*
 * CONTEXT_ALIGN is a DMA constraint, not a cache one.  buffer[] is the only
 * storage in this driver the host controller reads directly, and EHCI addresses
 * transfer buffers as a page plus an offset: a buffer that straddles a 4 KiB
 * boundary forces the second page pointer in the descriptor to be programmed as
 * well.  Aligning each context to a divisor of the page size, with the buffer
 * first and no larger than that alignment, puts every buffer inside a single
 * page by construction.  The asserts below are what make it a guarantee rather
 * than an accident of where BSS happened to land -- which is what it was before
 * this alignment existed, with the whole array sitting at page offset 448 and
 * nothing stopping the next added variable from pushing it over a boundary.
 */
#define CONTEXT_ALIGN 256u

typedef struct {
	uint8_t buffer[TRANSFER_BYTES] __attribute__((aligned(64)));
	SonyIsoTransfer transfer __attribute__((aligned(64)));
	uint32_t callback_token;
	uint32_t sequence;
	int state;
} StreamContext;

typedef char context_must_tile_its_alignment[
	(sizeof(StreamContext) == CONTEXT_ALIGN) ? 1 : -1];
typedef char context_buffer_must_fit_one_block[
	(TRANSFER_BYTES <= CONTEXT_ALIGN) ? 1 : -1];
typedef char context_blocks_must_tile_a_page[
	(4096u % CONTEXT_ALIGN == 0u) ? 1 : -1];

/*
 * One captured block, plus the seqlock words that publish it.  The PCM starts
 * on a cache line so both the producer's memcpy in and the consumer's copy out
 * begin aligned, and guard sits next to the data it guards rather than in a
 * separate array, so validating a read touches the same lines as the read.
 *
 * The 64 below is two lines, not one, and is deliberate: it puts the payload at
 * offset 64 so a slot comes to exactly 1 KiB and the array to exactly 4 KiB at
 * the current block size.  Only the 32 matters for correctness -- UAC_ERG would
 * do, at 992-byte slots -- so do not read the 64 as a claim about line size.
 *
 * The array is page-sized but not page-aligned, and aligning it to a page was
 * measured to change no instructions for 3.4 KiB of padding.  What the asserts
 * enforce is the part that would silently cost performance instead: every slot
 * landing on a line, not just the first, which holds for any block size that is
 * a whole number of packets.
 */
typedef struct {
	uint32_t guard;
	uint32_t sequence;
	int16_t pcm[UAC_STREAM_CAPTURE_FRAMES * 2u]
		__attribute__((aligned(64)));
} PcmSlot;

/*
 * Two separate properties, and only the first is obvious.  The stride assert
 * is what makes every slot start on a line rather than only the first, and the
 * offset assert is what keeps each slot's PCM itself line-aligned -- both hold
 * for any block size that is a whole number of packets, which is the real
 * constraint, but neither is visible at the point where someone would change
 * UAC_STREAM_CAPTURE_FRAMES.
 */
typedef char pcm_slot_stride_must_tile_lines[
	(sizeof(PcmSlot) % UAC_ERG == 0u) ? 1 : -1];
typedef char pcm_slot_payload_must_start_on_a_line[
	(offsetof(PcmSlot, pcm) % UAC_ERG == 0u) ? 1 : -1];
typedef char pcm_slot_count_must_be_power_of_two[
	(PCM_SLOT_COUNT != 0u &&
	 (PCM_SLOT_COUNT & PCM_SLOT_MASK) == 0u) ? 1 : -1];

static StreamContext contexts[CONTEXT_COUNT]
	__attribute__((aligned(CONTEXT_ALIGN)));
static PcmSlot pcm_slots[PCM_SLOT_COUNT] __attribute__((aligned(UAC_ERG)));

/*
 * Shared state, grouped by who touches it and padded to own its granule.
 *
 * Aligning the variables alone is not enough.  Alignment fixes where an object
 * starts, not how much it occupies, so GCC stays free to pack the next scalar
 * into the tail.  Aligning the type makes sizeof a multiple of that alignment,
 * which is what actually reserves the granule; the asserts pin it.
 *
 * The attribute has to sit before the typedef name, as written below.  Putting
 * it after -- "} TxPump __attribute__((aligned(32)))" -- reads the same and is
 * not: that applies to the typedef declarator, giving alignof 32 while leaving
 * sizeof at 24, which is the exact hole this is meant to close.  The asserts
 * are here because that mistake was made writing this comment block.
 *
 * This is not hypothetical.  These words were laid out by declaration order and
 * happened to be grouped correctly, until an unrelated change shifted GCC's
 * anchor grouping and dropped the per-millisecond PCM cursor into the same
 * granule as in_flight and pump_requests.  Nothing warned, because nothing had
 * ever stated the requirement.
 *
 * New shared state goes inside one of these, not at file scope.
 */

/* Read or updated by both the feeder and the USBD callback on every packet. */
typedef struct {
	int state;
	uint32_t generation;
	int pipe;
	/* Requests USBD owns. Raised inside pump_ready, lowered by the completion. */
	uint32_t in_flight;
	/* Coalesces feeder and callback requests while admitting one pumper. */
	uint32_t pump_requests;
	int primed;
} __attribute__((aligned(UAC_ERG))) TxPump;

/*
 * Callback-thread private in steady state: only callback_enter/leave touch
 * these per packet, and they are the reason this group is not folded into
 * TxPump.  Sharing a granule would let the feeder's pump_requests RMW clear the
 * callback's guard reservation every millisecond, a collision that does not
 * otherwise exist.  feeder_state rides along because it is teardown-only.
 */
typedef struct {
	uint32_t guard;
	int feeder_state;
} __attribute__((aligned(UAC_ERG))) CallbackLifetime;

/*
 * Producer to consumer.  Both live on CAPTURE_CPU_MASK, so this group carries
 * no cross-core traffic at all; it is grouped to keep it out of TxPump's
 * granule, not for any benefit of its own.
 */
typedef struct {
	int enabled;
	int producer_busy;
	uint32_t latest;
} __attribute__((aligned(UAC_ERG))) PcmSource;

/*
 * Feeder-private packetizer cursor.  The callback never reads any of it, which
 * is exactly why it must not sit next to what the callback does ldrex on: these
 * are plain stores, once per millisecond, and they were clearing reservations
 * on in_flight and pump_requests for no reason at all.
 *
 * write_sequence is the ticket stamped on each staged packet.
 * claim_oldest_ready() submits by it rather than by slot, so PCM order survives
 * contexts completing and being reused out of slot order -- which is what a
 * fixed slot rotation gets wrong, stalling on one particular context while
 * another sits ready.  It also serves as the priming count.
 */
typedef struct {
	uint32_t sequence;
	uint32_t offset;
	uint32_t write_sequence;
	int valid;
} __attribute__((aligned(UAC_ERG))) PcmCursor;

/*
 * Both halves are needed and they fail differently.  sizeof catches the group
 * outgrowing its granule, or the aligned attribute being written in the place
 * that raises alignof without padding.  _Alignof catches the group starting
 * mid-granule, which would straddle two of them however big it is.
 */
#define MUST_OWN_ONE_GRANULE(type, tag) \
	typedef char tag##_must_fill_one_granule[ \
		(sizeof(type) == UAC_ERG) ? 1 : -1]; \
	typedef char tag##_must_start_on_a_granule[ \
		(_Alignof(type) == UAC_ERG) ? 1 : -1]

MUST_OWN_ONE_GRANULE(TxPump, tx_pump);
MUST_OWN_ONE_GRANULE(CallbackLifetime, callback_lifetime);
MUST_OWN_ONE_GRANULE(PcmSource, pcm_source);
MUST_OWN_ONE_GRANULE(PcmCursor, pcm_cursor);

static TxPump tx = { .pipe = -1 };
static CallbackLifetime cb;
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
/*
 * Deliberately left at file scope rather than grouped: all three are written
 * only by init and teardown, so in steady state they are read-only and cannot
 * be invalidated by anything.  Where they land does not matter.  Give one of
 * them a per-packet writer and that stops being true.
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

/*
 * Completions arriving a very long time after the previous one.  Measured at
 * one full pass of a 256-entry periodic frame list, freezing the whole pipe
 * rather than one descriptor, and each one followed by a resync.  Depth is
 * therefore not a lever: a fourth request in flight would freeze with the rest.
 *
 * Only transfer_done() touches this, on USBD's single callback thread, so plain
 * accesses suffice; the counters it samples are relaxed atomics because
 * pump_ready() also runs on the feeder.
 */
#define STALL_LOG_MAX 8u
#define STALL_THRESHOLD_US 50000u
static uint32_t completion_at_us;
static int completion_seen;
static uint32_t completion_gap_max;
static uint32_t stall_count;
static uint32_t stall_at[STALL_LOG_MAX];
static uint32_t stall_gap[STALL_LOG_MAX];
static uint32_t stall_starves[STALL_LOG_MAX];
static uint32_t starve_at_completion;

static void track_completion_gap(void)
{
	uint32_t now = ksceKernelGetSystemTimeLow();
	uint32_t starves = __atomic_load_n(&starve_count, __ATOMIC_RELAXED);
	uint32_t gap;

	if (!completion_seen) {
		completion_seen = 1;
		completion_at_us = now;
		starve_at_completion = starves;
		return;
	}
	/* Unsigned difference stays correct across the counter's ~71 min wrap. */
	gap = now - completion_at_us;
	completion_at_us = now;
	if (gap > completion_gap_max)
		completion_gap_max = gap;
	if (gap >= STALL_THRESHOLD_US) {
		if (stall_count < STALL_LOG_MAX) {
			stall_at[stall_count] =
				__atomic_load_n(&submit_count, __ATOMIC_RELAXED);
			stall_gap[stall_count] = gap;
			stall_starves[stall_count] = starves - starve_at_completion;
		}
		++stall_count;
	}
	starve_at_completion = starves;
}
#define TRACK_COMPLETION_GAP() track_completion_gap()
#else
#define COUNT_STARVE() ((void)0)
#define COUNT_SUBMIT() ((void)0)
#define COUNT_PCM_WAIT() ((void)0)
#define COUNT_RESYNC(gap) ((void)0)
#define TRACK_MARGIN(m) ((void)0)
#define TRACK_COMPLETION_GAP() ((void)0)
#endif

/* Producer state, written by audio_tap's capture worker. */
static PcmSource src;

/* Consumer state belongs exclusively to the feeder thread. */
static PcmCursor cur;

static uint32_t next_generation(void)
{
	/* Keep the context-index bits clear; skip zero when the token wraps. */
	uint32_t value = __atomic_add_fetch(&tx.generation, CONTEXT_COUNT,
		__ATOMIC_ACQ_REL);

	return value ? value : __atomic_add_fetch(&tx.generation, CONTEXT_COUNT,
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
		__atomic_load_n(&src.latest, __ATOMIC_RELAXED));

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
	__atomic_store_n(&src.latest, sequence, __ATOMIC_RELEASE);
	signal_pcm();
}

int uac_stream_publish(const void *pcm)
{
	if (pcm == NULL || !__atomic_load_n(&src.enabled, __ATOMIC_ACQUIRE))
		return 0;
	if (__atomic_exchange_n(&src.producer_busy, 1, __ATOMIC_ACQUIRE))
		return 0;
	if (__atomic_load_n(&src.enabled, __ATOMIC_ACQUIRE))
		pcm_write(pcm);
	__atomic_store_n(&src.producer_busy, 0, __ATOMIC_RELEASE);
	return 1;
}

/* Four-element scan; cheaper than any index the two callers would have to keep. */
static int any_context_in(int state)
{
	uint32_t index;

	for (index = 0; index < CONTEXT_COUNT; ++index) {
		if (__atomic_load_n(&contexts[index].state, __ATOMIC_ACQUIRE) == state)
			return 1;
	}
	return 0;
}

static void finish_retire(void)
{
	uint32_t expected = CALLBACK_CLOSING;

	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_STOPPING &&
	    __atomic_load_n(&tx.pipe, __ATOMIC_ACQUIRE) < 0 &&
	    __atomic_load_n(&cb.feeder_state, __ATOMIC_ACQUIRE) == FEEDER_NONE &&
	    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) < 0 &&
	    __atomic_compare_exchange_n(&cb.guard, &expected, 0u, 0,
		__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		__atomic_store_n(&tx.in_flight, 0u, __ATOMIC_RELAXED);
		__atomic_store_n(&tx.pump_requests, 0u, __ATOMIC_RELAXED);
		__atomic_store_n(&tx.primed, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&tx.state, STREAM_IDLE, __ATOMIC_RELEASE);
		publish_free_context();
	}
}

static void callback_leave(void)
{
	if (__atomic_fetch_sub(&cb.guard, 1, __ATOMIC_RELEASE) ==
	    (CALLBACK_CLOSING | 1u))
		finish_retire();
}

static int callback_enter(uint32_t generation)
{
	uint32_t guard;

	for (;;) {
		guard = __atomic_load_n(&cb.guard, __ATOMIC_ACQUIRE);
		if (guard & CALLBACK_CLOSING)
			return 0;
		if (__atomic_compare_exchange_n(&cb.guard, &guard, guard + 1u,
			0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
			break;
	}
	if (__atomic_load_n(&tx.generation, __ATOMIC_ACQUIRE) == generation)
		return 1;
	callback_leave();
	return -1;
}

static void stop_stream(int result, uint16_t status)
{
	int previous;

	for (;;) {
		previous = __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE);
		if (previous != STREAM_RUNNING && previous != STREAM_STARTING)
			return;
		if (__atomic_compare_exchange_n(&tx.state, &previous,
			STREAM_STOPPING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			break;
	}

	__atomic_fetch_or(&cb.guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	publish_free_context();
	signal_pcm();
	uac1_stream_failed();
	if (result < 0)
		uac_log(LOG_PREFIX "transfer failed: 0x%08x\n", result);
	else if (status != USBD_CC_NOERR)
		uac_log(LOG_PREFIX "packet failed: status 0x%04x\n", status);
}

void uac_stream_source_failed(int result)
{
	stop_stream(result < 0 ? result : -1, USBD_CC_NOERR);
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *transfer,
	void *arg);

static int submit_context(StreamContext *context)
{
	uint32_t generation = context->callback_token & CALLBACK_GENERATION_MASK;
	int pipe;

	context->transfer.packets[0].len = PACKET_BYTES;
	context->transfer.packets[0].status = 0;

	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
	    __atomic_load_n(&tx.generation, __ATOMIC_ACQUIRE) != generation)
		return 1;
	pipe = __atomic_load_n(&tx.pipe, __ATOMIC_ACQUIRE);
	if (pipe < 0)
		return 1;

	/* After the aborts, not before: a teardown should not pay for a flush. */
	ksceKernelDcacheCleanRange(context->buffer, TRANSFER_BYTES);
	return ksceUsbdIsochronousTransfer(
		pipe,
		(ksceUsbdIsochTransfer *)(void *)&context->transfer,
		transfer_done,
		(void *)(uintptr_t)context->callback_token);
}

static StreamContext *claim_oldest_ready(void)
{
	StreamContext *best = NULL;
	uint32_t best_sequence = 0;
	uint32_t index;
	int expected;

	for (index = 0; index < CONTEXT_COUNT; ++index) {
		StreamContext *context = &contexts[index];

		if (__atomic_load_n(&context->state, __ATOMIC_ACQUIRE) !=
		    CONTEXT_READY)
			continue;
		/*
		 * Wrap-safe ordering: a plain < picks the wrong packet for one
		 * comparison once cur.write_sequence wraps, which at one packet per
		 * millisecond is about every 50 days of continuous playback.
		 * The signed difference stays correct across the wrap and costs
		 * the same single subtraction.
		 */
		if (best == NULL ||
		    (int32_t)(context->sequence - best_sequence) < 0) {
			best = context;
			best_sequence = context->sequence;
		}
	}
	if (best == NULL)
		return NULL;
	expected = CONTEXT_READY;
	if (!__atomic_compare_exchange_n(&best->state, &expected,
		CONTEXT_IN_FLIGHT, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return NULL;
	return best;
}

static int pump_ready(void)
{
	int result = FEED_QUEUED;

	/*
	 * The first requester owns the drain.  Later requesters only add a ticket;
	 * the owner consumes every ticket before leaving.  Unlike a boolean gate,
	 * the same atomic operation both publishes work and hands ownership over,
	 * so a READY transition cannot land between an owner's final scan and its
	 * unlock and be lost there.
	 */
	if (__atomic_fetch_add(&tx.pump_requests, 1u, __ATOMIC_ACQ_REL) != 0u)
		return result;

	for (;;) {
		while (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) ==
		       STREAM_RUNNING &&
		       __atomic_load_n(&tx.primed, __ATOMIC_ACQUIRE) &&
		       __atomic_load_n(&tx.in_flight, __ATOMIC_ACQUIRE) <
		       MAX_IN_FLIGHT) {
			StreamContext *context = claim_oldest_ready();
			int submitted;

			if (context == NULL)
				break;
			__atomic_add_fetch(&tx.in_flight, 1u, __ATOMIC_ACQ_REL);
			COUNT_SUBMIT();
			submitted = submit_context(context);
			if (submitted == 0)
				continue;

			/*
			 * Declined: no callback is coming, so reclaim it.
			 * RELEASE, not ACQ_REL -- see the note in transfer_done.
			 * The release store below supplies the acquire half, so
			 * do not put a plain access between the two.
			 */
			__atomic_sub_fetch(&tx.in_flight, 1u, __ATOMIC_RELEASE);
			__atomic_store_n(&context->state, CONTEXT_FREE,
				__ATOMIC_RELEASE);
			publish_free_context();
			stop_stream(submitted < 0 ? submitted : 0, USBD_CC_NOERR);
			result = submitted < 0 ? submitted : FEED_STREAM_ENDED;
			break;
		}

		/* A hole is the schedule running dry, not merely being full. */
		if (result == FEED_QUEUED &&
		    __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_RUNNING &&
		    __atomic_load_n(&tx.primed, __ATOMIC_ACQUIRE) &&
		    __atomic_load_n(&tx.in_flight, __ATOMIC_ACQUIRE) == 0u)
			COUNT_STARVE();

		/* The last consumed ticket releases ownership. */
		if (__atomic_fetch_sub(&tx.pump_requests, 1u, __ATOMIC_ACQ_REL) == 1u)
			return result;
	}
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *sdk_transfer,
	void *arg)
{
	uint32_t token = (uint32_t)(uintptr_t)arg;
	uint32_t generation = token & CALLBACK_GENERATION_MASK;
	StreamContext *context;
	SonyIsoTransfer *transfer;
	uint16_t status;
	int entered;

	entered = callback_enter(generation);
	if (entered <= 0)
		return;
	context = &contexts[token & CALLBACK_CONTEXT_MASK];
	transfer = (SonyIsoTransfer *)(void *)sdk_transfer;
	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_RUNNING)
		goto out;

	/*
	 * Before pump_ready(), deliberately.  A starve raised by this callback's
	 * own drain belongs to the gap that is about to open, not to the one that
	 * just closed, and sampling it afterwards would misattribute every one.
	 */
	TRACK_COMPLETION_GAP();

	status = transfer->packets[0].status;
	if (result < 0 || status != USBD_CC_NOERR) {
		stop_stream(result, status);
		goto out;
	}

	__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	/*
	 * RELEASE rather than ACQ_REL, and the difference is one dmb per
	 * completion.  ARM maps an acquire-release RMW to dmb-ldrex/strex-dmb,
	 * so an ACQ_REL here followed by pump_ready()'s ACQ_REL ticket
	 * increment emitted two barriers back to back with nothing but address
	 * arithmetic between them -- the second one already orders everything
	 * the first was there for.
	 *
	 * What the dropped acquire half would have ordered is the loads that
	 * follow, and those all sit behind the ticket increment's own leading
	 * barrier.  That makes this correct only because the next statement is
	 * an acquire-release atomic: put a plain load between them and the
	 * ordering is gone with nothing to warn you.
	 */
	__atomic_sub_fetch(&tx.in_flight, 1u, __ATOMIC_RELEASE);
	/*
	 * Submit first, wake second, and not the other way round.
	 *
	 * Both orderings end with the same work done, but only one of them keeps
	 * the schedule fed.  publish_free_context() is a kernel call that can
	 * reschedule on the spot -- the feeder sits at FEEDER_PRIORITY on
	 * FEEDER_CPU_MASK, so waking it can preempt this callback before the
	 * resubmit happens -- and the resubmit is the deadline-bearing half:
	 * ksceUsbdIsochronousTransfer() has been measured taking 260-505 us of a
	 * 1 ms frame, so anything that delays it eats the whole margin.  Nothing
	 * the feeder can do is urgent by comparison; the context it is being
	 * woken for went FREE above and stays FREE.
	 */
	(void)pump_ready();
	publish_free_context();

out:
	callback_leave();
}

static int wait_for_free_context(uint32_t generation, StreamContext **out)
{
	*out = NULL;

	for (;;) {
		uint32_t index;
		uint32_t matched_bits;
		int result;

		if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
		    __atomic_load_n(&tx.generation, __ATOMIC_ACQUIRE) != generation)
			return FEED_STREAM_ENDED;

		result = ksceKernelWaitEventFlag(free_event, FREE_EVENT_BIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched_bits, NULL);
		if (result < 0)
			return result;

		if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_RUNNING ||
		    __atomic_load_n(&tx.generation, __ATOMIC_ACQUIRE) != generation)
			return FEED_STREAM_ENDED;

		for (index = 0; index < CONTEXT_COUNT; ++index) {
			/* The feeder is the sole FREE claimant. */
			if (__atomic_load_n(&contexts[index].state,
				__ATOMIC_ACQUIRE) == CONTEXT_FREE) {
				__atomic_store_n(&contexts[index].state,
					CONTEXT_WRITING, __ATOMIC_RELAXED);
				*out = &contexts[index];
				if (any_context_in(CONTEXT_FREE))
					publish_free_context();
				return FEED_QUEUED;
			}
		}
	}
}

static void pcm_begin(void)
{
	__atomic_store_n(&src.enabled, 0, __ATOMIC_RELEASE);
	while (__atomic_load_n(&src.producer_busy, __ATOMIC_ACQUIRE))
		ksceKernelDelayThread(50);

	memset(pcm_slots, 0, sizeof(pcm_slots));
	__atomic_store_n(&src.latest, 0, __ATOMIC_RELAXED);
	cur.sequence = 0;
	cur.offset = 0;
	cur.valid = 0;
	reset_event(pcm_event);
	__atomic_store_n(&src.enabled, 1, __ATOMIC_RELEASE);
	uac_log("[uac-pstv-source] armed: %ux%u PCM staging; "
		"%u-frame packetizer\n", PCM_SLOT_COUNT,
		UAC_STREAM_CAPTURE_FRAMES, PACKET_FRAMES);
}

static void pcm_end(void)
{
	__atomic_store_n(&src.enabled, 0, __ATOMIC_RELEASE);
	signal_pcm();
	while (__atomic_load_n(&src.producer_busy, __ATOMIC_ACQUIRE))
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
	/* Sequence 1 is startup when invalid, and follows UINT32_MAX at wrap. */
	cur.sequence = latest == 1u ? (cur.valid ? UINT32_MAX : 1u) : latest - 1u;
	cur.offset = 0;
	cur.valid = latest != 0;
}

/* Forward distance on the 1..UINT32_MAX sequence ring; zero if not ahead. */
static uint32_t pcm_ahead_by(uint32_t newer, uint32_t older)
{
	uint32_t distance = newer - older;

	if ((int32_t)distance <= 0)
		return 0;
	return newer < older ? distance - 1u : distance;
}

/* Return zero for a packet, one when no packet is ready, or a negative error. */
static int pcm_next(uint8_t packet[PACKET_BYTES])
{
	uint32_t latest = __atomic_load_n(&src.latest, __ATOMIC_ACQUIRE);
	uint32_t gap;

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
	if (!cur.valid) {
		if (latest < 2u) {
			memset(packet, 0, PACKET_BYTES);
			return 0;
		}
		pcm_resync(latest);
	}
	/* How far ahead the producer is right now; zero means we caught up. */
	gap = pcm_ahead_by(latest, cur.sequence);
	TRACK_MARGIN(gap);

	if (gap >= PCM_SLOT_COUNT) {
		/* Lapped: the block being read got overwritten. Audible jump. */
		COUNT_RESYNC(gap);
		pcm_resync(latest);
	}

	if (!pcm_copy(cur.sequence, cur.offset, packet)) {
		latest = __atomic_load_n(&src.latest, __ATOMIC_ACQUIRE);
		gap = pcm_ahead_by(latest, cur.sequence);
		if (gap >= PCM_SLOT_COUNT) {
			COUNT_RESYNC(gap);
			pcm_resync(latest);
		}
		if (!pcm_copy(cur.sequence, cur.offset, packet))
			return 1;
	}

	cur.offset += PACKET_FRAMES;
	if (cur.offset == UAC_STREAM_CAPTURE_FRAMES) {
		cur.sequence = next_pcm_sequence(cur.sequence);
		cur.offset = 0;
	}
	return 0;
}

static int pcm_wait(void)
{
	uint32_t matched;
	int result;

	if (!__atomic_load_n(&src.enabled, __ATOMIC_ACQUIRE))
		return 1;
	result = ksceKernelWaitEventFlag(pcm_event, PCM_EVENT_BIT,
		SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, NULL);
	if (result < 0)
		return result;
	return __atomic_load_n(&src.enabled, __ATOMIC_ACQUIRE) ? 0 : 1;
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
	uint32_t sequence;
	int result;

	generation = __atomic_load_n(&tx.generation, __ATOMIC_ACQUIRE);
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

	/*
	 * cur.write_sequence has exactly one writer -- this thread -- so a plain
	 * increment is enough; the release store on the context state below is
	 * what publishes the new sequence to claim_oldest_ready().  It also
	 * serves as the priming count, since the Nth packet ever staged is by
	 * definition the one that fills the Nth context; a separate counter for
	 * that would only be a second thing to keep in step.
	 */
	sequence = ++cur.write_sequence;
	context->sequence = sequence;
	__atomic_store_n(&context->state, CONTEXT_READY, __ATOMIC_RELEASE);

	/* The transport stays idle until every context holds a packet. */
	if (sequence == CONTEXT_COUNT)
		__atomic_store_n(&tx.primed, 1, __ATOMIC_RELEASE);

	if (__atomic_load_n(&tx.primed, __ATOMIC_ACQUIRE))
		return pump_ready();
	return FEED_QUEUED;
}

static int usb_feeder_thread(SceSize args, void *argp)
{
	int expected = FEEDER_STARTING;
	int pcm_started = 0;
	int result;

	(void)args;
	(void)argp;
	if (!__atomic_compare_exchange_n(&cb.feeder_state, &expected,
		FEEDER_RUNNING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;
	if (!uac_stream_is_active())
		return 0;
	/*
	 * The route is not this thread's to take.  session.c acquires it after
	 * this feeder is already running, so the endpoint is fed silence for the
	 * whole of AVConfig's ~400 ms convergence rather than seeing a gap; PCM
	 * simply starts arriving partway through, and pcm_next() switches to it.
	 */
	pcm_begin();
	pcm_started = 1;

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

	if (pcm_started)
		pcm_end();
#ifdef UAC_PSTV_ENABLE_LOGGING
	{
		/*
		 * Still the sentinel means the cursor never read a block -- a session
		 * that ended inside priming, which a replug storm produces plenty of.
		 * Printing 4294967295 there reads as a fault rather than as no data.
		 */
		uint32_t margin = __atomic_load_n(&margin_min, __ATOMIC_RELAXED);

		if (margin == 0xffffffffu)
			uac_log(LOG_PREFIX
				"session: %u packets, %u starved, %u pcm waits, "
				"%u resyncs, no margin sampled\n",
				__atomic_load_n(&submit_count, __ATOMIC_RELAXED),
				__atomic_load_n(&starve_count, __ATOMIC_RELAXED),
				__atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED),
				__atomic_load_n(&resync_count, __ATOMIC_RELAXED));
		else
			uac_log(LOG_PREFIX
				"session: %u packets, %u starved, %u pcm waits, "
				"%u resyncs, min margin %u blocks\n",
				__atomic_load_n(&submit_count, __ATOMIC_RELAXED),
				__atomic_load_n(&starve_count, __ATOMIC_RELAXED),
				__atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED),
				__atomic_load_n(&resync_count, __ATOMIC_RELAXED),
				margin);
	}
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
	uac_log(LOG_PREFIX "stalls: %u over %u ms, max completion gap %u us\n",
		stall_count, STALL_THRESHOLD_US / 1000u, completion_gap_max);
	{
		uint32_t shown = stall_count < STALL_LOG_MAX ?
			stall_count : STALL_LOG_MAX;
		uint32_t i;

		/*
		 * "starves in gap" is the whole point of this line: non-zero on
		 * every stall means the pipe drained first, zero means it did not.
		 */
		for (i = 0; i < shown; ++i)
			uac_log(LOG_PREFIX
				"  stall %u at packet %u, gap %u us, "
				"%u starves in gap\n",
				i + 1u, stall_at[i], stall_gap[i],
				stall_starves[i]);
	}
#endif
	return 0;
}

static int reap_feeder(SceUInt timeout_us)
{
	SceUID thread;
	int expected;
	int state;
	int status;
	int result;

	for (;;) {
		SceUInt delay = timeout_us < STOP_POLL_US ? timeout_us : STOP_POLL_US;

		state = __atomic_load_n(&cb.feeder_state, __ATOMIC_ACQUIRE);
		thread = __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE);
		if (state == FEEDER_NONE)
			return thread < 0 ? 0 : -1;
		if (state == FEEDER_STARTING || state == FEEDER_REAPING) {
			if (delay == 0)
				return -1;
			ksceKernelDelayThread(delay);
			timeout_us -= delay;
			continue;
		}
		expected = state;
		if (__atomic_compare_exchange_n(&cb.feeder_state, &expected,
			FEEDER_REAPING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			break;
	}
	thread = __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE);
	if (thread < 0) {
		__atomic_store_n(&cb.feeder_state, state, __ATOMIC_RELEASE);
		return -1;
	}
	if (state == FEEDER_RUNNING) {
		result = ksceKernelWaitThreadEnd(thread, &status, &timeout_us);
		if (result < 0) {
			__atomic_store_n(&cb.feeder_state, FEEDER_RUNNING,
				__ATOMIC_RELEASE);
			return result;
		}
	}
	result = ksceKernelDeleteThread(thread);
	if (result >= 0) {
		__atomic_store_n(&feeder_thread, -1, __ATOMIC_RELAXED);
		__atomic_store_n(&cb.feeder_state, FEEDER_NONE, __ATOMIC_RELEASE);
		finish_retire();
	} else
		__atomic_store_n(&cb.feeder_state, FEEDER_DORMANT, __ATOMIC_RELEASE);
	return result;
}

int uac_stream_start(int pipe_id)
{
	int expected = STREAM_IDLE;
	int feeder_expected = FEEDER_NONE;
	uint32_t generation;
	uint32_t context_index;
	int result;

	if (pipe_id < 0)
		return -1;
	if (__atomic_load_n(&cb.feeder_state, __ATOMIC_ACQUIRE) != FEEDER_NONE ||
	    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
		SceUInt no_wait = 0;

		if (reap_feeder(no_wait) < 0)
			return -1;
	}
	if (!__atomic_compare_exchange_n(&cb.feeder_state, &feeder_expected,
		FEEDER_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;
	if (!__atomic_compare_exchange_n(&tx.state, &expected,
		STREAM_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		__atomic_store_n(&cb.feeder_state, FEEDER_NONE, __ATOMIC_RELEASE);
		return -1;
	}

	if (free_event < 0 || pcm_event < 0) {
		__atomic_store_n(&cb.feeder_state, FEEDER_NONE, __ATOMIC_RELEASE);
		stop_stream(-1, USBD_CC_NOERR);
		return -1;
	}

	generation = next_generation();
	__atomic_store_n(&tx.pipe, pipe_id, __ATOMIC_RELEASE);
	/*
	 * STREAM_IDLE proves finish_retire() removed CALLBACK_CLOSING.  Do not
	 * reset the reference count here: a callback from the retired generation
	 * may have entered after that gate reopened and must be allowed to drop its
	 * own reference without underflowing the new session's guard.
	 */
	__atomic_store_n(&tx.in_flight, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&tx.pump_requests, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&cur.write_sequence, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&tx.primed, 0, __ATOMIC_RELEASE);
#ifdef UAC_PSTV_ENABLE_LOGGING
	/* completion_seen especially, or session one's last completion becomes
	 * session two's baseline and the unplug time reads as a stall. */
	completion_seen = 0;
	completion_at_us = 0u;
	completion_gap_max = 0u;
	stall_count = 0u;
	starve_at_completion = 0u;
#endif
	memset(contexts, 0, sizeof(contexts));

	for (context_index = 0; context_index < CONTEXT_COUNT; ++context_index) {
		StreamContext *context = &contexts[context_index];

		context->callback_token = generation | context_index;
		context->transfer.buffer_base = context->buffer;
		context->transfer.num_packets = 1u;
		__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	}

	reset_event(free_event);
	publish_free_context();

	result = ksceKernelCreateThread("uac_usb_feeder", usb_feeder_thread,
		FEEDER_PRIORITY, FEEDER_STACK, 0, FEEDER_CPU_MASK, NULL);
	if (result < 0) {
		__atomic_store_n(&cb.feeder_state, FEEDER_NONE, __ATOMIC_RELEASE);
		stop_stream(result, USBD_CC_NOERR);
		return result;
	}
	__atomic_store_n(&feeder_thread, result, __ATOMIC_RELEASE);
	expected = STREAM_STARTING;
	if (!__atomic_compare_exchange_n(&tx.state, &expected, STREAM_RUNNING,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		__atomic_store_n(&cb.feeder_state, FEEDER_DORMANT, __ATOMIC_RELEASE);
		return -1;
	}
	result = ksceKernelStartThread(
		__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE), 0, NULL);
	if (result < 0) {
		/* Retain the dormant UID until teardown can prove deletion. */
		__atomic_store_n(&cb.feeder_state, FEEDER_DORMANT, __ATOMIC_RELEASE);
		stop_stream(result, USBD_CC_NOERR);
		return result;
	}

	uac_log(LOG_PREFIX
		"ready: %u fixed 1-ms contexts; %u in flight + %u READY; "
		"completion-driven rotation; 1x%u bytes/request; "
		"virtual native DataSend\n",
		CONTEXT_COUNT, MAX_IN_FLIGHT,
		CONTEXT_COUNT - MAX_IN_FLIGHT, TRANSFER_BYTES);
	return 0;
}

int uac_stream_is_active(void)
{
	return __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_RUNNING;
}

void uac_stream_stop(void)
{
	int state;
	int expected;
	uint32_t wait;
	SceUInt feeder_timeout;
	int feeder_result;

	for (;;) {
		state = __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE);
		if (state == STREAM_IDLE) {
			if (__atomic_load_n(&cb.feeder_state, __ATOMIC_ACQUIRE) ==
			    FEEDER_NONE &&
			    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) < 0)
				return;
		}
		if (state == STREAM_STOPPING)
			break;
		expected = state;
		if (__atomic_compare_exchange_n(&tx.state, &expected,
			STREAM_STOPPING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			break;
	}

	__atomic_fetch_or(&cb.guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	publish_free_context();
	signal_pcm();

	if (__atomic_load_n(&cb.feeder_state, __ATOMIC_ACQUIRE) != FEEDER_NONE ||
	    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
		feeder_timeout = FEEDER_STOP_TIMEOUT_US;
		feeder_result = reap_feeder(feeder_timeout);
		if (feeder_result < 0)
			uac_log(LOG_PREFIX "USB feeder stop failed: 0x%08x\n",
				feeder_result);
	}

	for (wait = 0; wait < STOP_POLLS; ++wait) {
		if (!(__atomic_load_n(&cb.guard, __ATOMIC_ACQUIRE) & CALLBACK_REFS)) {
			finish_retire();
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
	    __atomic_load_n(&tx.pipe, __ATOMIC_ACQUIRE) != pipe_id)
		return;

	__atomic_exchange_n(&tx.state, STREAM_STOPPING, __ATOMIC_ACQ_REL);
	__atomic_fetch_or(&cb.guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	/* Invalidate submitted tokens before finish_retire() can reopen the gate. */
	(void)next_generation();
	__atomic_store_n(&tx.pipe, -1, __ATOMIC_RELEASE);
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

	uac_stream_stop();
	if (__atomic_load_n(&cb.feeder_state, __ATOMIC_ACQUIRE) != FEEDER_NONE ||
	    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0) {
		SceUInt timeout = FEEDER_STOP_TIMEOUT_US;
		publish_free_context();
		signal_pcm();
		result = reap_feeder(timeout);
		if (result < 0)
			return result;
	}
	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_IDLE)
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
