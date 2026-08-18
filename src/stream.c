/*
 * PCM handoff and USB transport.
 *
 * Two halves that meet in the middle:
 *
 *   - the source side is Sony's own audio engine, writing captured PCM into
 *     slices of AVConfig's RAM-output region;
 *   - the transport side runs one feeder thread that cuts those slices into
 *     48-frame (1 ms) packets and keeps three isochronous requests in flight,
 *     with one context staged behind them.
 *
 * Staging is latest-wins, and deliberately not a FIFO.  Producer and consumer
 * run off unrelated clocks -- Sony's audio hardware and the USB host's frame
 * timer -- so they are never rate-matched, and USB completions have been
 * measured stalling for 250 ms at a stretch.  Latest-wins throws the backlog
 * away and resumes at current audio for the price of one discontinuity; a FIFO
 * would add that 250 ms to output latency permanently, again on each stall.
 *
 * session.c owns this file's lifecycle: uac_stream_start(), uac_stream_stop()
 * and uac_stream_pipe_closed() are called from the session thread and nowhere
 * else, so the feeder has exactly one creator and one reaper.
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

#define STATIC_ASSERT(condition, tag) typedef char tag[(condition) ? 1 : -1]

/* Sony-shaped shallow transport for the tested HS bInterval=4 adapter. */
#define PACKET_FRAMES 48u
#define PACKET_BYTES (PACKET_FRAMES * 4u)
#define SONY_ISO_PACKET_SLOTS 8u
#define PCM_SLICE_MASK (UAC_STREAM_SLICE_COUNT - 1u)
/*
 * Two slices are unreadable at any instant -- ram_submit()'s mailbox holds one
 * queued while Sony fills another -- and the third is latest itself, which
 * trail counts from.  Hence COUNT - 3, not COUNT - 2.
 */
#define PCM_MAX_TRAIL (UAC_STREAM_SLICE_COUNT - 3u)
/*
 * Transport depth: three requests owned by USBD and one READY context.  The
 * fourth lets the feeder prepare the next millisecond without touching
 * DMA-owned storage, while three queued frames keep SceUsbd's periodic cursor
 * ahead.
 */
#define CONTEXT_COUNT 4u
#define MAX_IN_FLIGHT 3u

/*
 * USBD may deliver a completion after its pipe has been closed and the static
 * contexts have been reused by a later session, so the callback identity must
 * be immutable: the opaque argument carries the session generation in its upper
 * bits and the context index in its lower two.  Never pass a context pointer.
 */
#define CALLBACK_CONTEXT_MASK (CONTEXT_COUNT - 1u)
#define CALLBACK_GENERATION_MASK (~CALLBACK_CONTEXT_MASK)
#define CALLBACK_CLOSING 0x80000000u
#define CALLBACK_REFS 0x7fffffffu

#define FREE_EVENT_BIT 0x00000001u
#define PCM_EVENT_BIT 0x00000002u
#define STOP_POLL_US 1000u
#define STOP_POLLS 50u
/*
 * The feeder exits through audio_tap_end(), whose own worst case is
 * WORKER_TIMEOUT_US + STOP_SETTLE_US + ROUTE_TIMEOUT_US.  This must exceed that
 * sum or teardown returns while the feeder still owns the route and the hooks.
 */
#define FEEDER_STOP_TIMEOUT_US 4000000u

/*
 * Feeder scheduling.  0x40 is the bottom of the kernel band and a poor place
 * for a thread with a 1 ms deadline; 0x20 lifts it clear while staying below
 * the capture worker's 0x12, which must not be starved because it is the
 * producer.  Per wakeup the feeder claims a context, copies 192 bytes and
 * blocks again, so it cannot monopolise a core.
 *
 * The affinity is a cache decision rather than a scheduling one: pinned to the
 * capture worker's core (CAPTURE_CPU_MASK in audio_tap.c), a slice Sony has
 * just filled is still in L1 when the feeder reads it out.
 */
#define FEEDER_PRIORITY 0x20
#define FEEDER_CPU_MASK 0x80000u
#define FEEDER_STACK 0x3000u

/*
 * UAC_ERG is the Cortex-A9 exclusive-reservation granule, which on this core is
 * also the L1 line length.  Both are 32 bytes; do not carry a 64 over from a
 * later Cortex.
 *
 * A reservation covers the granule, not the word, so a plain store by any core
 * into a granule clears an ldrex another core holds on it.  The two things that
 * must never share one are a word written every millisecond by one thread and a
 * word ldrex/strex'd every millisecond by another.
 */
#define UAC_ERG 32u

STATIC_ASSERT(UAC_ERG != 0u && (UAC_ERG & (UAC_ERG - 1u)) == 0u,
	erg_must_be_a_power_of_two);

#if PACKET_BYTES != 192u
#error Tested transport must remain one 192-byte packet per request
#endif

#if CONTEXT_COUNT != 4u
#error Callback token encoding requires four contexts
#endif

#if UAC_STREAM_CAPTURE_FRAMES % PACKET_FRAMES != 0u
#error Native blocks must contain a whole number of USB packets
#endif

/*
 * Sony's DMA rounds its write up to a 256-byte multiple, so a slice has to hold
 * the rounded size rather than the block size.  A 576-byte block packed into a
 * 576-byte slice rounds to 768 and spills 192 bytes into the next slice, which
 * is audible as distortion rather than a click because it corrupts every block.
 * Leave headroom; do not pack the stride down to the payload.
 */
#define DMA_WRITE_ROUND 256u
STATIC_ASSERT(UAC_STREAM_SLICE_BYTES >=
	((UAC_STREAM_CAPTURE_BYTES + DMA_WRITE_ROUND - 1u) / DMA_WRITE_ROUND) *
		DMA_WRITE_ROUND,
	slice_must_hold_the_rounded_up_dma_write);
STATIC_ASSERT(UAC_STREAM_SLICE_COUNT > 3u &&
	(UAC_STREAM_SLICE_COUNT & PCM_SLICE_MASK) == 0u,
	slice_count_must_be_a_power_of_two_above_three);
/* Keeps every slice line-aligned, not just the first. */
STATIC_ASSERT(UAC_STREAM_SLICE_BYTES % UAC_ERG == 0u,
	slice_stride_must_tile_lines);

enum {
	STREAM_IDLE = 0,
	STREAM_STARTING,
	STREAM_RUNNING,
	STREAM_STOPPING,
};

/*
 * IN_FLIGHT exists because READY cannot mean two things at once.  With several
 * requests outstanding, READY must stay distinct from a buffer USBD still owns,
 * or DMA storage can be submitted twice.
 */
enum {
	CONTEXT_FREE = 0,
	CONTEXT_WRITING,
	CONTEXT_READY,
	CONTEXT_IN_FLIGHT,
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

STATIC_ASSERT(sizeof(SonyIsoPacket) == 4u, sony_iso_packet_size_must_be_4);
#if UINTPTR_MAX == 0xffffffffu
STATIC_ASSERT(sizeof(SonyIsoTransfer) == 0x28u, sony_iso_size_must_be_0x28);
STATIC_ASSERT(offsetof(SonyIsoTransfer, num_packets) == 0x04u,
	sony_iso_count_offset_must_be_4);
STATIC_ASSERT(offsetof(SonyIsoTransfer, packets) == 0x08u,
	sony_iso_packets_offset_must_be_8);
#endif

/*
 * CONTEXT_ALIGN is a DMA constraint, not a cache one.  buffer[] is the only
 * storage the host controller reads directly, and EHCI addresses transfer
 * buffers as a page plus an offset, so one that straddles a 4 KiB boundary
 * needs the descriptor's second page pointer programmed too.  Aligning each
 * context to a divisor of the page size, buffer first and no larger than that
 * alignment, keeps every buffer inside one page by construction.
 */
#define CONTEXT_ALIGN 256u

typedef struct {
	uint8_t buffer[PACKET_BYTES] __attribute__((aligned(64)));
	SonyIsoTransfer transfer __attribute__((aligned(64)));
	uint32_t callback_token;
	uint32_t sequence;
	int state;
} StreamContext;

STATIC_ASSERT(sizeof(StreamContext) == CONTEXT_ALIGN,
	context_must_tile_its_alignment);
STATIC_ASSERT(PACKET_BYTES <= CONTEXT_ALIGN, context_buffer_must_fit_one_block);
STATIC_ASSERT(4096u % CONTEXT_ALIGN == 0u, context_blocks_must_tile_a_page);

static StreamContext contexts[CONTEXT_COUNT]
	__attribute__((aligned(CONTEXT_ALIGN)));

/*
 * Shared state, grouped by who touches it and padded to own its granule.
 *
 * Aligning the variables alone is not enough: that fixes where an object
 * starts, not how much it occupies, so GCC stays free to pack the next scalar
 * into the tail.  Aligning the type is what reserves the granule.
 *
 * The attribute must sit before the typedef name, as written below.  After it
 * -- "} TxPump __attribute__((aligned(32)))" -- applies to the declarator,
 * giving alignof 32 while leaving sizeof 24, which is the exact hole this
 * closes.  New shared state goes inside one of these, not at file scope.
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
 * Callback-thread private in steady state, and the reason it is not folded into
 * TxPump: sharing a granule would let the feeder's pump_requests RMW clear the
 * callback's guard reservation every millisecond.
 */
typedef struct {
	uint32_t guard;
} __attribute__((aligned(UAC_ERG))) CallbackLifetime;

/*
 * Capture slices.  AVConfig's two RAM-output pages are contiguous -- 0x400 and
 * 0xC00, 0x800 each -- and ram_submit() takes its buffer per call, validating
 * only the frame count.  The two-page ping-pong is Sony's convention, not their
 * API's, so we rotate our own slices through that one region and their engine
 * writes the staging buffer directly.
 *
 * The payload is Sony's; the publication protocol is ours.  guard[] is odd
 * while a slice is in flight, even once the following submit proves it
 * complete.  sequence and previous are the slices being filled and queued
 * behind it; zero means none, which is why sequence numbers skip it.  base is
 * written once per session, before any worker exists.
 */
typedef struct {
	void *base;
	uint32_t latest;
	uint32_t sequence;
	uint32_t previous;
	uint32_t guard[UAC_STREAM_SLICE_COUNT];
} __attribute__((aligned(UAC_ERG))) PcmSource;

/*
 * Feeder-private packetizer cursor.  The callback never reads any of it, which
 * is exactly why it must not sit next to what the callback does ldrex on.
 *
 * write_sequence is the ticket stamped on each staged packet.
 * claim_oldest_ready() submits by it rather than by slot, so PCM order survives
 * contexts completing and being reused out of slot order -- which is what a
 * fixed slot rotation gets wrong.  It also serves as the priming count.
 */
typedef struct {
	uint32_t sequence;
	uint32_t offset;
	uint32_t write_sequence;
	int valid;
} __attribute__((aligned(UAC_ERG))) PcmCursor;

/*
 * A group must occupy whole granules and start on one, so nothing outside it
 * can ever share.  sizeof catches the aligned attribute written in the place
 * that raises alignof without padding; _Alignof catches a group starting
 * mid-granule, which straddles two of them however big it is.
 */
#define MUST_OWN_WHOLE_GRANULES(type, tag) \
	STATIC_ASSERT(sizeof(type) % UAC_ERG == 0u, tag##_must_fill_its_granules); \
	STATIC_ASSERT(_Alignof(type) == UAC_ERG, tag##_must_start_on_a_granule)

MUST_OWN_WHOLE_GRANULES(TxPump, tx_pump);
MUST_OWN_WHOLE_GRANULES(CallbackLifetime, callback_lifetime);
MUST_OWN_WHOLE_GRANULES(PcmSource, pcm_source);
MUST_OWN_WHOLE_GRANULES(PcmCursor, pcm_cursor);

static TxPump tx = { .pipe = -1 };
static CallbackLifetime cb;
/* Producer state, written by audio_tap's capture worker. */
static PcmSource src;
/* Consumer state belongs exclusively to the feeder thread. */
static PcmCursor cur;

/*
 * Two event flags, one bit each.  Never merge them into one flag.
 *
 * Both waiters use SCE_EVENT_WAITCLEAR, which clears the entire flag rather
 * than only the matched pattern, so a shared flag lets either wake destroy the
 * other subsystem's pending wakeup.  That is unrecoverable during priming:
 * there is no transfer in flight yet to re-post FREE_EVENT_BIT, so the feeder
 * waits forever and the stream comes up silent with every setup step in the log
 * reporting success.  Use SCE_EVENT_WAITCLEAR_PAT if you ever do need to share.
 *
 * At file scope rather than grouped: none of the three has a per-packet writer,
 * so nothing here can invalidate a granule another core is holding.
 */
static SceUID free_event = -1;
static SceUID pcm_event = -1;
static SceUID feeder_thread = -1;

/*
 * Session instrumentation, logging builds only.
 *
 *   starve   the transport won the right to submit and found the next context
 *            unfilled, so the isochronous stream gets a hole.  Audible.
 *   pcm wait the feeder had a context but no PCM to put in it.  Absorbed by the
 *            staged contexts; not audible by itself.
 *   resync   the producer lapped the slice being read and the cursor jumped to
 *            current audio.  Audible, and the one that matters most.  Its gap
 *            says what caused it: two or three slices is ordinary jitter, fifty
 *            is one of the 250 ms whole-pipe USB stalls.
 *
 * These swing by orders of magnitude between runs of the same binary, so read
 * patterns across several sessions and never conclude anything from one number.
 */
#ifdef UAC_PSTV_ENABLE_LOGGING
#define RESYNC_LOG_MAX 8u
static uint32_t starve_count;
static uint32_t submit_count;
static uint32_t pcm_wait_count;
static uint32_t resync_count;
static uint32_t resync_at[RESYNC_LOG_MAX];
static uint32_t resync_gap[RESYNC_LOG_MAX];
#define COUNT_STARVE() __atomic_add_fetch(&starve_count, 1u, __ATOMIC_RELAXED)
#define COUNT_SUBMIT() __atomic_add_fetch(&submit_count, 1u, __ATOMIC_RELAXED)
#define COUNT_PCM_WAIT() __atomic_add_fetch(&pcm_wait_count, 1u, __ATOMIC_RELAXED)
/*
 * Stored rather than logged inline, because uac_log() writes to the filesystem
 * and doing that from pcm_next() would stall the feeder and manufacture the
 * very problem being measured.
 *
 * A ring, so what survives is the last few rather than the first few.  Once
 * resyncs start repeating, the opening ones are all onset and say nothing about
 * the state it settles into.
 */
#define COUNT_RESYNC(gap) do { \
	uint32_t n = __atomic_fetch_add(&resync_count, 1u, __ATOMIC_RELAXED) % \
		RESYNC_LOG_MAX; \
	resync_at[n] = __atomic_load_n(&submit_count, __ATOMIC_RELAXED); \
	resync_gap[n] = (gap); \
} while (0)
#else
#define COUNT_STARVE() ((void)0)
#define COUNT_SUBMIT() ((void)0)
#define COUNT_PCM_WAIT() ((void)0)
#define COUNT_RESYNC(gap) ((void)0)
#endif

static uint32_t next_generation(void)
{
	/* Keep the context-index bits clear; skip zero when the token wraps. */
	uint32_t value = __atomic_add_fetch(&tx.generation, CONTEXT_COUNT,
		__ATOMIC_ACQ_REL);

	return value ? value : __atomic_add_fetch(&tx.generation, CONTEXT_COUNT,
		__ATOMIC_ACQ_REL);
}

static int stream_running(void)
{
	return __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_RUNNING;
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

static uint8_t *slice(uint32_t sequence)
{
	return (uint8_t *)src.base +
		(sequence & PCM_SLICE_MASK) * UAC_STREAM_SLICE_BYTES;
}

void uac_stream_capture_region(void *base)
{
	__atomic_store_n(&src.base, base, __ATOMIC_RELEASE);
}

void *uac_stream_capture_claim(void)
{
	uint32_t sequence = next_pcm_sequence(src.sequence);

	src.previous = src.sequence;
	src.sequence = sequence;
	/*
	 * Odd marks the slice in flight, and the fence keeps that visible before
	 * Sony's engine starts writing behind it.  A relaxed store plus a release
	 * fence is one dmb; a seq_cst store would be dmb-str-dmb and buy nothing,
	 * there being a single producer.  This is the write_seqlock() half.
	 */
	__atomic_store_n(&src.guard[sequence & PCM_SLICE_MASK],
		(sequence << 1) | 1u, __ATOMIC_RELAXED);
	__atomic_thread_fence(__ATOMIC_RELEASE);
	return slice(sequence);
}

void uac_stream_capture_ready(void)
{
	uint32_t sequence = src.previous;

	/* The first submit of a session has nothing queued behind it. */
	if (sequence == 0u)
		return;
	__atomic_store_n(&src.guard[sequence & PCM_SLICE_MASK], sequence << 1,
		__ATOMIC_RELEASE);
	__atomic_store_n(&src.latest, sequence, __ATOMIC_RELEASE);
	signal_pcm();
}

static void finish_retire(void)
{
	uint32_t expected = CALLBACK_CLOSING;

	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_STOPPING &&
	    __atomic_load_n(&tx.pipe, __ATOMIC_ACQUIRE) < 0 &&
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

	if (!stream_running() ||
	    __atomic_load_n(&tx.generation, __ATOMIC_ACQUIRE) != generation)
		return 1;
	pipe = __atomic_load_n(&tx.pipe, __ATOMIC_ACQUIRE);
	if (pipe < 0)
		return 1;

	/* After the aborts, not before: a teardown should not pay for a flush. */
	ksceKernelDcacheCleanRange(context->buffer, PACKET_BYTES);
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
		 * comparison once cur.write_sequence wraps, which at one packet
		 * per millisecond is about every 50 days of continuous playback.
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

/*
 * Drain every READY context the schedule has room for.  Failures stop the
 * stream here rather than reporting upwards, so a caller only ever has to
 * re-test stream_running() to learn the transport is gone.
 */
static void pump_ready(void)
{
	/*
	 * The first requester owns the drain.  Later requesters only add a ticket;
	 * the owner consumes every ticket before leaving.  Unlike a boolean gate,
	 * the same atomic operation both publishes work and hands ownership over,
	 * so a READY transition cannot land between an owner's final scan and its
	 * unlock and be lost there.
	 */
	if (__atomic_fetch_add(&tx.pump_requests, 1u, __ATOMIC_ACQ_REL) != 0u)
		return;

	for (;;) {
		while (stream_running() &&
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
			break;
		}

		/*
		 * A hole is the schedule running dry, not merely being full.  A
		 * submit that just failed has already left STREAM_RUNNING, so it
		 * cannot be miscounted as one.
		 */
		if (stream_running() &&
		    __atomic_load_n(&tx.primed, __ATOMIC_ACQUIRE) &&
		    __atomic_load_n(&tx.in_flight, __ATOMIC_ACQUIRE) == 0u)
			COUNT_STARVE();

		/* The last consumed ticket releases ownership. */
		if (__atomic_fetch_sub(&tx.pump_requests, 1u, __ATOMIC_ACQ_REL) == 1u)
			return;
	}
}

static void transfer_done(int32_t result, ksceUsbdIsochTransfer *sdk_transfer,
	void *arg)
{
	uint32_t token = (uint32_t)(uintptr_t)arg;
	StreamContext *context;
	uint16_t status;

	if (callback_enter(token & CALLBACK_GENERATION_MASK) <= 0)
		return;
	context = &contexts[token & CALLBACK_CONTEXT_MASK];
	if (!stream_running())
		goto out;

	status = ((SonyIsoTransfer *)(void *)sdk_transfer)->packets[0].status;
	if (result < 0 || status != USBD_CC_NOERR) {
		stop_stream(result, status);
		goto out;
	}

	__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	/*
	 * RELEASE, not ACQ_REL: one dmb per completion.  ARM maps an
	 * acquire-release RMW to dmb-ldrex/strex-dmb, so ACQ_REL here plus
	 * pump_ready()'s ACQ_REL ticket increment emitted two barriers back to
	 * back.  The dropped acquire half would have ordered the loads that
	 * follow, and those sit behind the ticket increment's own barrier -- so
	 * this is correct only while the next statement is an acquire-release
	 * atomic.  Put a plain load between them and the ordering is gone.
	 */
	__atomic_sub_fetch(&tx.in_flight, 1u, __ATOMIC_RELEASE);
	/*
	 * Submit first, wake second, and not the other way round.  Both orderings
	 * end with the same work done, but only one keeps the schedule fed.
	 * publish_free_context() is a kernel call that can reschedule on the spot
	 * -- the feeder sits at FEEDER_PRIORITY on this core, so waking it can
	 * preempt this callback before the resubmit happens -- and the resubmit is
	 * the deadline-bearing half: ksceUsbdIsochronousTransfer() has been
	 * measured taking 260-505 us of a 1 ms frame.  Nothing the feeder can do
	 * is urgent by comparison; the context it is being woken for went FREE
	 * above and stays FREE.
	 */
	pump_ready();
	publish_free_context();

out:
	callback_leave();
}

/*
 * Claim a context USBD does not own, blocking until one exists.  Scanning
 * before waiting is what makes the wakeup unloseable: a completion that posts
 * between the scan and the wait leaves FREE_EVENT_BIT set, and the wait returns
 * on it immediately.
 */
static int wait_for_free_context(StreamContext **out)
{
	uint32_t matched;
	uint32_t index;
	int result;

	for (;;) {
		if (!stream_running())
			return 0;
		for (index = 0; index < CONTEXT_COUNT; ++index) {
			/* The feeder is the sole FREE claimant. */
			if (__atomic_load_n(&contexts[index].state,
				__ATOMIC_ACQUIRE) != CONTEXT_FREE)
				continue;
			__atomic_store_n(&contexts[index].state,
				CONTEXT_WRITING, __ATOMIC_RELAXED);
			*out = &contexts[index];
			return 1;
		}
		result = ksceKernelWaitEventFlag(free_event, FREE_EVENT_BIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, NULL);
		if (result < 0) {
			stop_stream(result, USBD_CC_NOERR);
			return 0;
		}
	}
}

static int pcm_copy(uint32_t sequence, uint32_t offset, uint8_t *packet)
{
	uint32_t *guard = &src.guard[sequence & PCM_SLICE_MASK];
	uint32_t expected = sequence << 1;
	uint32_t attempt;

	for (attempt = 0; attempt < 3u; ++attempt) {
		if (__atomic_load_n(guard, __ATOMIC_ACQUIRE) != expected)
			continue;
		memcpy(packet, slice(sequence) + offset * 4u, PACKET_BYTES);
		/*
		 * Fence first, then a relaxed read -- not an acquire load.
		 * Acquire orders what comes after it; what has to be ordered
		 * here is the memcpy above, which must be complete before the
		 * guard is sampled again.  An acquire load puts its barrier on
		 * the far side and leaves the copy free to be reordered past
		 * the check, which is how a torn read passes validation.  This
		 * is the read_seqretry() half of the pattern.
		 */
		__atomic_thread_fence(__ATOMIC_ACQUIRE);
		if (__atomic_load_n(guard, __ATOMIC_RELAXED) == expected)
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

/* Return zero for a packet, one when no packet is ready. */
static int pcm_next(uint8_t packet[PACKET_BYTES])
{
	uint32_t latest = __atomic_load_n(&src.latest, __ATOMIC_ACQUIRE);
	uint32_t gap;

	/*
	 * Priming.  uac1 has already selected the alternate setting, so the device
	 * is live and expecting a packet every millisecond well before capture has
	 * produced anything -- the tap still has to acquire the route, and two
	 * slices have to exist before the consumer can trail the producer.  Send
	 * silence across that window rather than nothing: a gap on an active
	 * isochronous endpoint reads as an invalid stream to the device, while
	 * continuous silence lets it lock cleanly and audio simply fades in.
	 * This is also what keeps src.base from being read before audio_tap has
	 * published it, since latest only leaves zero once a slice is complete.
	 */
	if (!cur.valid) {
		if (latest < 2u) {
			memset(packet, 0, PACKET_BYTES);
			return 0;
		}
		pcm_resync(latest);
	}

	gap = pcm_ahead_by(latest, cur.sequence);
	if (gap > PCM_MAX_TRAIL) {
		/* Lapped: Sony is overwriting the slice we were reading. */
		COUNT_RESYNC(gap);
		pcm_resync(latest);
	}

	if (!pcm_copy(cur.sequence, cur.offset, packet)) {
		latest = __atomic_load_n(&src.latest, __ATOMIC_ACQUIRE);
		gap = pcm_ahead_by(latest, cur.sequence);
		if (gap > PCM_MAX_TRAIL) {
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

static void pcm_wait(void)
{
	uint32_t matched;
	int result;

	result = ksceKernelWaitEventFlag(pcm_event, PCM_EVENT_BIT,
		SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, NULL);
	if (result < 0)
		stop_stream(result, USBD_CC_NOERR);
}

/*
 * Stage one packet.  Returns non-zero only when there was no PCM to stage;
 * every other outcome, success or failure, leaves the caller to re-test
 * stream_running().
 *
 * Order matters: take USB-owned storage first, then sample the source.  The
 * reverse reads as equivalent and is not -- that wait is unbounded if a
 * completion is delayed, and a packet copied before it goes stale while we sit
 * in it.
 */
static int queue_next_source_packet(void)
{
	StreamContext *context;
	uint32_t sequence;

	if (!wait_for_free_context(&context))
		return 0;

	if (pcm_next(context->buffer) != 0) {
		__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
		publish_free_context();
		return 1;
	}

	/* One writer, so a plain increment is enough; the release store below is
	 * what publishes the new sequence to claim_oldest_ready(). */
	sequence = ++cur.write_sequence;
	context->sequence = sequence;
	__atomic_store_n(&context->state, CONTEXT_READY, __ATOMIC_RELEASE);

	/* The transport stays idle until every context holds a packet. */
	if (sequence == CONTEXT_COUNT)
		__atomic_store_n(&tx.primed, 1, __ATOMIC_RELEASE);
	if (__atomic_load_n(&tx.primed, __ATOMIC_ACQUIRE))
		pump_ready();
	return 0;
}

#ifdef UAC_PSTV_ENABLE_LOGGING
static void report_session(void)
{
	uint32_t total = __atomic_load_n(&resync_count, __ATOMIC_RELAXED);
	uint32_t shown = total < RESYNC_LOG_MAX ? total : RESYNC_LOG_MAX;
	uint32_t first = total - shown;
	uint32_t i;

	uac_log(LOG_PREFIX
		"session: %u packets, %u starved, %u pcm waits, %u resyncs\n",
		__atomic_load_n(&submit_count, __ATOMIC_RELAXED),
		__atomic_load_n(&starve_count, __ATOMIC_RELAXED),
		__atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED), total);
	/* Numbered by their real ordinal, so a run reads 331..338 rather than 1..8
	 * and the packet numbers show how far apart they are firing. */
	for (i = 0; i < shown; ++i) {
		uint32_t k = (first + i) % RESYNC_LOG_MAX;

		uac_log(LOG_PREFIX
			"  resync %u at packet %u, producer was %u slices ahead\n",
			first + i + 1u, resync_at[k], resync_gap[k]);
	}

	__atomic_store_n(&submit_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&starve_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&pcm_wait_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&resync_count, 0u, __ATOMIC_RELAXED);
}
#define REPORT_SESSION() report_session()
#else
#define REPORT_SESSION() ((void)0)
#endif

static int usb_feeder_thread(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	/*
	 * The route is not this thread's to take.  session.c acquires it after
	 * this feeder is already running, so the endpoint is fed silence for the
	 * whole of AVConfig's ~400 ms convergence rather than seeing a gap; PCM
	 * simply starts arriving partway through, and pcm_next() switches to it.
	 */
	while (stream_running()) {
		/*
		 * A context with no PCM to put in it.  If this tracks the starve
		 * count, the shortfall is on the producer side and feeder
		 * priority is irrelevant.
		 */
		if (queue_next_source_packet()) {
			COUNT_PCM_WAIT();
			pcm_wait();
		}
	}
	REPORT_SESSION();
	return 0;
}

/*
 * Only the session thread creates or reaps the feeder, and feeder_thread is set
 * only for a thread that started, so there is no state machine here: either a
 * thread is running or the field is -1.
 */
static int reap_feeder(SceUInt timeout_us)
{
	SceUID thread = __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE);
	int status;
	int result;

	if (thread < 0)
		return 0;
	result = ksceKernelWaitThreadEnd(thread, &status, &timeout_us);
	if (result < 0)
		return result;
	result = ksceKernelDeleteThread(thread);
	if (result < 0)
		return result;
	__atomic_store_n(&feeder_thread, -1, __ATOMIC_RELEASE);
	finish_retire();
	return 0;
}

int uac_stream_start(int pipe_id)
{
	int expected = STREAM_IDLE;
	uint32_t generation;
	uint32_t index;
	int thread;
	int result;

	if (pipe_id < 0 || free_event < 0 || pcm_event < 0)
		return -1;
	/*
	 * A feeder outliving its session means the last reap failed.  Refuse
	 * rather than delete a thread that may still be running; the next
	 * teardown retries it with the full timeout.
	 */
	if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0)
		return -1;
	if (!__atomic_compare_exchange_n(&tx.state, &expected,
		STREAM_STARTING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;

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
	memset(contexts, 0, sizeof(contexts));

	/*
	 * The source is reset here rather than on the feeder, so it is ordered
	 * before audio_tap_begin() by the session thread itself.  Guards start
	 * odd, which is what makes an untouched slice unreadable rather than
	 * looking like a complete sequence zero.
	 */
	memset(&src, 0, sizeof(src));
	for (index = 0; index < UAC_STREAM_SLICE_COUNT; ++index)
		src.guard[index] = 1u;
	cur.sequence = 0;
	cur.offset = 0;
	cur.valid = 0;

	for (index = 0; index < CONTEXT_COUNT; ++index) {
		StreamContext *context = &contexts[index];

		context->callback_token = generation | index;
		context->transfer.buffer_base = context->buffer;
		context->transfer.num_packets = 1u;
		__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	}

	reset_event(free_event);
	reset_event(pcm_event);
	publish_free_context();

	thread = ksceKernelCreateThread("uac_usb_feeder", usb_feeder_thread,
		FEEDER_PRIORITY, FEEDER_STACK, 0, FEEDER_CPU_MASK, NULL);
	if (thread < 0) {
		stop_stream(thread, USBD_CC_NOERR);
		return thread;
	}
	/* RUNNING before the thread starts, so its first loop test cannot lose a
	 * race with its own creation. */
	expected = STREAM_STARTING;
	if (!__atomic_compare_exchange_n(&tx.state, &expected, STREAM_RUNNING,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		(void)ksceKernelDeleteThread(thread);
		return -1;
	}
	__atomic_store_n(&feeder_thread, thread, __ATOMIC_RELEASE);
	result = ksceKernelStartThread(thread, 0, NULL);
	if (result < 0) {
		__atomic_store_n(&feeder_thread, -1, __ATOMIC_RELEASE);
		(void)ksceKernelDeleteThread(thread);
		stop_stream(result, USBD_CC_NOERR);
		return result;
	}

	uac_log(LOG_PREFIX "ready: %u contexts, %u in flight, %u bytes/request\n",
		CONTEXT_COUNT, MAX_IN_FLIGHT, PACKET_BYTES);
	return 0;
}

void uac_stream_stop(void)
{
	uint32_t wait;
	int result;

	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_IDLE &&
	    __atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) < 0)
		return;

	__atomic_store_n(&tx.state, STREAM_STOPPING, __ATOMIC_RELEASE);
	__atomic_fetch_or(&cb.guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);
	publish_free_context();
	signal_pcm();

	result = reap_feeder(FEEDER_STOP_TIMEOUT_US);
	if (result < 0)
		uac_log(LOG_PREFIX "USB feeder stop failed: 0x%08x\n", result);

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
	if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0 ||
	    __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_IDLE)
		return -1;
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
