/*
 * PCM handoff and USB transport.
 *
 * Two halves that meet in the middle:
 *
 *   - the source side is Sony's own audio engine, writing captured PCM into
 *     slices of AVConfig's RAM-output region and handing each finished block
 *     to a queue as four 1 ms packets;
 *   - the transport side runs one feeder thread that drains that queue a packet
 *     per frame, pointing the host controller straight at the slice.
 *
 * Nothing copies the audio and nothing samples a position: the producer sets
 * both the content and the pace, and the queue depth is the only state between
 * them.  Its two bounds are the only places a discontinuity can enter, and both
 * are counted.
 *
 * The queue is latest-wins at the top, deliberately, and not a FIFO.  USB
 * completions have been measured stopping for 250 ms at a stretch with a full
 * pipe, which is below this layer and not ours to prevent.  Discarding the
 * backlog costs one break and resumes at current audio; a FIFO would add that
 * quarter second to output latency permanently, again on every stall.
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
 * The producer is the clock.
 *
 * Sony completes one block every four milliseconds and is steadier at it than
 * anything on this side, so the capture worker enqueues that block's packets as
 * it finishes them and the transport drains the queue one per frame.  There is
 * no cursor and no trail: nothing samples anything, so there is no phase to get
 * wrong and no distance to tune.
 *
 * uac1.c accepts only adaptive and synchronous endpoints, and an adaptive sink
 * locks its clock to the rate it is fed, so sending exactly what Sony produces
 * is the rate match itself, not an approximation of it.  A frame we cannot fill
 * tells the device to slow down, which is the correct signal rather than a
 * defect to conceal by repeating or dropping a block.
 *
 * A block arrives whole and leaves a packet at a time, so the depth swings by
 * PACKETS_PER_BLOCK every period no matter what; the only choice is where that
 * swing sits.  FLOOR is the bottom and the entire margin against the producer
 * being late: at zero, a publication and a take land on the same instant every
 * block and either order is a coin flip.  TARGET is the top, one block above the
 * floor, and both where draining begins and where an overrun is cut back to.
 * FLOOR is the only knob: each unit is a millisecond of latency bought for a
 * millisecond of producer-late tolerance.
 *
 * MAX bounds how long an entry may wait: it points into a slice and must reach
 * the wire before Sony writes that slice again, so the queue is cut back to
 * TARGET the moment it would run past it.
 */
#define PACKETS_PER_BLOCK (UAC_STREAM_CAPTURE_FRAMES / PACKET_FRAMES)
#define PCM_QUEUE_SLOTS 16u
#define PCM_QUEUE_MASK (PCM_QUEUE_SLOTS - 1u)
#define PCM_QUEUE_FLOOR 8u
#define PCM_QUEUE_TARGET (PACKETS_PER_BLOCK + PCM_QUEUE_FLOOR)
/*
 * The cut rides the peak rather than an older (COUNT - 1) reclaim bound: resting
 * at the cut is the normal operating point, so a lower bound would fire a slip
 * on ordinary jitter instead of only on a real overrun.
 */
#define PCM_QUEUE_MAX PCM_QUEUE_TARGET
/* Three requests owned by USBD, one context idle behind them being filled. */
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
 * for a thread with a 1 ms deadline; 0x20 lifts it clear while staying lower
 * priority than the capture worker's 0x12, which must not be starved because it
 * is the producer.  Per wakeup the feeder takes a context, pops a pointer and
 * blocks again, so it cannot monopolise a core.
 *
 * The shared core affinity is kept only because every session so far was
 * measured on it; the original cache-warmth reason no longer applies, since the
 * feeder no longer reads the audio.
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
 * Sony's DMA writes past the block, to the next 256-byte multiple STRICTLY
 * greater than it -- never to the block itself, even when the block is already a
 * multiple.  Ceiling rounding gets that last case wrong, so divide and add one:
 * a block that lands on a boundary still claims the next one.  Corruption here
 * is continuous rather than a click, because it lands in every block.
 */
#define DMA_WRITE_ROUND 256u
#define DMA_WRITE_SIZE(bytes) \
	(((bytes) / DMA_WRITE_ROUND + 1u) * DMA_WRITE_ROUND)
STATIC_ASSERT(UAC_STREAM_SLICE_BYTES >= DMA_WRITE_SIZE(UAC_STREAM_CAPTURE_BYTES),
	slice_must_hold_the_rounded_up_dma_write);
STATIC_ASSERT(UAC_STREAM_SLICE_COUNT > 3u &&
	(UAC_STREAM_SLICE_COUNT & PCM_SLICE_MASK) == 0u,
	slice_count_must_be_a_power_of_two_above_three);
/*
 * The floor has to be a real margin, and the working peak has to sit below the
 * cut: the closer TARGET runs to MAX, the more ordinary jitter trips the cut and
 * turns into a slip manufactured out of nothing.
 */
STATIC_ASSERT(PCM_QUEUE_FLOOR >= 1u, queue_floor_must_be_a_margin);
/*
 * Equality is allowed and measured clean: the test is strictly greater, so the
 * peak may sit on the cut and resting there is stable -- a cut needs a
 * publication to land, and that phase moves with the clock error rather than
 * every block.  What equality spends is the reclaim budget: the deepest entry
 * sits as far from the wire as the slice rotation allows, and a wrong answer is
 * the silent kind -- a slice read while it is being rewritten moves no counter
 * here.
 */
STATIC_ASSERT(PCM_QUEUE_TARGET <= PCM_QUEUE_MAX,
	queue_target_must_not_pass_the_cut);
STATIC_ASSERT(PCM_QUEUE_SLOTS > PCM_QUEUE_MAX &&
	(PCM_QUEUE_SLOTS & PCM_QUEUE_MASK) == 0u,
	queue_must_be_a_power_of_two_deeper_than_its_bound);
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
 * A context carries no buffer: the host controller reads Sony's slice in place,
 * so buffer_base is just a pointer set per submit.  CONTEXT_ALIGN keeps each
 * context's state word out of its neighbour's reservation granule.
 */
#define CONTEXT_ALIGN 64u

typedef struct {
	SonyIsoTransfer transfer __attribute__((aligned(64)));
	uint32_t callback_token;
	uint32_t sequence;
	int state;
} StreamContext;

STATIC_ASSERT(sizeof(StreamContext) == CONTEXT_ALIGN,
	context_must_tile_its_alignment);
STATIC_ASSERT(CONTEXT_ALIGN >= UAC_ERG,
	contexts_must_not_share_a_reservation_granule);

/*
 * EHCI addresses a transfer buffer as a page plus an offset, so one straddling a
 * 4 KiB boundary needs the descriptor's second page pointer programmed too.
 * Every packet the controller reads must therefore sit inside one page.
 *
 * Two of the three conditions are geometry and are settled here: packets tile a
 * slice, and the slice stride divides a page.  The third is where Sony's region
 * happens to start, which only the firmware knows, so it is checked once when
 * the region is published.
 */
STATIC_ASSERT(UAC_STREAM_SLICE_BYTES % PACKET_BYTES == 0u ||
	(UAC_STREAM_CAPTURE_BYTES / PACKET_BYTES) * PACKET_BYTES <=
		UAC_STREAM_SLICE_BYTES,
	packets_must_tile_a_slice);
STATIC_ASSERT(4096u % UAC_STREAM_SLICE_BYTES == 0u,
	slice_stride_must_divide_a_page);

/* Sent while capture has published nothing yet; read by the controller, so it
 * needs the same page containment as a slice does. */
static const uint8_t pcm_silence[PACKET_BYTES]
	__attribute__((aligned(4096)));

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
#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t completion_at;
	int completion_seen;
#endif
} __attribute__((aligned(UAC_ERG))) CallbackLifetime;

/*
 * Capture slices.  AVConfig's two RAM-output pages are contiguous -- 0x400 and
 * 0xC00, 0x800 each -- and ram_submit() takes its buffer per call, validating
 * only the frame count.  The two-page ping-pong is Sony's convention, not their
 * API's, so we rotate our own slices through that one region and their engine
 * writes the staging buffer directly.
 *
 * The payload is Sony's; the publication protocol is ours.  sequence is the
 * slice being filled, previous the one behind it that the next submit proves
 * complete; zero means none, which is why sequence numbers skip it.  base is
 * written once per session, before any worker exists.
 */
typedef struct {
	void *base;
	uint32_t sequence;
	uint32_t previous;
} __attribute__((aligned(UAC_ERG))) PcmSource;

/*
 * The handoff.  One producer, the capture worker, appending whole blocks; one
 * consumer, the feeder, taking a packet at a time.  head is the only shared
 * word -- entries are pointer-sized and aligned, so a reader either sees the
 * old one or the new one -- and publishing head after the entries is what makes
 * a block visible only once all of it is written.  A block is queued only after
 * ram_submit() has proved it complete, and the bound on how deep the queue may
 * run is what keeps the far end from being reclaimed underneath.
 */
typedef struct {
	uint32_t head;
	const uint8_t *entry[PCM_QUEUE_SLOTS];
} __attribute__((aligned(UAC_ERG))) PcmQueue;

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
	uint32_t tail;
	uint32_t write_sequence;
	/* What went out last, so a frame the producer has not filled can be fed
	 * something rather than nothing. */
	const uint8_t *last;
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
MUST_OWN_WHOLE_GRANULES(PcmQueue, pcm_queue);
MUST_OWN_WHOLE_GRANULES(PcmCursor, pcm_cursor);

static TxPump tx = { .pipe = -1 };
static CallbackLifetime cb;
/* Producer state, written by audio_tap's capture worker. */
static PcmSource src;
static PcmQueue queue;
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
static SceUID feeder_thread = -1;

/*
 * Session instrumentation, logging builds only.
 *
 *   starve   the transport won the right to submit and found the next context
 *            unfilled, so the isochronous stream gets a hole.  Audible.
 *   pcm wait the feeder had a context but no PCM to put in it, meaning
 *            pcm_copy() found the guard in flight or the slot reclaimed.  The
 *            span says whether they are spread through the session or bunched
 *            into one stretch, which is the difference between contention and
 *            a cursor that stopped making progress.
 *   slip     the writer had not advanced exactly one slice since the last
 *            boundary, so following it strictly did not land on the next slice.
 *            Positive skipped one, negative repeats one.  Audible.  A strict
 *            follow reports the phase between the two clocks as well as any
 *            real rate difference, so pairs of +1 and -1 a slice apart are
 *            jitter cancelling out rather than drift.
 *   freeze   completions stopped arriving for long enough that the pipe, not
 *            the producer, is what went quiet.
 *
 * These swing by orders of magnitude between runs of the same binary, so read
 * patterns across several sessions and never conclude anything from one number.
 */
#ifdef UAC_PSTV_ENABLE_LOGGING
#define SLIP_LOG_MAX 8u
/*
 * A completion gap this long is not jitter.  The pipe stops for one wrap of the
 * 256-entry periodic frame list, 256 frames, so any threshold well inside that
 * separates a frozen pipe from ordinary scheduling noise.
 */
#define FREEZE_GAP_US 50000u
static uint32_t starve_count;
static uint32_t submit_count;
static uint32_t pcm_wait_count;
static uint32_t slip_count;
static uint32_t freeze_count;
static uint32_t slip_at[SLIP_LOG_MAX];
static int32_t slip_delta[SLIP_LOG_MAX];
static uint32_t wait_first;
static uint32_t wait_last;
/*
 * The two clamps are the drift meter.  Neither clock is observable directly, but
 * every packet repeated is 48 frames delivered that Sony never made, and every
 * packet dropped is 48 frames made that were never delivered.  The running
 * difference over the packets sent is the rate error between Sony's audio clock
 * and the USB frame timer, in parts per million, which is the number this has
 * been guessing at all along.
 */
static uint32_t dropped_packets;
#define FREEZE_LOG_MAX 8u
static uint32_t freeze_at[FREEZE_LOG_MAX];
static uint32_t freeze_gap[FREEZE_LOG_MAX];
static uint32_t freeze_flight[FREEZE_LOG_MAX];
#define COUNT_STARVE() __atomic_add_fetch(&starve_count, 1u, __ATOMIC_RELAXED)
#define COUNT_SUBMIT() __atomic_add_fetch(&submit_count, 1u, __ATOMIC_RELAXED)

/*
 * Bracket the waits by packet number.  A count alone cannot tell fourteen
 * thousand of them spread evenly across a session from fourteen thousand in one
 * unbroken stretch at the end, and only the second means the cursor stopped.
 */
static void record_pcm_wait(void)
{
	uint32_t packet = __atomic_load_n(&submit_count, __ATOMIC_RELAXED);

	if (__atomic_fetch_add(&pcm_wait_count, 1u, __ATOMIC_RELAXED) == 0u)
		__atomic_store_n(&wait_first, packet, __ATOMIC_RELAXED);
	__atomic_store_n(&wait_last, packet, __ATOMIC_RELAXED);
}
#define COUNT_PCM_WAIT() record_pcm_wait()

/*
 * Stored rather than logged inline, because uac_log() writes to the filesystem
 * and doing that from the feeder would stall it and manufacture the very
 * problem being measured.
 *
 * A ring, so what survives is the last few rather than the first few.  Once
 * slips start repeating, the opening ones are all onset and say nothing about
 * the state it settles into.
 */
static void record_slip(int32_t dropped)
{
	uint32_t n = __atomic_fetch_add(&slip_count, 1u, __ATOMIC_RELAXED) %
		SLIP_LOG_MAX;

	slip_at[n] = __atomic_load_n(&submit_count, __ATOMIC_RELAXED);
	slip_delta[n] = dropped;
	__atomic_add_fetch(&dropped_packets, (uint32_t)dropped, __ATOMIC_RELAXED);
}
#define COUNT_SLIP(dropped) record_slip(dropped)
/*
 * Separates a frozen pipe from a producer that ran away.  A slip says the pin
 * moved by the wrong amount; it does not say which side moved.  Counting
 * completion gaps says which, and the two read together are what a session
 * line is for.
 */
static void track_freeze(void)
{
	uint32_t now = ksceKernelGetSystemTimeLow();
	uint32_t gap;
	uint32_t n;

	if (!cb.completion_seen) {
		cb.completion_seen = 1;
		cb.completion_at = now;
		return;
	}
	/* Unsigned difference stays correct across the counter's ~71 min wrap. */
	gap = now - cb.completion_at;
	cb.completion_at = now;
	if (gap < FREEZE_GAP_US)
		return;

	n = __atomic_fetch_add(&freeze_count, 1u, __ATOMIC_RELAXED) %
		FREEZE_LOG_MAX;
	freeze_at[n] = __atomic_load_n(&submit_count, __ATOMIC_RELAXED);
	freeze_gap[n] = gap;
	/*
	 * This runs before the completion drops its own reference, so the count
	 * includes the request reporting in.  One means nothing else was
	 * outstanding and the schedule had emptied behind us -- our fault, and
	 * the feeder is where to look.  Three means the controller stopped with
	 * a full pipe, which is its scheduling and not something this side can
	 * reach.  It is the only field that tells the two apart.
	 */
	freeze_flight[n] = __atomic_load_n(&tx.in_flight, __ATOMIC_ACQUIRE);
}
#define TRACK_FREEZE() track_freeze()
#define RESET_FREEZE() (cb.completion_seen = 0)
#else
#define COUNT_STARVE() ((void)0)
#define COUNT_SUBMIT() ((void)0)
#define COUNT_PCM_WAIT() ((void)0)
#define COUNT_SLIP(dropped) ((void)0)
#define TRACK_FREEZE() ((void)0)
#define RESET_FREEZE() ((void)0)
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
	/*
	 * The controller reads these slices directly, and the geometry only keeps
	 * a packet inside one page if the first slice starts on a stride boundary.
	 * Sony's region has always been aligned well past that, so this is a net
	 * rather than a branch -- but an unaligned base would show up as one
	 * packet in four arriving corrupt, which is not a thing worth debugging by
	 * ear a second time.
	 */
	if (((uintptr_t)base % UAC_STREAM_SLICE_BYTES) != 0u)
		uac_log(LOG_PREFIX "capture base %p is not %u-aligned; packets "
			"may straddle a page\n", base, UAC_STREAM_SLICE_BYTES);
	__atomic_store_n(&src.base, base, __ATOMIC_RELEASE);
}

void *uac_stream_capture_claim(void)
{
	uint32_t sequence = next_pcm_sequence(src.sequence);

	src.previous = src.sequence;
	src.sequence = sequence;
	return slice(sequence);
}

/*
 * Append the completed block, one entry per USB frame it is worth.
 *
 * Entries are written before head moves, and head is released, so the feeder
 * sees a block either not at all or entire.  Nothing here can block or fail:
 * this runs on the capture worker between two ram_submit() calls, and Sony's
 * mailbox is one deep, so time spent here is time the engine has nothing queued.
 */
void uac_stream_capture_ready(void)
{
	uint32_t sequence = src.previous;
	const uint8_t *base;
	uint32_t head;
	uint32_t i;

	/* The first submit of a session has nothing queued behind it. */
	if (sequence == 0u)
		return;
	base = slice(sequence);
	head = queue.head;
	for (i = 0; i < PACKETS_PER_BLOCK; ++i)
		queue.entry[(head + i) & PCM_QUEUE_MASK] = base + i * PACKET_BYTES;
	__atomic_store_n(&queue.head, head + PACKETS_PER_BLOCK, __ATOMIC_RELEASE);
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

	/*
	 * No cache maintenance.  Nothing on this side writes the bytes the
	 * controller is about to read -- Sony's engine put them in memory by DMA
	 * and we only ever load from them -- so there is no dirty line to clean,
	 * and cleaning would risk writing a stale one back over the engine's work.
	 *
	 * The cost of that is the buffer staying live until this completes.  A
	 * queued request holds a pointer into a slice for as long as the schedule
	 * takes to reach it, which is a frame or three normally and one whole wrap
	 * of the periodic list when the pipe freezes.  The ring turns over in
	 * COUNT block periods, so a freeze outlives it and the controller sends
	 * whatever the engine has since written there.  That is the price of not
	 * copying, and it is paid in torn audio on the far side of a freeze.
	 */
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

	TRACK_FREEZE();

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

/*
 * Take the oldest queued packet, or say there is none.
 *
 * Two bounds, and they mean opposite things.  Empty is the transport having
 * outrun the producer: the frame goes unfilled, which on an adaptive endpoint is
 * how the device is told its clock is fast, so it is a wait rather than an error
 * and nothing is invented to fill it.  Past MAX is the producer having outrun
 * the transport far enough that the oldest entries point into a slice about to
 * be reclaimed, so the queue is cut back to TARGET and the packets in between
 * gone -- audible, counted, and the only place a discontinuity can enter.
 */
static int pcm_take(const void **packet)
{
	uint32_t head = __atomic_load_n(&queue.head, __ATOMIC_ACQUIRE);
	uint32_t depth = head - cur.tail;

	if (depth == 0u)
		return 1;
	if (depth > PCM_QUEUE_MAX) {
		COUNT_SLIP((int32_t)(depth - PCM_QUEUE_TARGET));
		cur.tail = head - PCM_QUEUE_TARGET;
	}
	*packet = queue.entry[cur.tail & PCM_QUEUE_MASK];
	cur.tail++;
	return 0;
}
static void pcm_next(const void **packet)
{
	/*
	 * Priming.  uac1 has already selected the alternate setting, so the device
	 * is live and expecting a packet every millisecond well before capture has
	 * produced anything -- the tap still has to acquire the route.  Send
	 * silence across that window rather than nothing: a gap on an active
	 * isochronous endpoint reads as an invalid stream to the device, while
	 * continuous silence lets it lock cleanly and audio simply fades in.
	 *
	 * Starting lands the queue on TARGET rather than merely past it.  Blocks
	 * arrive whole, so the first depth to clear the threshold overshoots it,
	 * and beginning there would fix the swing one block higher than asked for
	 * and keep it there all session.  Dropping to TARGET costs the packets in
	 * between, which are the oldest audio of a stream that has not started.
	 */
	if (!cur.valid) {
		uint32_t head = __atomic_load_n(&queue.head, __ATOMIC_ACQUIRE);

		if (head - cur.tail < PCM_QUEUE_TARGET) {
			*packet = pcm_silence;
			return;
		}
		cur.tail = head - PCM_QUEUE_TARGET;
		cur.valid = 1;
	}

	/*
	 * Every frame gets a packet, even the ones the producer has not filled.
	 * Holding the rate steady at one packet per frame rather than leaving the
	 * gap: under-feeding an adaptive sink makes its clock-hunting loop chase the
	 * irregular arrival rate, which lands on every sample while it wanders, so
	 * repeat the last packet -- a millisecond of stutter, counted -- instead.
	 */
	if (pcm_take(packet) != 0) {
		COUNT_PCM_WAIT();
		*packet = cur.last;
	}
	cur.last = *packet;
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
static void queue_next_source_packet(void)
{
	StreamContext *context;
	const void *packet;
	uint32_t sequence;

	if (!wait_for_free_context(&context))
		return;
	pcm_next(&packet);

	/*
	 * The controller reads Sony's slice in place, so the bytes are not ours and
	 * stay live until the transfer completes -- see the note on submit_context().
	 */
	context->transfer.buffer_base = (void *)(uintptr_t)packet;

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
}

#ifdef UAC_PSTV_ENABLE_LOGGING
static void report_session(void)
{
	uint32_t total = __atomic_load_n(&slip_count, __ATOMIC_RELAXED);
	uint32_t shown = total < SLIP_LOG_MAX ? total : SLIP_LOG_MAX;
	uint32_t first = total - shown;
	uint32_t i;

	uac_log(LOG_PREFIX
		"session: %u packets, %u starved, %u pcm waits, %u slips, "
		"%u freezes\n",
		__atomic_load_n(&submit_count, __ATOMIC_RELAXED),
		__atomic_load_n(&starve_count, __ATOMIC_RELAXED),
		__atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED), total,
		__atomic_load_n(&freeze_count, __ATOMIC_RELAXED));
	/* Numbered by their real ordinal, so a run reads 331..338 rather than 1..8
	 * and the packet numbers show how far apart they are firing. */
	for (i = 0; i < shown; ++i) {
		uint32_t k = (first + i) % SLIP_LOG_MAX;

		uac_log(LOG_PREFIX "  slip %u at packet %u, dropped %+d packets\n",
			first + i + 1u, slip_at[k], slip_delta[k]);
	}
	if (__atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED) != 0u)
		uac_log(LOG_PREFIX "  pcm waits spanned packets %u..%u\n",
			__atomic_load_n(&wait_first, __ATOMIC_RELAXED),
			__atomic_load_n(&wait_last, __ATOMIC_RELAXED));
	{
		uint32_t fz = __atomic_load_n(&freeze_count, __ATOMIC_RELAXED);
		uint32_t fz_shown = fz < FREEZE_LOG_MAX ? fz : FREEZE_LOG_MAX;
		uint32_t fz_first = fz - fz_shown;

		for (i = 0; i < fz_shown; ++i) {
			uint32_t k = (fz_first + i) % FREEZE_LOG_MAX;

			uac_log(LOG_PREFIX
				"  freeze %u at packet %u, %u us, %u in flight\n",
				fz_first + i + 1u, freeze_at[k], freeze_gap[k],
				freeze_flight[k]);
		}
	}
	/*
	 * The feeder has already left its loop, so this is where the queue ended
	 * up.  Anywhere in FLOOR..TARGET says the two clocks kept station; at zero
	 * the transport was outrunning the producer and the endpoint was going
	 * unfed; near MAX the reverse.
	 */
	{
		uint32_t head = __atomic_load_n(&queue.head, __ATOMIC_ACQUIRE);

		uac_log(LOG_PREFIX
			"  final queue: head %u tail %u depth %u valid %d "
			"(swing %u..%u, max %u)\n",
			head, cur.tail, head - cur.tail, cur.valid,
			PCM_QUEUE_FLOOR, PCM_QUEUE_TARGET, PCM_QUEUE_MAX);
	}
	{
		uint32_t sent = __atomic_load_n(&submit_count, __ATOMIC_RELAXED);
		uint32_t rep = __atomic_load_n(&pcm_wait_count, __ATOMIC_RELAXED);
		uint32_t drop = __atomic_load_n(&dropped_packets, __ATOMIC_RELAXED);
		/* Signed, and scaled before the divide so integer maths keeps the
		 * resolution: one packet in a million is one ppm. */
		int32_t net = (int32_t)rep - (int32_t)drop;
		int32_t ppm = sent ? (int32_t)((int64_t)net * 1000000 / sent) : 0;

		uac_log(LOG_PREFIX
			"  clock: %u repeated, %u dropped of %u packets"
			" = %+d ppm (%s)\n",
			rep, drop, sent, ppm,
			ppm > 0 ? "USB frame timer faster than Sony" :
			ppm < 0 ? "Sony faster than USB frame timer" : "matched");
	}

	__atomic_store_n(&submit_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&starve_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&pcm_wait_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&slip_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&freeze_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&dropped_packets, 0u, __ATOMIC_RELAXED);
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
	while (stream_running())
		queue_next_source_packet();
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

	if (pipe_id < 0 || free_event < 0)
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
	RESET_FREEZE();
	memset(contexts, 0, sizeof(contexts));

	/*
	 * The source and the queue are reset here rather than on the feeder, so
	 * they are ordered before audio_tap_begin() by the session thread itself.
	 * An empty queue is what holds the feeder on silence until the producer has
	 * published enough to start from.
	 */
	memset(&src, 0, sizeof(src));
	memset(&queue, 0, sizeof(queue));
	cur.tail = 0;
	cur.valid = 0;

	for (index = 0; index < CONTEXT_COUNT; ++index) {
		StreamContext *context = &contexts[index];

		context->callback_token = generation | index;
		context->transfer.buffer_base = (void *)(uintptr_t)pcm_silence;
		context->transfer.num_packets = 1u;
		__atomic_store_n(&context->state, CONTEXT_FREE, __ATOMIC_RELEASE);
	}

	reset_event(free_event);
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
	return 0;
}

int uac_stream_shutdown(void)
{
	int result;

	uac_stream_stop();
	if (__atomic_load_n(&feeder_thread, __ATOMIC_ACQUIRE) >= 0 ||
	    __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) != STREAM_IDLE)
		return -1;
	if (free_event >= 0) {
		result = ksceKernelDeleteEventFlag(free_event);
		if (result < 0)
			return result;
		free_event = -1;
	}
	return 0;
}
