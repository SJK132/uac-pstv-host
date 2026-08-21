/*
 * PCM handoff and USB transport.
 *
 * Two halves that meet in the middle:
 *
 *   - the source side is Sony's own audio engine, writing captured PCM into
 *     slices of AVConfig's RAM-output region and handing each finished block
 *     to a queue as five 1 ms packets;
 *   - the transport side is the completion callback and nothing else: each
 *     finished request takes the next packet, copies it into its own context
 *     and hands the same context straight back to USBD.
 *
 * Nothing samples a position: the producer sets both the content and the pace,
 * and the queue depth is the only state between them.  Its two bounds are the
 * only places a discontinuity can enter, and both are counted.
 *
 * The queue is latest-wins at the top, deliberately, and not a FIFO.  USB
 * completions have been measured stopping for 250 ms at a stretch with a full
 * pipe, which is below this layer and not ours to prevent.  Discarding the
 * backlog costs one break and resumes at current audio; a FIFO would add that
 * quarter second to output latency permanently, again on every stall.
 *
 * session.c owns this file's lifecycle: uac_stream_start(), uac_stream_stop()
 * and uac_stream_pipe_closed() are called from the session thread and nowhere
 * else.  Starting primes the pipe by hand and every submit after that is made
 * by a completion, so stopping is only a matter of refusing the next one and
 * waiting for the outstanding requests to report in.
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
 * When a block finishes, read the block PCM_TRAIL_BLOCKS behind it, from its
 * start.  PCM_READ_BACK is that distance in packets, and it is one block more
 * than the trail: reaching the start of the block one back means spanning that
 * block and the newer one.
 *
 * Two is measured, not derived.  One desynchronised at this block size, which
 * no account of the producer's slack predicts, so lower it only on hardware.
 * PCM_QUEUE_MAX rides PCM_READ_BACK so an overrun lands where priming does.
 */
#define PACKETS_PER_BLOCK (UAC_STREAM_CAPTURE_FRAMES / PACKET_FRAMES)
#define PCM_TRAIL_BLOCKS 2u
#define PCM_READ_BACK ((PCM_TRAIL_BLOCKS + 1u) * PACKETS_PER_BLOCK)
#define PCM_QUEUE_SLOTS 16u
#define PCM_QUEUE_MASK (PCM_QUEUE_SLOTS - 1u)
#define PCM_QUEUE_MAX PCM_READ_BACK
/* Three requests owned by USBD, and a fourth context so the token index stays a
 * two-bit mask. */
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

#define STOP_POLL_US 1000u
#define STOP_POLLS 50u
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
 * greater than it -- never to the block itself, even when the block is already
 * a multiple.  Measured: 384 bytes writes 512, 576 writes 768, 768 writes 1024.
 * A ceiling rounding fits the first two and lets 768-in-768 through, which is
 * continuous distortion rather than a click because it lands in every block.
 */
#define DMA_WRITE_ROUND 256u
#define DMA_WRITE_SIZE(bytes) \
	(((bytes) / DMA_WRITE_ROUND + 1u) * DMA_WRITE_ROUND)
STATIC_ASSERT(
	UAC_STREAM_SLICE_BYTES >= DMA_WRITE_SIZE(UAC_STREAM_CAPTURE_BYTES),
	slice_must_hold_the_rounded_up_dma_write);
STATIC_ASSERT(UAC_STREAM_SLICE_COUNT > 3u &&
	(UAC_STREAM_SLICE_COUNT & PCM_SLICE_MASK) == 0u,
	slice_count_must_be_a_power_of_two_above_three);
/*
 * A trail of zero would put the reader on the block the engine is still handing
 * over, and one leaves it on the block behind with nothing in hand for the
 * producer to be late in.  Two is the first value with any slack, and by ear it
 * is where the hiss stops.
 */
STATIC_ASSERT(PCM_TRAIL_BLOCKS >= 1u, reader_must_trail_the_engine);
/*
 * The reader has to stay inside the ring, since an entry further back than the
 * ring is deep has been overwritten by the producer.  Strictly greater, not
 * equal: at equality the oldest readable entry is the one about to be written.
 *
 * Depth itself does pass this during a stall -- completions stop arriving so
 * nothing is taken while the producer keeps publishing, and it reaches hundreds
 * -- but the cut runs before every read, so what is read is always the newest
 * PCM_READ_BACK, which are always intact.
 */
STATIC_ASSERT(PCM_QUEUE_SLOTS > PCM_READ_BACK &&
	(PCM_QUEUE_SLOTS & PCM_QUEUE_MASK) == 0u,
	queue_must_be_a_power_of_two_deeper_than_the_read_back);
/*
 * How long a slice must survive after publication -- the one figure here that
 * is measured rather than derived, since nothing on this side can observe when
 * the engine comes back to a slice.  18 packets played clean at this geometry.
 *
 * An assertion because the failure is silent: a slice read while the engine
 * rewrites it passes the queue, passes the transfer, and only sounds wrong.  A
 * geometry past the evidence should stop the build rather than the ear.
 */
#define PCM_PROVEN_READ_DEPTH 18u
STATIC_ASSERT(PCM_READ_BACK <= PCM_PROVEN_READ_DEPTH,
	read_depth_must_stay_inside_what_has_been_measured);
/* Keeps every slice line-aligned, not just the first. */
STATIC_ASSERT(UAC_STREAM_SLICE_BYTES % UAC_ERG == 0u,
	slice_stride_must_tile_lines);

enum {
	STREAM_IDLE = 0,
	STREAM_STARTING,
	STREAM_RUNNING,
	STREAM_STOPPING,
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
 * storage the host controller reads, and EHCI addresses a transfer buffer as a
 * page plus an offset, so one straddling a 4 KiB boundary needs the
 * descriptor's second page pointer programmed too.  Aligning each context to a
 * divisor of the page size, buffer first and no larger than that alignment,
 * keeps every buffer inside one page by construction.
 */
#define CONTEXT_ALIGN 256u

typedef struct {
	uint8_t buffer[PACKET_BYTES] __attribute__((aligned(64)));
	SonyIsoTransfer transfer __attribute__((aligned(64)));
	uint32_t callback_token;
} StreamContext;

STATIC_ASSERT(sizeof(StreamContext) == CONTEXT_ALIGN,
	context_must_tile_its_alignment);
STATIC_ASSERT(PACKET_BYTES <= CONTEXT_ALIGN, context_buffer_must_fit_one_block);
STATIC_ASSERT(4096u % CONTEXT_ALIGN == 0u, context_blocks_must_tile_a_page);

/* Packets tile a slice, so a copy never spans two. */
STATIC_ASSERT(UAC_STREAM_CAPTURE_BYTES % PACKET_BYTES == 0u,
	packets_must_tile_a_slice);

/* Sent while capture has published nothing yet. */
static const uint8_t pcm_silence[PACKET_BYTES] __attribute__((aligned(64)));

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

/* Written by the session thread, read by the USBD callback on every packet. */
typedef struct {
	int state;
	uint32_t generation;
	int pipe;
} __attribute__((aligned(UAC_ERG))) TxPump;

/*
 * Callback-thread private in steady state, and the reason it is not folded into
 * TxPump: the two are written by different threads, and sharing a granule would
 * let a store to one clear an ldrex the other holds on the other.
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
 * The payload is Sony's; the publication protocol is ours.  guard[] is odd
 * while a slice is in flight, even once the following submit proves it
 * complete.  sequence and previous are the slices being filled and queued
 * behind it; zero means none, which is why sequence numbers skip it.  base is
 * written once per session, before any worker exists.
 */
typedef struct {
	void *base;
	uint32_t sequence;
	uint32_t previous;
} __attribute__((aligned(UAC_ERG))) PcmSource;

/*
 * The handoff.  One producer, the capture worker, appending whole blocks; one
 * consumer, the completion callback, taking a packet at a time.  head is the
 * only shared word, and publishing it after the entries is what makes a block
 * visible only once all of it is written.
 */
typedef struct {
	uint32_t head;
	const uint8_t *entry[PCM_QUEUE_SLOTS];
} __attribute__((aligned(UAC_ERG))) PcmQueue;

/*
 * Feeder-private packetizer cursor.  The callback never reads any of it, which
 * is exactly why it must not sit next to what the callback does ldrex on.
 *
 */
typedef struct {
	uint32_t tail;
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
/* Consumer state, touched only by the completion callback. */
static PcmCursor cur;

/*
 * Two event flags, one bit each.  Never merge them into one flag.
 *
 * Both waiters use SCE_EVENT_WAITCLEAR, which clears the entire flag rather
 * than only the matched pattern, so a shared flag lets either wake destroy the
 * other subsystem's pending wakeup.  That is unrecoverable during priming:
 *
 * At file scope rather than grouped: none of the three has a per-packet writer,
 * so nothing here can invalidate a granule another core is holding.
 */

/*
 * Session instrumentation, logging builds only.
 *
 *   starve   the transport won the right to submit and found the next context
 *            unfilled, so the isochronous stream gets a hole.  Audible.
 *   repeat   a completion came round with no packet queued, meaning
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
static uint32_t submit_count;
static uint32_t pcm_wait_count;
static uint32_t slip_count;
static uint32_t freeze_count;
static uint32_t slip_at[SLIP_LOG_MAX];
static int32_t slip_delta[SLIP_LOG_MAX];
static uint32_t wait_first;
static uint32_t wait_last;
/*
 * The two clamps are a drift meter: a packet repeated is 48 frames delivered
 * that Sony never made, a packet dropped is 48 frames made and never delivered.
 * The difference over the packets sent is the rate error between Sony's clock
 * and the USB frame timer, in parts per million.
 */
static uint32_t dropped_packets;
#define FREEZE_LOG_MAX 8u
static uint32_t freeze_at[FREEZE_LOG_MAX];
static uint32_t freeze_gap[FREEZE_LOG_MAX];
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
 * and doing that from the callback would stall it and manufacture the very
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
}
#define TRACK_FREEZE() track_freeze()
#define RESET_FREEZE() (cb.completion_seen = 0)
#else
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
 * Entries are written before head moves, and head is released, so a block is
 * seen either not at all or entire.  Nothing here can block or fail:
 * this runs on the capture worker between two ram_submit() calls, and Sony's
 * mailbox is one deep, so time spent here is time the engine has nothing
 * queued.
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
	    __atomic_compare_exchange_n(&cb.guard, &expected, 0u, 0,
		__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		__atomic_store_n(&tx.state, STREAM_IDLE, __ATOMIC_RELEASE);
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
	 * After the aborts, not before: a teardown should not pay for a flush.
	 * This buffer was written with the CPU, so the lines have to reach memory
	 * before the controller reads them.
	 */
	ksceKernelDcacheCleanRange(context->buffer, PACKET_BYTES);
	return ksceUsbdIsochronousTransfer(
		pipe,
		(ksceUsbdIsochTransfer *)(void *)&context->transfer,
		transfer_done,
		(void *)(uintptr_t)context->callback_token);
}

/*
 * Take the next packet and hand the same context straight back to USBD.
 *
 * A completion frees exactly one context -- the one it was called for -- so no
 * thread or handshake is needed, and PCM order falls out rather than being
 * arranged: isochronous requests complete in the order they were queued, so the
 * context reporting in is always the oldest.
 */
static void pcm_next(const void **packet);

static void refill_context(StreamContext *context)
{
	const void *packet;
	int submitted;

	pcm_next(&packet);
	memcpy(context->buffer, packet, PACKET_BYTES);
	COUNT_SUBMIT();
	submitted = submit_context(context);
	if (submitted != 0)
		stop_stream(submitted < 0 ? submitted : 0, USBD_CC_NOERR);
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

	refill_context(context);

out:
	callback_leave();
}

/*
 * The seqlock is gone with the copy, and it could not have survived it.  Its
 * guarantee was that the bytes we took were whole at the moment we took them,
 * which needed a read to bracket; the controller's read happens later and
 * elsewhere, so there is nothing left to bracket.  What remains is a claim
 * check: the slot still belongs to the sequence we mean to send, so it has been
 * published and not yet reclaimed.  That is a weaker statement and is meant to
 * be read as one.
 */
/*
 * Take the oldest queued packet, or report empty.  Past MAX the producer has
 * run far enough ahead that the oldest entries point into a slice about to be
 * reclaimed, so the queue is cut back and the packets between are dropped --
 * audible, counted, and the only place a discontinuity can enter.
 */
static int pcm_take(const void **packet)
{
	uint32_t head = __atomic_load_n(&queue.head, __ATOMIC_ACQUIRE);
	uint32_t depth = head - cur.tail;

	if (depth == 0u)
		return 1;
	if (depth > PCM_QUEUE_MAX) {
		COUNT_SLIP((int32_t)(depth - PCM_READ_BACK));
		cur.tail = head - PCM_READ_BACK;
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
	 * Starting lands on PCM_READ_BACK rather than merely past it: blocks arrive
	 * whole, so the first depth to clear it overshoots, and beginning there
	 * would hold the queue a block deeper for the whole session.
	 */
	if (!cur.valid) {
		uint32_t head = __atomic_load_n(&queue.head, __ATOMIC_ACQUIRE);

		if (head - cur.tail < PCM_READ_BACK) {
			*packet = pcm_silence;
			return;
		}
		cur.tail = head - PCM_READ_BACK;
		cur.valid = 1;
	}

	/*
	 * Every frame gets a packet, even one the producer has not filled.  Leaving
	 * it empty under-feeds the device, and a sink recovering its clock from the
	 * arrival rate chases that rather than settling: the damage lands on every
	 * sample instead of at the gap.  A repeat costs a millisecond of stutter,
	 * once, and is counted.
	 */
	if (pcm_take(packet) != 0) {
		COUNT_PCM_WAIT();
		*packet = cur.last;
	}
	cur.last = *packet;
}

#ifdef UAC_PSTV_ENABLE_LOGGING
static void report_session(void)
{
	uint32_t total = __atomic_load_n(&slip_count, __ATOMIC_RELAXED);
	uint32_t shown = total < SLIP_LOG_MAX ? total : SLIP_LOG_MAX;
	uint32_t first = total - shown;
	uint32_t i;

	uac_log(LOG_PREFIX
		"session: %u packets, %u repeats, %u slips, %u freezes\n",
		__atomic_load_n(&submit_count, __ATOMIC_RELAXED),
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

			uac_log(LOG_PREFIX "  freeze %u at packet %u, %u us\n",
				fz_first + i + 1u, freeze_at[k], freeze_gap[k]);
		}
	}
	/*
	 * Submitting has already been refused, so this is where the queue ended
	 * up.  Anywhere in FLOOR..TARGET says the two clocks kept station; at zero
	 * the transport was outrunning the producer and the endpoint was going
	 * unfed; near MAX the reverse.
	 */
	{
		uint32_t head = __atomic_load_n(&queue.head, __ATOMIC_ACQUIRE);

		uac_log(LOG_PREFIX
			"  final queue: head %u tail %u depth %u valid %d "
			"(reading %u blocks back = %u..%u packets of %u)\n",
			head, cur.tail, head - cur.tail, cur.valid,
			PCM_TRAIL_BLOCKS, PCM_READ_BACK - PACKETS_PER_BLOCK,
			PCM_READ_BACK, PCM_QUEUE_SLOTS);
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
	__atomic_store_n(&pcm_wait_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&slip_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&freeze_count, 0u, __ATOMIC_RELAXED);
	__atomic_store_n(&dropped_packets, 0u, __ATOMIC_RELAXED);
}
#define REPORT_SESSION() report_session()
#else
#define REPORT_SESSION() ((void)0)
#endif

int uac_stream_start(int pipe_id)
{
	int expected = STREAM_IDLE;
	uint32_t generation;
	uint32_t index;

	if (pipe_id < 0)
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
	RESET_FREEZE();
	memset(contexts, 0, sizeof(contexts));

	/*
	 * The source and the queue are reset before anything is submitted, so they
	 * are ordered ahead of audio_tap_begin() by the session thread itself.  An
	 * empty queue is what makes the first packets silence until the producer
	 * has published enough to start from.
	 */
	memset(&src, 0, sizeof(src));
	memset(&queue, 0, sizeof(queue));
	cur.tail = 0;
	cur.valid = 0;

	for (index = 0; index < CONTEXT_COUNT; ++index) {
		StreamContext *context = &contexts[index];

		context->callback_token = generation | index;
		context->transfer.buffer_base = context->buffer;
		context->transfer.num_packets = 1u;
	}

	/* RUNNING before the first submit: submit_context() refuses to queue
	 * anything in any other state, and its completion is what keeps the
	 * schedule fed from here on. */
	expected = STREAM_STARTING;
	if (!__atomic_compare_exchange_n(&tx.state, &expected, STREAM_RUNNING,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return -1;

	/*
	 * Prime the pipe by hand, once.  Every later submit is made by the
	 * completion of the one before it, so this is the only place a request is
	 * queued from outside a callback.
	 */
	for (index = 0; index < MAX_IN_FLIGHT; ++index)
		refill_context(&contexts[index]);

	uac_log(LOG_PREFIX "ready: %u contexts, %u in flight, %u bytes/request\n",
		CONTEXT_COUNT, MAX_IN_FLIGHT, PACKET_BYTES);
	return 0;
}

void uac_stream_stop(void)
{
	uint32_t wait;

	if (__atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_IDLE)
		return;

	/*
	 * Refusing the next submit is the whole of the stop.  Every context is
	 * resubmitted by its own completion, so once submit_context() declines
	 * them the outstanding requests drain themselves and the reference count
	 * falls to zero without anything to reap.
	 */
	__atomic_store_n(&tx.state, STREAM_STOPPING, __ATOMIC_RELEASE);
	__atomic_fetch_or(&cb.guard, CALLBACK_CLOSING, __ATOMIC_ACQ_REL);

	REPORT_SESSION();
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
	finish_retire();
}

int uac_stream_init(void)
{
	/* Nothing to set up: the transport is static storage and a callback. */
	return 0;
}

int uac_stream_shutdown(void)
{
	uac_stream_stop();
	return __atomic_load_n(&tx.state, __ATOMIC_ACQUIRE) == STREAM_IDLE ?
		0 : -1;
}
