#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

/*
 * Frames per captured block, and the dominant term in output latency: the
 * feeder starts one whole block behind the newest, because blocks arrive in
 * discrete steps while the USB side drains continuously, so anything less runs
 * dry between arrivals.  Latency is therefore about two block periods plus the
 * three scheduled 1 ms USB requests.  Must stay a whole number of USB packets
 * and fit one slice; see the assertions in stream.c.
 */
#define UAC_STREAM_CAPTURE_FRAMES 144u
#define UAC_STREAM_CAPTURE_BYTES (UAC_STREAM_CAPTURE_FRAMES * 4u)

/*
 * Staging is AVConfig's own 0x1000 RAM-output region, cut into slices that
 * Sony's audio engine fills directly.  The region is fixed, so size and count
 * trade against each other, and the count is what buys stall tolerance: three
 * slices are always spoken for (see PCM_MAX_TRAIL), leaving COUNT - 3 block
 * periods of slack.  Seven 144-frame slices pack 4032 of the 4096 bytes and
 * give four, or 12 ms.
 *
 * The cost of a smaller block is one more capture wakeup, and each of those
 * preempts the feeder on its own core -- which matters more than the cycles,
 * because the feeder is the thread carrying the 1 ms deadline.  333 Hz is where
 * the slack this buys stops outrunning that.  Nothing here needs COUNT to be a
 * power of two; slice indices are carried, never derived.
 *
 * resolver.c checks the region against what these two say it needs.
 */
#define UAC_STREAM_SLICE_BYTES 0x240u
#define UAC_STREAM_SLICE_COUNT 7u

int uac_stream_init(void);

/*
 * Capture rotation, driven by audio_tap's worker.  claim() marks the next
 * slice in flight and returns the address to hand ram_submit(); ready() is
 * called once that submit returns, which is what proves the slice before it is
 * complete.
 */
void uac_stream_capture_region(void *base);
void *uac_stream_capture_claim(void);
void uac_stream_capture_ready(void);

/* Report an asynchronous source failure without blocking its worker. */
void uac_stream_source_failed(int result);
int uac_stream_start(int pipe_id);
void uac_stream_stop(void);
void uac_stream_pipe_closed(int pipe_id);
int uac_stream_shutdown(void);

#endif
