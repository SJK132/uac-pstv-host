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
 * Staging is AVConfig's own 0x1000 RAM-output region, cut into slices Sony's
 * engine fills directly.  Count buys stall tolerance -- three slices are always
 * spoken for, leaving COUNT - 3 block periods of slack -- and a smaller block
 * costs one more capture wakeup, each of which preempts the feeder on its own
 * core.  Seven 144-frame slices pack 4032 of 4096 bytes for 12 ms at 333 Hz.
 * COUNT need not be a power of two.  resolver.c checks the region fits.
 */
#define UAC_STREAM_SLICE_BYTES 0x240u
#define UAC_STREAM_SLICE_COUNT 7u

int uac_stream_init(void);

/* Capture rotation, driven by audio_tap. claim() marks the next slice in flight
 * and returns its address for ram_submit(); ready() is called once that submit
 * returns, which is what proves the slice before it is complete. */
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
