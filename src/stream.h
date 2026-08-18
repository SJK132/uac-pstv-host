#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

/*
 * Frames per captured block, and the dominant term in output latency: the
 * feeder follows one whole block behind the one under Sony's pen, because
 * blocks arrive in discrete steps while the USB side drains continuously.
 * Latency is therefore about two block periods plus the three scheduled 1 ms
 * USB requests.  Must stay a whole number of USB packets and fit one slice;
 * see the assertions in stream.c.
 *
 * 192 frames is 4 ms a block, so the capture worker wakes half as often as it
 * did at 96 -- and each of those wakeups runs ram_submit()'s spinlocks and event
 * wait on the same core as the feeder, so halving them is the point.
 */
#define UAC_STREAM_CAPTURE_FRAMES 192u
#define UAC_STREAM_CAPTURE_BYTES (UAC_STREAM_CAPTURE_FRAMES * 4u)

/*
 * Staging is AVConfig's own 0x1000 RAM-output region, cut into slices Sony's
 * engine fills directly.  Two slices are always spoken for -- the one in the
 * mailbox and the one claimed next -- so the cursor may trail by at most
 * COUNT - 3, and what is left over is the slack before Sony reclaims the slice
 * being read.  Four 192-frame slices at trail one leave one block period, 4 ms.
 *
 * The stride is not the block size.  Sony's DMA rounds its write up to a
 * 256-byte multiple, so a slice has to hold the rounded figure or it spills
 * into the next one -- which is audible as distortion rather than a click,
 * because the corruption is continuous.  768 is already three whole rounds, so
 * this geometry has nothing to round.  stream.c asserts it; resolver.c asserts
 * that the region holds them all.
 */
#define UAC_STREAM_SLICE_BYTES 0x300u
#define UAC_STREAM_SLICE_COUNT 4u

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
