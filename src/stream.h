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
 * 240 frames is 5 ms a block and five packets to a slice, so the capture worker
 * wakes 200 times a second where 192-frame blocks woke it 250 and 96-frame ones
 * 500.  Each of those wakeups runs ram_submit()'s spinlocks and event wait on
 * the same core as the feeder, which is the whole reason to want fewer.
 *
 * It is also the largest block the region can carry.  The write rounds to 1024
 * either way, so 240 frames spend 960 of those bytes where 192 spent 768; the
 * next size up rounds to 1280 and four of those overrun the region.
 */
#define UAC_STREAM_CAPTURE_FRAMES 240u
#define UAC_STREAM_CAPTURE_BYTES (UAC_STREAM_CAPTURE_FRAMES * 4u)

/*
 * Staging is AVConfig's own 0x1000 RAM-output region, cut into slices Sony's
 * engine fills directly.  Four of them at a 1024-byte stride fill the region
 * exactly, from 0x0400 to 0x1400, ending four bytes before send_worker_active.
 *
 * The stride is not the block size.  Sony's DMA rounds its write up to the next
 * 256-byte multiple strictly greater than the block, so a slice has to hold the
 * rounded figure or it spills into the next one -- audible as continuous
 * distortion rather than a click, because it lands in every block.  768 bytes
 * needs no rounding by the obvious reading and still distorted at a 768 stride,
 * which is what established the rule.  stream.c asserts it; resolver.c asserts
 * that the region holds them all.
 */
#define UAC_STREAM_SLICE_BYTES 0x400u
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
