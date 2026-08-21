#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

/*
 * Frames per captured block, and the dominant term in output latency: the
 * reader trails whole blocks, so latency is block periods.  Must stay a whole
 * number of USB packets and fit one slice; stream.c asserts both.
 *
 * 240 is 5 ms a block, so the capture worker wakes 200 times a second against
 * 250 at 192 frames and 500 at 96.  Each wakeup runs ram_submit()'s spinlocks
 * and event wait, which is the reason to want fewer.  It is also the largest
 * block the region carries: the write rounds to 1024 either way, and the next
 * size up rounds to 1280, four of which overrun.
 */
#define UAC_STREAM_CAPTURE_FRAMES 240u
#define UAC_STREAM_CAPTURE_BYTES (UAC_STREAM_CAPTURE_FRAMES * 4u)

/*
 * Staging is AVConfig's own 0x1000 RAM-output region, cut into slices Sony's
 * engine fills directly.  Four at a 1024-byte stride fill it exactly, 0x0400 to
 * 0x1400, ending four bytes before send_worker_active.
 *
 * The stride is not the block size: Sony's DMA writes past the block, so a
 * slice must hold the rounded figure or spill into its neighbour.  stream.c has
 * the rule and asserts it; resolver.c asserts the region holds them all.
 */
#define UAC_STREAM_SLICE_BYTES 0x400u
#define UAC_STREAM_SLICE_COUNT 4u

int uac_stream_init(void);

/* Capture rotation, driven by audio_tap.  claim() returns the next slice for
 * ram_submit(); ready() is called once that submit returns, which is what
 * proves the slice before it complete and queues its packets. */
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
