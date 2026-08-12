#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

/*
 * Frames per captured block, and the dominant term in output latency: the
 * feeder starts one whole block behind the newest, because blocks arrive in
 * discrete steps while the USB side drains continuously, so anything less runs
 * dry between arrivals.  Latency is therefore about two block periods plus the
 * two 1 ms transport contexts.  Must stay a whole number of USB packets; see
 * the assert in stream.c.
 */
#define UAC_STREAM_CAPTURE_FRAMES 240u
#define UAC_STREAM_CAPTURE_BYTES (UAC_STREAM_CAPTURE_FRAMES * 4u)

int uac_stream_init(void);
int uac_stream_publish(const void *pcm);
int uac_stream_start(int pipe_id);
void uac_stream_stop(void);
void uac_stream_pipe_closed(int pipe_id);
int uac_stream_is_active(void);
int uac_stream_shutdown(void);

#endif
