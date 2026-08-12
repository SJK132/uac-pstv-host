#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

#define UAC_STREAM_CAPTURE_FRAMES 480u
#define UAC_STREAM_CAPTURE_BYTES (UAC_STREAM_CAPTURE_FRAMES * 4u)

int uac_stream_init(void);
int uac_stream_publish(const void *pcm);
int uac_stream_start(int pipe_id);
void uac_stream_stop(void);
void uac_stream_pipe_closed(int pipe_id);
int uac_stream_is_active(void);
int uac_stream_shutdown(void);

#endif
