#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

int uac_stream_start(int pipe_id);
void uac_stream_stop(void);
void uac_stream_pipe_closed(int pipe_id);

#endif
