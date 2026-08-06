#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

#include <stdint.h>

int uac_stream_start(int pipe_id, uint16_t packet_bytes, uint8_t speed,
	uint8_t interval);
void uac_stream_stop(void);

#endif
