#ifndef UAC_PSTV_STREAM_H
#define UAC_PSTV_STREAM_H

#include <stdint.h>

typedef void (*UacCoreFillCallback)(int16_t *output);
typedef void (*UacCoreStateCallback)(int running);

/* Bridge from the early USB module to the late audio/mixer module. Passing
 * NULL for both callbacks drains any in-flight call before returning. */
int uac_core_set_audio_callbacks(UacCoreFillCallback fill,
	UacCoreStateCallback state);

int uac_stream_start(int pipe_id, uint16_t packet_bytes, uint8_t speed,
	uint8_t interval);
void uac_stream_stop(void);

#endif
