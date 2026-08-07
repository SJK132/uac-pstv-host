#ifndef UAC_PSTV_MIXER_H
#define UAC_PSTV_MIXER_H

#include <stdint.h>

#define UAC_AUDIO_SOURCE_SLOTS 8

int uac_mixer_source_open(int source, uint32_t rate, int mode,
	uint32_t generation);
void uac_mixer_source_close(int source);
void uac_mixer_source_push(int source, int pid, const void *buffer,
	uint32_t frames, uint32_t generation);

void uac_mixer_start(void);
void uac_mixer_stop(void);
void uac_mixer_fill(int16_t *output);

#ifdef UAC_MIXER_TEST
extern void (*uac_mixer_test_before_commit)(void);
#endif

#endif
