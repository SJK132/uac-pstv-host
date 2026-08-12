#ifndef UAC_PSTV_AUDIO_TAP_H
#define UAC_PSTV_AUDIO_TAP_H

/* One USB session. Resolution and private hooks exist only between these. */
int audio_tap_begin(void);
int audio_tap_end(void);

/* Module shutdown: retry cleanup after an interrupted USB session. */
int audio_tap_shutdown(void);

#endif
