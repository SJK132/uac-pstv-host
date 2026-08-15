#ifndef UAC_PSTV_AUDIO_TAP_H
#define UAC_PSTV_AUDIO_TAP_H

/* Start route acquisition, then poll it without blocking the USB feeder. */
int audio_tap_begin(void);
/* 0: ready, 1: pending, negative: acquisition failed. */
int audio_tap_poll(void);

/* End one USB session. Resolution and private hooks exist only between these. */
int audio_tap_end(void);

/* Module shutdown: retry cleanup after an interrupted USB session. */
int audio_tap_shutdown(void);

#endif
