#ifndef UAC_PSTV_AUDIO_TAP_H
#define UAC_PSTV_AUDIO_TAP_H

/*
 * Take the AVConfig route, and block until it has converged.  Call from the
 * session thread, after the transport is running: acquisition takes roughly
 * 400 ms, and the feeder covers that window with silence.
 */
int audio_tap_begin(void);

/* End one USB session. Resolution and private hooks exist only between these. */
int audio_tap_end(void);

/* Module shutdown: retry cleanup after an interrupted USB session. */
int audio_tap_shutdown(void);

#endif
