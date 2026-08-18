#ifndef UAC_PSTV_AUDIO_TAP_H
#define UAC_PSTV_AUDIO_TAP_H

/*
 * Take the AVConfig route, and block until it has converged.  Call from the
 * session thread, after the transport is running: acquisition takes roughly
 * 400 ms, and the feeder covers that window with silence.
 */
int audio_tap_begin(void);

/*
 * End one USB session; resolution and private hooks exist only between these.
 * Fail-closed and idempotent, so module_stop calls it again to retry a cleanup
 * an interrupted session left behind.
 */
int audio_tap_end(void);

#endif
