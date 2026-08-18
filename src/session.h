#ifndef UAC_PSTV_SESSION_H
#define UAC_PSTV_SESSION_H

/*
 * One thread owns a USB audio session for its whole life.
 *
 * USBD callbacks and the system-event handler post a command here and return,
 * so nothing that can block runs on a callback thread, and no step of setup or
 * teardown has to prove it is still the session it started on.
 */

/* Session-thread failure, distinct from the negative SCE error codes. */
#define SESSION_ERROR_CONTROL_TIMEOUT -0x7f000001

/* Init before USB registration; shutdown only after the driver is unregistered. */
int session_init(void);
int session_shutdown(void);

/* Refuse new sessions and retire the live one; accept rolls a failed stop back. */
void session_quiesce(void);
void session_accept(void);

/* Posted from USBD callbacks and the system-event handler. These never block. */
int session_start(int device_id);
void session_stop(const char *reason);
void session_stop_detached(int device_id);

/* Device id the live session owns, or -1. */
int session_device(void);

#endif
