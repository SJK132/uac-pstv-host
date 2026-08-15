#ifndef UAC_PSTV_UAC1_H
#define UAC_PSTV_UAC1_H

/* Init before USB registration. Quiesce before unregister; accept rolls back a
 * failed unregister. Shutdown succeeds only after unregister and retirement. */
int uac1_lifecycle_init(void);
int uac1_lifecycle_shutdown(void);
void uac1_lifecycle_quiesce(void);
void uac1_lifecycle_accept(void);

/* Retire the live session on the teardown worker. Never blocks. */
void uac1_suspend_session(void);
void uac1_resume_retry(void);
void uac1_stream_failed(void);

int uac1_probe(int device_id);
int uac1_attach(int device_id);
int uac1_detach(int device_id);

#endif
