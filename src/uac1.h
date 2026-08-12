#ifndef UAC_PSTV_UAC1_H
#define UAC_PSTV_UAC1_H

/* Teardown worker lifetime. Init before registering the USB driver; shut down
 * only after unregistering it, so no callback can queue new work. */
int uac1_lifecycle_init(void);
int uac1_lifecycle_shutdown(void);

/* Retire the live session on the teardown worker. Never blocks. */
void uac1_suspend_session(void);
void uac1_resume_retry(void);

int uac1_probe(int device_id);
int uac1_attach(int device_id);
int uac1_detach(int device_id);

#endif
