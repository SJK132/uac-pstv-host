#ifndef UAC_PSTV_UAC1_H
#define UAC_PSTV_UAC1_H

/* USB driver callbacks registered by main.c. */
int uac1_probe(int device_id);
int uac1_attach(int device_id);
int uac1_detach(int device_id);

#endif
