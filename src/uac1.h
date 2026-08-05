#ifndef UAC_PSTV_UAC1_H
#define UAC_PSTV_UAC1_H

#include <psp2kern/types.h>

int uac1_probe(int device_id);
int uac1_attach(int device_id);
int uac1_detach(int device_id);
int uac_core_is_attached(void);

#endif
