/* Self-contained stub: mixer.c only uses ksceKernelCopyFromUserProc, which
 * the harness implements as a plain memcpy. */
#ifndef TEST_STUB_DATA_TRANSFERS_H
#define TEST_STUB_DATA_TRANSFERS_H
typedef int SceUID;
typedef unsigned int SceSize;
int ksceKernelCopyFromUserProc(SceUID pid, void *dst, const void *src, SceSize len);
#endif
