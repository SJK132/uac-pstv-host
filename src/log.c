#include "log.h"

#ifdef UAC_PSTV_ENABLE_LOGGING

#include <stdarg.h>

#include <psp2kern/io/fcntl.h>
#include <psp2kern/kernel/debug.h>
#include <psp2kern/kernel/sysclib.h>

#define UAC_LOG_PATH "ur0:data/uac_pstv.log"
#define UAC_LOG_BUFFER_SIZE 512u

/*
 * The file is opened once at module start and held until module stop.
 *
 * Callers include USBD completion callbacks and the system-event handler, which
 * have no business touching the filesystem at all; one append per line is the
 * cheapest thing that still works from those contexts.  Resist going back to
 * open/write/close per line -- three syscalls per line across four threads
 * drops lines during a sysevent burst, which is precisely when the log is worth
 * having.  Concurrent lines can still interleave; each one is atomic enough to
 * read.
 */
static SceUID log_fd = -1;

void uac_log_open(void)
{
	if (log_fd < 0)
		log_fd = ksceIoOpen(UAC_LOG_PATH,
			SCE_O_WRONLY | SCE_O_CREAT | SCE_O_APPEND, 0666);
}

void uac_log_close(void)
{
	if (log_fd >= 0) {
		(void)ksceIoClose(log_fd);
		log_fd = -1;
	}
}

void uac_log(const char *format, ...)
{
	char buffer[UAC_LOG_BUFFER_SIZE];
	va_list args;
	int length;

	va_start(args, format);
	length = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	if (length <= 0)
		return;
	if ((unsigned int)length >= sizeof(buffer))
		length = (int)sizeof(buffer) - 1;

	(void)ksceKernelPrintf("%s", buffer);
	if (log_fd >= 0)
		(void)ksceIoWrite(log_fd, buffer, (SceSize)length);
}

#endif
