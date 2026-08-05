#include "log.h"

#include <psp2kern/io/fcntl.h>
#include <psp2kern/kernel/debug.h>
#include <psp2kern/kernel/sysclib.h>
#include <psp2kern/types.h>
#include <stdarg.h>

#define LOG_FILE "ur0:data/uac_pstv.log"
#define LOG_LINE_MAX 256

void uac_log(const char *format, ...)
{
	char line[LOG_LINE_MAX];
	va_list args;
	int length;
	SceUID fd;

	va_start(args, format);
	length = vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	if (length < 0)
		return;
	if (length >= (int)sizeof(line))
		length = sizeof(line) - 1;

	/* Keep the existing debug stream for PLog/ShipLog users. */
	ksceDebugPrintf("%s", line);

	fd = ksceIoOpen(LOG_FILE, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_APPEND, 0666);
	if (fd >= 0) {
		ksceIoWrite(fd, line, (SceSize)length);
		ksceIoClose(fd);
	}
}
