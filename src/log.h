#ifndef UAC_PSTV_LOG_H
#define UAC_PSTV_LOG_H

#ifdef UAC_PSTV_ENABLE_LOGGING
void uac_log(const char *format, ...);
#else
/* Variadic macro: arguments and format strings disappear before compilation. */
#define uac_log(...) ((void)0)
#endif

#endif
