#ifndef UAC_PSTV_LOG_H
#define UAC_PSTV_LOG_H

#ifdef UAC_PSTV_ENABLE_LOGGING
void uac_log_open(void);
void uac_log_close(void);
void uac_log(const char *format, ...);
#else
#define uac_log_open() ((void)0)
#define uac_log_close() ((void)0)
#define uac_log(...) ((void)0)
#endif

#endif
