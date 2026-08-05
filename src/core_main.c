#include "log.h"
#include "uac1.h"

#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv-boot] "

static const SceUsbdDriver uac_driver = {
	.name = "uac_pstv_boot",
	.probe = uac1_probe,
	.attach = uac1_attach,
	.detach = uac1_detach,
};

int module_start(SceSize args, const void *argp)
{
	int result;
	(void)args;
	(void)argp;

	result = ksceUsbdRegisterDriver(&uac_driver);
	uac_log(LOG_PREFIX "early register: 0x%08x\n", result);
	return result < 0 ? SCE_KERNEL_START_FAILED : SCE_KERNEL_START_SUCCESS;
}

int module_stop(SceSize args, const void *argp)
{
	int result;
	(void)args;
	(void)argp;

	result = ksceUsbdUnregisterDriver(&uac_driver);
	uac_log(LOG_PREFIX "unregister: 0x%08x\n", result);
	return result < 0 ? SCE_KERNEL_STOP_FAIL : SCE_KERNEL_STOP_SUCCESS;
}

int _start(SceSize args, const void *argp)
	__attribute__((weak, alias("module_start")));
