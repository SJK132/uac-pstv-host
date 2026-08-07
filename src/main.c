#include "audio_hooks.h"
#include "log.h"
#include "mixer.h"
#include "uac1.h"

#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/usbserv.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv] "

static int started;
static int owns_usb_driver;

static const SceUsbdDriver uac_driver = {
	.name = "uac_pstv",
	.probe = uac1_probe,
	.attach = uac1_attach,
	.detach = uac1_detach,
};

static int register_usb_driver(void)
{
	int result = ksceUsbdRegisterDriver(&uac_driver);

	uac_log(LOG_PREFIX "USB register: 0x%08x\n", result);
	if (result >= 0)
		owns_usb_driver = 1;
	return result;
}

static int unregister_usb_driver(void)
{
	int result;

	if (!owns_usb_driver)
		return 0;

	result = ksceUsbdUnregisterDriver(&uac_driver);
	uac_log(LOG_PREFIX "USB unregister: 0x%08x\n", result);
	if (result >= 0)
		owns_usb_driver = 0;
	return result;
}

static int uac_sysevent_handler(int resume, int eventid, void *args, void *opt)
{
	int result;

	(void)eventid;
	(void)args;
	(void)opt;

	if (resume && started) {
		result = ksceUsbServMacSelect(2, 0);
		(void)result;
		uac_log(LOG_PREFIX "resume host select: 0x%08x\n", result);
	}
	return 0;
}

int module_start(SceSize args, const void *argp)
{
	int result;

	(void)args;
	(void)argp;

	result = ksceUsbServMacSelect(2, 0);
	if (result < 0)
		return SCE_KERNEL_START_FAILED;

	if (register_usb_driver() < 0)
		return SCE_KERNEL_START_FAILED;

	result = uac_audio_hooks_init();
	if (result < 0) {
		(void)uac_audio_hooks_fini();
		(void)unregister_usb_driver();
		return SCE_KERNEL_START_FAILED;
	}

	started = 1;
	result = ksceKernelRegisterSysEventHandler(
		"uac_pstv_sysevent", uac_sysevent_handler, NULL);
	uac_log(LOG_PREFIX "sysevent register: 0x%08x\n", result);

	return SCE_KERNEL_START_SUCCESS;
}

int module_stop(SceSize args, const void *argp)
{
	int result;

	(void)args;
	(void)argp;

	started = 0;
	result = unregister_usb_driver();
	if (result < 0) {
		started = 1;
		return SCE_KERNEL_STOP_FAIL;
	}

	uac_mixer_stop();
	result = uac_audio_hooks_fini();
	if (result < 0)
		return SCE_KERNEL_STOP_FAIL;

	return SCE_KERNEL_STOP_SUCCESS;
}

int _start(SceSize args, const void *argp)
	__attribute__((weak, alias("module_start")));
