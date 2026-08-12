/*
 * Module entry points and system-wide lifecycle.
 *
 * Start order is dependency order -- stream, then the uac1 teardown worker,
 * then the USB driver -- and stop reverses it, so nothing can ever be called
 * back into after it has been torn down.  audio_tap deliberately does not
 * appear in that sequence: it is owned by the feeder thread and lives exactly
 * as long as one USB session.
 */

#include "audio_tap.h"
#include "log.h"
#include "stream.h"
#include "uac1.h"

#include <stddef.h>
#include <psp2kern/kernel/aimgr.h>
#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/usbserv.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv] "

/* Present in the VitaSDK NID database and newer suspend.h revisions. */
int ksceKernelUnregisterSysEventHandler(SceUID handler_id);

static int started;
static int owns_usb_driver;
static SceUID sysevent_id = -1;

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

/*
 * Suspend arrives as one descending sweep of resume=0 events, observed on this
 * firmware as 0x020F down to 0x0200 (then an unrelated 0x0401).  Gate on the
 * head of that sweep: it is the earliest notice, so the teardown worker gets
 * the most time before the system goes under, and acting once avoids relying on
 * request_cleanup() being idempotent across sixteen more events.
 *
 * A firmware whose sweep starts elsewhere simply never matches, which degrades
 * to the previous behaviour -- standby handled by the detach that follows --
 * rather than misfiring on a live session.
 */
#define SYSEVENT_SUSPEND_HEAD 0x0000020Fu

static int uac_sysevent_handler(int resume, int eventid, void *args, void *opt)
{
	(void)args;
	(void)opt;
	if (!__atomic_load_n(&started, __ATOMIC_ACQUIRE))
		return 0;
	if (resume) {
		(void)ksceUsbServMacSelect(2, 0);
		/*
		 * Suspend may have cut a teardown short before the worker was
		 * scheduled.  Poking it is cheap and no-ops when idle.
		 */
		uac1_resume_retry();
	} else if ((uint32_t)eventid == SYSEVENT_SUSPEND_HEAD) {
		uac1_suspend_session();
	}
	return 0;
}

int module_start(SceSize args, const void *argp)
{
	int result;

	(void)args;
	(void)argp;
	uac_log_open();
	/* First line of every session: the log appends across boots, so this both
	 * identifies the build and marks where one run ends and the next begins. */
	uac_log(LOG_PREFIX "uac-pstv %s (%s)\n", UAC_PSTV_VERSION,
		UAC_PSTV_BUILD_REV);

	/*
	 * PSTV only.  "Dolce" is Sony's name for it.  A handheld Vita has no USB
	 * host port to claim, so bail out before touching USB rather than forcing
	 * a mode change on hardware this was never tested against.
	 */
	if (!ksceSblAimgrIsDolce()) {
		uac_log(LOG_PREFIX "not a PlayStation TV, not loading\n");
		uac_log_close();
		return SCE_KERNEL_START_NO_RESIDENT;
	}
	/*
	 * Mode 2 is USB host, which is what the PSTV port already runs in -- this
	 * asserts it rather than changing it.  The same call on resume is the one
	 * that matters, since the mode is not guaranteed to survive suspend.
	 */
	result = ksceUsbServMacSelect(2, 0);
	uac_log(LOG_PREFIX "USB host select: 0x%08x\n", result);
	if (result < 0) {
		uac_log_close();
		return SCE_KERNEL_START_FAILED;
	}

	result = uac_stream_init();
	if (result < 0)
		return SCE_KERNEL_START_FAILED;

	/* The teardown worker must exist before any callback can request it. */
	result = uac1_lifecycle_init();
	if (result < 0) {
		(void)uac_stream_shutdown();
		return SCE_KERNEL_START_FAILED;
	}

	result = register_usb_driver();
	if (result < 0) {
		(void)uac1_lifecycle_shutdown();
		(void)uac_stream_shutdown();
		return SCE_KERNEL_START_FAILED;
	}

	__atomic_store_n(&started, 1, __ATOMIC_RELEASE);
	result = ksceKernelRegisterSysEventHandler(
		"uac_pstv_sysevent", uac_sysevent_handler, NULL);
	uac_log(LOG_PREFIX "sysevent register: 0x%08x\n", result);
	if (result >= 0)
		sysevent_id = result;
	return SCE_KERNEL_START_SUCCESS;
}

int module_stop(SceSize args, const void *argp)
{
	int usb_result;
	int lifecycle_result;
	int stream_result;
	int tap_result;

	(void)args;
	(void)argp;
	__atomic_store_n(&started, 0, __ATOMIC_RELEASE);

	usb_result = unregister_usb_driver();
	if (usb_result < 0) {
		__atomic_store_n(&started, 1, __ATOMIC_RELEASE);
		return SCE_KERNEL_STOP_FAIL;
	}
	if (sysevent_id >= 0) {
		int result = ksceKernelUnregisterSysEventHandler(sysevent_id);

		if (result < 0)
			return SCE_KERNEL_STOP_FAIL;
		sysevent_id = -1;
	}
	/* Driver and sysevent are gone, so no new teardown can be queued. */
	lifecycle_result = uac1_lifecycle_shutdown();
	if (lifecycle_result < 0)
		return SCE_KERNEL_STOP_FAIL;
	stream_result = uac_stream_shutdown();
	if (stream_result < 0)
		return SCE_KERNEL_STOP_FAIL;
	tap_result = audio_tap_shutdown();
	if (tap_result < 0)
		return SCE_KERNEL_STOP_FAIL;
	uac_log_close();
	return SCE_KERNEL_STOP_SUCCESS;
}

int _start(SceSize args, const void *argp)
	__attribute__((weak, alias("module_start")));
