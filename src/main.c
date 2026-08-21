/*
 * Module entry points and system-wide lifecycle.
 *
 * Start order is dependency order -- stream, then the session thread, then the
 * USB driver -- and stop reverses it, so nothing can ever be called back into
 * after it has been torn down.  audio_tap deliberately does not appear in that
 * sequence: it belongs to the transport and lives exactly as long as one USB
 * session.
 */

#include "audio_tap.h"
#include "log.h"
#include "session.h"
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
static int suspend_seen;
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

static int unregister_sysevent(void)
{
	int result;

	if (sysevent_id < 0)
		return 0;
	result = ksceKernelUnregisterSysEventHandler(sysevent_id);
	uac_log(LOG_PREFIX "sysevent unregister: 0x%08x\n", result);
	if (result >= 0)
		sysevent_id = -1;
	return result;
}

/* Observed PSTV suspend event; duplicate delivery is collapsed below. */
#define SYSEVENT_SUSPEND 0x0000020fu

/*
 * suspend_seen gates both halves, and the resume half needs it more.
 *
 * A registered handler is called for every system event, and the resume phase
 * of most of them has nothing to do with sleep -- on hardware they arrive in
 * the hundreds during ordinary play.  Acting on `resume` alone therefore
 * re-asserted USB host mode hundreds of times per session and, in logging
 * builds, buried the log under it.  Requiring a suspend we actually handled
 * first is both cheaper and more accurate than guessing which event id the
 * matching resume carries, and it collapses duplicate resume delivery for
 * free.
 */
static int uac_sysevent_handler(int resume, int eventid, void *args, void *opt)
{
	(void)args;
	(void)opt;
	if (!__atomic_load_n(&started, __ATOMIC_ACQUIRE))
		return 0;
	if (resume) {
		int result;

		if (!__atomic_exchange_n(&suspend_seen, 0, __ATOMIC_ACQ_REL))
			return 0;
		result = ksceUsbServMacSelect(2, 0);
		uac_log(LOG_PREFIX "USB host resume select: 0x%08x\n", result);
		(void)result;
		/*
		 * Nothing to re-drive: a teardown suspend interrupted is still the
		 * session thread's current command and resumes with it.
		 */
	} else if ((uint32_t)eventid == SYSEVENT_SUSPEND &&
		!__atomic_exchange_n(&suspend_seen, 1, __ATOMIC_ACQ_REL)) {
		session_stop("system suspend");
	}
	return 0;
}

/* Returning START_FAILED is safe only after every published callback is gone. */
static int fail_start(void)
{
	int safe = 1;

	session_quiesce();
	if (unregister_usb_driver() < 0) {
		session_accept();
		__atomic_store_n(&started, 1, __ATOMIC_RELEASE);
		return SCE_KERNEL_START_SUCCESS;
	}
	/* A registered sysevent is inert while started is clear, so later stages
	 * may still be retired safely and module_stop can retry its unregister. */
	if (unregister_sysevent() < 0)
		safe = 0;
	if (session_shutdown() < 0)
		safe = 0;
	if (safe && uac_stream_shutdown() < 0)
		safe = 0;
	if (safe && audio_tap_end() < 0)
		safe = 0;
	if (!safe) {
		uac_log(LOG_PREFIX "start rollback incomplete; staying resident\n");
		return SCE_KERNEL_START_SUCCESS;
	}
	uac_log_close();
	return SCE_KERNEL_START_FAILED;
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
	if (result < 0) {
		if (uac_stream_shutdown() < 0)
			return SCE_KERNEL_START_SUCCESS;
		uac_log_close();
		return SCE_KERNEL_START_FAILED;
	}

	/* The teardown worker must exist before any callback can request it. */
	result = session_init();
	if (result < 0) {
		session_quiesce();
		if (session_shutdown() < 0 ||
		    uac_stream_shutdown() < 0) {
			uac_log(LOG_PREFIX
				"lifecycle rollback incomplete; staying resident\n");
			return SCE_KERNEL_START_SUCCESS;
		}
		uac_log_close();
		return SCE_KERNEL_START_FAILED;
	}

	result = ksceKernelRegisterSysEventHandler(
		"uac_pstv_sysevent", uac_sysevent_handler, NULL);
	uac_log(LOG_PREFIX "sysevent register: 0x%08x\n", result);
	if (result < 0)
		return fail_start();
	sysevent_id = result;

	result = register_usb_driver();
	if (result < 0) {
		return fail_start();
	}

	__atomic_store_n(&started, 1, __ATOMIC_RELEASE);
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
	session_quiesce();

	usb_result = unregister_usb_driver();
	if (usb_result < 0) {
		session_accept();
		__atomic_store_n(&started, 1, __ATOMIC_RELEASE);
		return SCE_KERNEL_STOP_FAIL;
	}
	if (unregister_sysevent() < 0)
		return SCE_KERNEL_STOP_FAIL;
	/* Driver and sysevent are gone, so no new teardown can be queued. */
	lifecycle_result = session_shutdown();
	if (lifecycle_result < 0)
		return SCE_KERNEL_STOP_FAIL;
	stream_result = uac_stream_shutdown();
	if (stream_result < 0)
		return SCE_KERNEL_STOP_FAIL;
	tap_result = audio_tap_end();
	if (tap_result < 0)
		return SCE_KERNEL_STOP_FAIL;
	uac_log_close();
	return SCE_KERNEL_STOP_SUCCESS;
}

int _start(SceSize args, const void *argp)
	__attribute__((weak, alias("module_start")));
