#include "audio.h"
#include "log.h"
#include "mixer.h"
#include "stream.h"
#include "uac1.h"

#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/usbserv.h>

#define LOG_PREFIX "[uac-pstv-audio] "

static int started;

static void mixer_state(int running)
{
	/* The core only raises running after a supported UAC1 device attaches.
	 * Keep the attachment check here as a second boundary: disconnected
	 * systems never activate the mixer. */
	if (running && uac_core_is_attached())
		uac_mixer_start();
	else
		uac_mixer_stop();
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

	/* The boot_config core owns UAC transport. This late taiHEN helper only
	 * installs audio hooks and supplies PCM while that transport is active. */
	result = ksceUsbServMacSelect(2, 0);
	uac_log(LOG_PREFIX "host select: 0x%08x, attached %d\n", result,
		uac_core_is_attached());
	if (result < 0)
		return SCE_KERNEL_START_FAILED;

	result = uac_audio_init();
	if (result < 0) {
		uac_audio_fini();
		return SCE_KERNEL_START_FAILED;
	}
	result = uac_core_set_audio_callbacks(uac_mixer_fill, mixer_state);
	uac_log(LOG_PREFIX "audio bridge register: 0x%08x\n", result);
	if (result < 0) {
		uac_audio_fini();
		return SCE_KERNEL_START_FAILED;
	}

	if (!uac_core_is_attached())
		uac_log(LOG_PREFIX
			"no UAC1 device; mixer and stream remain stopped\n");

	started = 1;
	result = ksceKernelRegisterSysEventHandler("uac_pstv_audio_sysevent",
		uac_sysevent_handler, NULL);
	uac_log(LOG_PREFIX "sysevent register: 0x%08x\n", result);

	return SCE_KERNEL_START_SUCCESS;
}

int module_stop(SceSize args, const void *argp)
{
	int result;
	(void)args;
	(void)argp;

	started = 0;
	result = uac_core_set_audio_callbacks(NULL, NULL);
	uac_log(LOG_PREFIX "audio bridge unregister: 0x%08x\n", result);
	if (result < 0)
		return SCE_KERNEL_STOP_FAIL;
	uac_audio_fini();
	return SCE_KERNEL_STOP_SUCCESS;
}

int _start(SceSize args, const void *argp)
	__attribute__((weak, alias("module_start")));
