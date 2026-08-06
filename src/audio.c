#include "audio.h"
#include "log.h"
#include "mixer.h"

#include <psp2/audioout.h>
#include <psp2kern/kernel/threadmgr/misc.h>
#include <string.h>
#include <taihen.h>

#define LOG_PREFIX "[uac-pstv-audio] "
#define SCE_AUDIO_OUTPUT_NID 0x02DB3F5Fu
#define SCE_AUDIO_OPEN_PORT_NID 0x5BC341E4u
#define SCE_AUDIO_RELEASE_PORT_NID 0x69E2E6B5u
#define SCE_AUDIO_SET_CONFIG_NID 0xB8BA0D07u
#define SCE_AUDIO_GET_CONFIG_NID 0x9C8EDAEAu

static tai_hook_ref_t output_ref;
static tai_hook_ref_t open_ref;
static tai_hook_ref_t release_ref;
static tai_hook_ref_t config_ref;
static tai_hook_ref_t get_config_ref;
static SceUID output_hook = -1;
static SceUID open_hook = -1;
static SceUID release_hook = -1;
static SceUID config_hook = -1;
static SceUID get_config_hook = -1;

typedef struct {
	SceUID pid;
	int port;
	uint32_t frames;
	uint32_t rate;
	int mode;
	uint32_t generation;
	uint32_t sequence;
} __attribute__((aligned(32))) AudioPort;

static AudioPort ports[UAC_AUDIO_SOURCE_SLOTS];
static int ports_lock;
static uint32_t port_generation;

/* Non-blocking: these hooks run in the calling app's thread and must not
 * sleep. Acquire/release rather than __sync_*, which emits a full barrier. */
static int try_lock_ports(void)
{
	return __atomic_exchange_n(&ports_lock, 1, __ATOMIC_ACQUIRE) == 0;
}

static void unlock_ports(void)
{
	__atomic_store_n(&ports_lock, 0, __ATOMIC_RELEASE);
}

/* Odd means the slot is being changed; even means it is a stable snapshot.
 * Writers are serialized by ports_lock. */
static void begin_port_write(AudioPort *entry)
{
	__atomic_fetch_add(&entry->sequence, 1, __ATOMIC_ACQ_REL);
}

static void end_port_write(AudioPort *entry)
{
	__atomic_fetch_add(&entry->sequence, 1, __ATOMIC_RELEASE);
}

static int find_port_locked(SceUID pid, int port)
{
	int index;
	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index)
		if (ports[index].frames != 0 && ports[index].pid == pid &&
			ports[index].port == port)
			return index;
	return -1;
}

static int find_port_snapshot(SceUID pid, int port, uint32_t *frames,
	uint32_t *generation)
{
	int index;
	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index) {
		uint32_t sequence = __atomic_load_n(&ports[index].sequence,
			__ATOMIC_ACQUIRE);
		uint32_t count;
		uint32_t lifetime;
		SceUID owner;
		int number;

		if (sequence & 1u)
			continue;
		count = __atomic_load_n(&ports[index].frames, __ATOMIC_RELAXED);
		if (count == 0)
			continue;
		owner = __atomic_load_n(&ports[index].pid, __ATOMIC_RELAXED);
		if (owner != pid)
			continue;
		number = __atomic_load_n(&ports[index].port, __ATOMIC_RELAXED);
		if (number != port)
			continue;
		lifetime = __atomic_load_n(&ports[index].generation,
			__ATOMIC_RELAXED);
		__atomic_thread_fence(__ATOMIC_ACQUIRE);
		if (sequence == __atomic_load_n(&ports[index].sequence,
				__ATOMIC_RELAXED)) {
			*frames = count;
			*generation = lifetime;
			return index;
		}
	}
	return -1;
}

static int call_audio_open(int type, int len, int freq, int mode)
{
	struct _tai_hook_user *cur = (struct _tai_hook_user *)open_ref;
	struct _tai_hook_user *next = (struct _tai_hook_user *)cur->next;
	int (*function)(int, int, int, int) = (int (*)(int, int, int, int))
		(next == NULL ? cur->old : next->func);
	return function(type, len, freq, mode);
}

static int call_audio_output(int port, const void *buffer)
{
	struct _tai_hook_user *cur = (struct _tai_hook_user *)output_ref;
	struct _tai_hook_user *next = (struct _tai_hook_user *)cur->next;
	int (*function)(int, const void *) = (int (*)(int, const void *))
		(next == NULL ? cur->old : next->func);
	return function(port, buffer);
}

static int call_audio_release(int port)
{
	struct _tai_hook_user *cur = (struct _tai_hook_user *)release_ref;
	struct _tai_hook_user *next = (struct _tai_hook_user *)cur->next;
	int (*function)(int) = (int (*)(int))(next == NULL ? cur->old : next->func);
	return function(port);
}

static int call_audio_set_config(int port, SceSize frames, int rate, int mode)
{
	struct _tai_hook_user *cur = (struct _tai_hook_user *)config_ref;
	struct _tai_hook_user *next = (struct _tai_hook_user *)cur->next;
	int (*function)(int, SceSize, int, int) =
		(int (*)(int, SceSize, int, int))
		(next == NULL ? cur->old : next->func);
	return function(port, frames, rate, mode);
}

static int call_audio_get_config(int port, int type)
{
	struct _tai_hook_user *cur = (struct _tai_hook_user *)get_config_ref;
	struct _tai_hook_user *next = (struct _tai_hook_user *)cur->next;
	int (*function)(int, int) = (int (*)(int, int))
		(next == NULL ? cur->old : next->func);
	return function(port, type);
}

static int register_port(SceUID pid, int port, uint32_t frames,
	uint32_t rate, int mode, uint32_t *generation)
{
	int slot = -1;
	int oldest = 0;
	int index;
	int chosen;
	uint32_t lifetime;

	if (frames < SCE_AUDIO_MIN_LEN || frames > SCE_AUDIO_MAX_LEN ||
		(mode != SCE_AUDIO_OUT_MODE_MONO &&
		 mode != SCE_AUDIO_OUT_MODE_STEREO))
		return -1;
	if (!try_lock_ports())
		return -1;
	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index) {
		if (ports[index].frames != 0 && ports[index].pid == pid &&
			ports[index].port == port) {
			slot = index;
			break;
		}
		if (ports[index].frames == 0 && slot < 0)
			slot = index;
		if (ports[index].generation < ports[oldest].generation)
			oldest = index;
	}
	if (slot < 0)
		slot = oldest;
	chosen = slot;
	begin_port_write(&ports[slot]);
	__atomic_store_n(&ports[slot].frames, 0, __ATOMIC_RELAXED);
	__atomic_store_n(&ports[slot].pid, pid, __ATOMIC_RELAXED);
	__atomic_store_n(&ports[slot].port, port, __ATOMIC_RELAXED);
	ports[slot].rate = rate;
	ports[slot].mode = mode;
	lifetime = ++port_generation;
	__atomic_store_n(&ports[slot].generation, lifetime,
		__ATOMIC_RELAXED);
	if (uac_mixer_source_open(slot, rate, mode, lifetime) == 0) {
		__atomic_store_n(&ports[slot].frames, frames, __ATOMIC_RELAXED);
		if (generation != NULL)
			*generation = lifetime;
	} else {
		slot = -1;
	}
	end_port_write(&ports[chosen]);
	unlock_ports();
	return slot;
}

static int audio_open_hook(int type, int frames, int rate, int mode)
{
	int port = call_audio_open(type, frames, rate, mode);

	if (port >= 0) {
		SceUID pid = ksceKernelGetProcessId();
		int slot = register_port(pid, port, (uint32_t)frames,
			(uint32_t)rate, mode, NULL);

		if (slot >= 0)
			uac_log(LOG_PREFIX
				"audio open pid 0x%08x port %d type %d, %d frames, %d Hz, mode %d, source %d\n",
				pid, port, type, frames, rate, mode, slot);
	}
	return port;
}

static int audio_output_hook(int port, const void *buffer)
{
	SceUID pid;
	uint32_t frames = 0;
	uint32_t generation = 0;
	int slot = -1;

	if (buffer == NULL)
		return call_audio_output(port, buffer);
	pid = ksceKernelGetProcessId();
	slot = find_port_snapshot(pid, port, &frames, &generation);
	if (slot < 0 && get_config_hook >= 0) {
		int recovered_frames = call_audio_get_config(port,
			SCE_AUDIO_OUT_CONFIG_TYPE_LEN);
		int recovered_rate = call_audio_get_config(port,
			SCE_AUDIO_OUT_CONFIG_TYPE_FREQ);
		int recovered_mode = call_audio_get_config(port,
			SCE_AUDIO_OUT_CONFIG_TYPE_MODE);

		if (recovered_frames > 0 && recovered_rate > 0 &&
			recovered_mode >= 0) {
			frames = (uint32_t)recovered_frames;
			slot = register_port(pid, port, frames,
				(uint32_t)recovered_rate, recovered_mode,
				&generation);
			if (slot >= 0)
				uac_log(LOG_PREFIX
					"audio recovered pid 0x%08x port %d, %u frames, %d Hz, mode %d, source %d\n",
					pid, port, frames, recovered_rate,
					recovered_mode, slot);
		}
	}
	if (slot >= 0)
		uac_mixer_source_push(slot, pid, buffer, frames, generation);
	return call_audio_output(port, buffer);
}

static int audio_set_config_hook(int port, SceSize frames, int rate, int mode)
{
	int result = call_audio_set_config(port, frames, rate, mode);
	SceUID pid;
	uint32_t new_frames = 0;
	uint32_t new_rate = 0;
	uint32_t lifetime = 0;
	int new_mode = -1;
	int reopen = 0;
	int configured = 1;
	int slot;

	if (result < 0)
		return result;
	pid = ksceKernelGetProcessId();
	if (!try_lock_ports())
		return result;
	slot = find_port_locked(pid, port);
	if (slot >= 0) {
		begin_port_write(&ports[slot]);
		if (frames != (SceSize)-1)
			new_frames = (uint32_t)frames;
		else
			new_frames = ports[slot].frames;
		if (rate != -1 && (uint32_t)rate != ports[slot].rate) {
			ports[slot].rate = (uint32_t)rate;
			reopen = 1;
		}
		if (mode != -1 && mode != ports[slot].mode) {
			ports[slot].mode = mode;
			reopen = 1;
		}
		new_rate = ports[slot].rate;
		new_mode = ports[slot].mode;
		__atomic_store_n(&ports[slot].frames, 0, __ATOMIC_RELAXED);
		if (reopen) {
			lifetime = ++port_generation;
			__atomic_store_n(&ports[slot].generation, lifetime,
				__ATOMIC_RELAXED);
			configured = uac_mixer_source_open(slot, new_rate,
				new_mode, lifetime) == 0;
		}
		if (configured)
			__atomic_store_n(&ports[slot].frames, new_frames,
				__ATOMIC_RELAXED);
		end_port_write(&ports[slot]);
	}
	unlock_ports();
	if (slot >= 0 && reopen && configured) {
		/* Only when the format actually changed: reopening drains the
		 * source, and a buffer-length-only setConfig must not do that. */
		uac_log(LOG_PREFIX
			"audio config pid 0x%08x port %d, %u frames, %u Hz, mode %d\n",
			pid, port, new_frames, new_rate, new_mode);
	}
	return result;
}

static int audio_release_hook(int port)
{
	SceUID pid = ksceKernelGetProcessId();
	int result = call_audio_release(port);
	int slot;

	if (!try_lock_ports())
		return result;
	slot = find_port_locked(pid, port);
	if (slot >= 0) {
		begin_port_write(&ports[slot]);
		__atomic_store_n(&ports[slot].frames, 0, __ATOMIC_RELAXED);
		uac_mixer_source_close(slot);
		__atomic_store_n(&ports[slot].pid, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&ports[slot].port, 0, __ATOMIC_RELAXED);
		ports[slot].rate = 0;
		ports[slot].mode = 0;
		ports[slot].generation = 0;
		end_port_write(&ports[slot]);
	}
	unlock_ports();
	return result;
}

static int audio_get_config_hook(int port, int type)
{
	return call_audio_get_config(port, type);
}

int uac_audio_init(void)
{
	memset(ports, 0, sizeof(ports));
	open_hook = taiHookFunctionExportForKernel(KERNEL_PID, &open_ref,
		"SceAudio", TAI_ANY_LIBRARY, SCE_AUDIO_OPEN_PORT_NID,
		audio_open_hook);
	output_hook = taiHookFunctionExportForKernel(KERNEL_PID, &output_ref,
		"SceAudio", TAI_ANY_LIBRARY, SCE_AUDIO_OUTPUT_NID,
		audio_output_hook);
	release_hook = taiHookFunctionExportForKernel(KERNEL_PID, &release_ref,
		"SceAudio", TAI_ANY_LIBRARY, SCE_AUDIO_RELEASE_PORT_NID,
		audio_release_hook);
	config_hook = taiHookFunctionExportForKernel(KERNEL_PID, &config_ref,
		"SceAudio", TAI_ANY_LIBRARY, SCE_AUDIO_SET_CONFIG_NID,
		audio_set_config_hook);
	get_config_hook = taiHookFunctionExportForKernel(KERNEL_PID,
		&get_config_ref, "SceAudio", TAI_ANY_LIBRARY,
		SCE_AUDIO_GET_CONFIG_NID, audio_get_config_hook);
	uac_log(LOG_PREFIX
		"audio hooks open 0x%08x output 0x%08x release 0x%08x config 0x%08x get 0x%08x\n",
		open_hook, output_hook, release_hook, config_hook, get_config_hook);
	return output_hook < 0 ? output_hook : 0;
}

void uac_audio_fini(void)
{
	int index;
	if (config_hook >= 0)
		taiHookReleaseForKernel(config_hook, config_ref);
	if (release_hook >= 0)
		taiHookReleaseForKernel(release_hook, release_ref);
	if (output_hook >= 0)
		taiHookReleaseForKernel(output_hook, output_ref);
	if (get_config_hook >= 0)
		taiHookReleaseForKernel(get_config_hook, get_config_ref);
	if (open_hook >= 0)
		taiHookReleaseForKernel(open_hook, open_ref);
	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index)
		uac_mixer_source_close(index);
	config_hook = -1;
	release_hook = -1;
	output_hook = -1;
	get_config_hook = -1;
	open_hook = -1;
}
