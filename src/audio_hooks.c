#include "audio_hooks.h"
#include "log.h"
#include "mixer.h"
#include <psp2/audioout.h>
#include <psp2kern/kernel/threadmgr/misc.h>
#include <string.h>
#include <taihen.h>

#define LOG_PREFIX "[uac-pstv-audio] "
#define AUDIO_OUTPUT_NID 0x02DB3F5Fu
#define AUDIO_OPEN_NID 0x5BC341E4u
#define AUDIO_RELEASE_NID 0x69E2E6B5u
#define AUDIO_CONFIG_NID 0xB8BA0D07u
#define AUDIO_GET_CONFIG_NID 0x9C8EDAEAu
#define PORT_MISSING (-1)

typedef struct {
	SceUID pid;
	int port;
	uint32_t frames;
	uint32_t rate;
	uint32_t generation;
	int mode;
} AudioPort;

typedef enum {
	HOOK_OPEN,
	HOOK_RELEASE,
	HOOK_CONFIG,
	HOOK_GET_CONFIG,
	HOOK_OUTPUT,
	HOOK_COUNT
} HookId;

typedef struct {
	tai_hook_ref_t ref;
	SceUID uid;
	uint32_t nid;
	const void *function;
} AudioHook;

typedef int (*OpenFn)(int, int, int, int);
typedef int (*OutputFn)(int, const void *);
typedef int (*PortFn)(int);
typedef int (*ConfigFn)(int, SceSize, int, int);
typedef int (*GetConfigFn)(int, int);

static int open_hook(int type, int frames, int rate, int mode);
static int output_hook(int port, const void *buffer);
static int release_hook(int port);
static int config_hook(int port, SceSize frames, int rate, int mode);
static int get_config_hook(int port, int type);
static AudioHook hooks[HOOK_COUNT] = {
	[HOOK_OPEN] = {0, -1, AUDIO_OPEN_NID, (const void *)open_hook},
	[HOOK_RELEASE] = {0, -1, AUDIO_RELEASE_NID, (const void *)release_hook},
	[HOOK_CONFIG] = {0, -1, AUDIO_CONFIG_NID, (const void *)config_hook},
	[HOOK_GET_CONFIG] = {0, -1, AUDIO_GET_CONFIG_NID, (const void *)get_config_hook},
	[HOOK_OUTPUT] = {0, -1, AUDIO_OUTPUT_NID, (const void *)output_hook},
};

static AudioPort ports[UAC_AUDIO_SOURCE_SLOTS];
static int ports_lock;
static int capture_enabled;
static uint32_t generation_counter;

static int enabled(void)
{
	return __atomic_load_n(&capture_enabled, __ATOMIC_ACQUIRE);
}
/* Output skips contention; lifecycle hooks serialize so updates are never lost. */
static int try_lock(void)
{
	return __atomic_exchange_n(&ports_lock, 1, __ATOMIC_ACQUIRE) == 0;
}
static void lock(void)
{
	while (!try_lock())
		;
}
static void unlock(void)
{
	__atomic_store_n(&ports_lock, 0, __ATOMIC_RELEASE);
}
static void *hook_target(HookId id)
{
	struct _tai_hook_user *current = (struct _tai_hook_user *)(uintptr_t)hooks[id].ref;
	struct _tai_hook_user *next = (struct _tai_hook_user *)(uintptr_t)current->next;
	return next != NULL ? next->func : current->old;
}
static uint32_t next_generation(void)
{
	do {
		generation_counter++;
	} while (generation_counter == 0);
	return generation_counter;
}
static void clear_port(int slot)
{
	memset(&ports[slot], 0, sizeof(ports[slot]));
}
static int find_port(SceUID pid, int port)
{
	int i;
	for (i = 0; i < UAC_AUDIO_SOURCE_SLOTS; ++i) {
		if (ports[i].frames && ports[i].pid == pid && ports[i].port == port)
			return i;
	}
	return PORT_MISSING;
}
static int choose_slot(SceUID pid, int port)
{
	int free_slot = PORT_MISSING;
	int oldest = 0;
	int i;
	for (i = 0; i < UAC_AUDIO_SOURCE_SLOTS; ++i) {
		if (ports[i].frames && ports[i].pid == pid && ports[i].port == port)
			return i;
		if (!ports[i].frames && free_slot < 0)
			free_slot = i;
		if (ports[i].generation < ports[oldest].generation)
			oldest = i;
	}
	return free_slot >= 0 ? free_slot : oldest;
}
/* Mixer generations reject old pushes after the registry lock is released. */
static int register_locked(SceUID pid, int port, uint32_t frames, uint32_t rate,
	int mode, uint32_t *generation)
{
	uint32_t lifetime;
	int slot;
	if (!frames)
		return PORT_MISSING;
	slot = choose_slot(pid, port);
	lifetime = next_generation();
	if (uac_mixer_source_open(slot, rate, mode, lifetime) < 0) {
		clear_port(slot);
		return PORT_MISSING;
	}
	ports[slot] = (AudioPort){pid, port, frames, rate, lifetime, mode};
	if (generation != NULL)
		*generation = lifetime;
	return slot;
}
static int recover_locked(SceUID pid, int port, GetConfigFn get_config,
	uint32_t *frames, uint32_t *generation)
{
	int recovered_frames = get_config(port, SCE_AUDIO_OUT_CONFIG_TYPE_LEN);
	int rate = get_config(port, SCE_AUDIO_OUT_CONFIG_TYPE_FREQ);
	int mode = get_config(port, SCE_AUDIO_OUT_CONFIG_TYPE_MODE);
	int slot;
	if (recovered_frames <= 0 || rate <= 0 || mode < 0)
		return PORT_MISSING;
	*frames = (uint32_t)recovered_frames;
	slot = register_locked(pid, port, *frames, (uint32_t)rate, mode, generation);
	if (slot >= 0)
		uac_log(LOG_PREFIX
			"recovered pid 0x%08x port %d, %u frames, %d Hz, mode %d, source %d\n",
			pid, port, *frames, rate, mode, slot);
	return slot;
}
static int snapshot_port(SceUID pid, int port, GetConfigFn get_config,
	uint32_t *frames, uint32_t *generation)
{
	int slot;
	if (!try_lock())
		return PORT_MISSING;
	if (!enabled()) {
		unlock();
		return PORT_MISSING;
	}
	slot = find_port(pid, port);
	if (slot < 0)
		slot = recover_locked(pid, port, get_config, frames, generation);
	else {
		*frames = ports[slot].frames;
		*generation = ports[slot].generation;
	}
	unlock();
	return slot;
}

static int open_hook(int type, int frames, int rate, int mode)
{
	OpenFn original = (OpenFn)hook_target(HOOK_OPEN);
	SceUID pid;
	int slot = PORT_MISSING;
	int port = original(type, frames, rate, mode);
	if (port >= 0 && enabled()) {
		pid = ksceKernelGetProcessId();
		lock();
		if (enabled())
			slot = register_locked(pid, port, (uint32_t)frames, (uint32_t)rate,
				mode, NULL);
		unlock();
		if (slot >= 0)
			uac_log(LOG_PREFIX
				"open pid 0x%08x port %d type %d, %d frames, %d Hz, mode %d, source %d\n",
				pid, port, type, frames, rate, mode, slot);
	}
	return port;
}
static int output_hook(int port, const void *buffer)
{
	OutputFn original = (OutputFn)hook_target(HOOK_OUTPUT);
	SceUID pid;
	uint32_t frames = 0;
	uint32_t generation = 0;
	int slot;
	if (buffer != NULL && enabled()) {
		GetConfigFn get_config = (GetConfigFn)hook_target(HOOK_GET_CONFIG);
		pid = ksceKernelGetProcessId();
		slot = snapshot_port(pid, port, get_config, &frames, &generation);
		if (slot >= 0)
			uac_mixer_source_push(slot, pid, buffer, frames, generation);
	}
	return original(port, buffer);
}
static int config_hook(int port, SceSize frames, int rate, int mode)
{
	ConfigFn original = (ConfigFn)hook_target(HOOK_CONFIG);
	uint32_t new_frames;
	uint32_t new_rate;
	uint32_t lifetime;
	SceUID pid;
	int new_mode;
	int result;
	int slot;
	if (!enabled())
		return original(port, frames, rate, mode);
	/* Block capture across Sony's change so no buffer sees stale metadata. */
	lock();
	result = original(port, frames, rate, mode);
	if (result < 0 || !enabled())
		goto out;
	pid = ksceKernelGetProcessId();
	slot = find_port(pid, port);
	if (slot < 0)
		goto out;
	new_frames = frames == (SceSize)-1 ? ports[slot].frames : (uint32_t)frames;
	new_rate = rate == -1 ? ports[slot].rate : (uint32_t)rate;
	new_mode = mode == -1 ? ports[slot].mode : mode;
	if (new_rate == ports[slot].rate && new_mode == ports[slot].mode) {
		ports[slot].frames = new_frames;
		goto out;
	}
	lifetime = next_generation();
	if (uac_mixer_source_open(slot, new_rate, new_mode, lifetime) < 0) {
		clear_port(slot); /* Next output retries Sony's current config. */
		goto out;
	}
	ports[slot] = (AudioPort){pid, port, new_frames, new_rate, lifetime, new_mode};
	uac_log(LOG_PREFIX "config pid 0x%08x port %d, %u frames, %u Hz, mode %d\n",
		pid, port, new_frames, new_rate, new_mode);
out:
	unlock();
	return result;
}
static int release_hook(int port)
{
	PortFn original = (PortFn)hook_target(HOOK_RELEASE);
	int result;
	int slot;
	if (!enabled())
		return original(port);
	/* Keep output away until Sony and the source registry agree. */
	lock();
	result = original(port);
	if (result >= 0 && enabled()) {
		slot = find_port(ksceKernelGetProcessId(), port);
		if (slot >= 0) {
			uac_mixer_source_close(slot);
			clear_port(slot);
		}
	}
	unlock();
	return result;
}
static int get_config_hook(int port, int type)
{
	GetConfigFn original = (GetConfigFn)hook_target(HOOK_GET_CONFIG);
	return original(port, type);
}

static int release_hooks(void)
{
	int first_error = 0;
	int i;
	for (i = HOOK_COUNT - 1; i >= 0; --i) {
		int result;
		if (hooks[i].uid < 0)
			continue;
		result = taiHookReleaseForKernel(hooks[i].uid, hooks[i].ref);
		if (result < 0) {
			if (!first_error)
				first_error = result;
			uac_log(LOG_PREFIX "hook %d release failed: 0x%08x\n", i, result);
		} else {
			hooks[i].uid = -1;
		}
	}
	return first_error;
}

int uac_audio_hooks_init(void)
{
	int i;
	for (i = 0; i < HOOK_COUNT; ++i)
		if (hooks[i].uid >= 0)
			return -1;
	memset(ports, 0, sizeof(ports));
	ports_lock = 0;
	generation_counter = 0;
	__atomic_store_n(&capture_enabled, 0, __ATOMIC_RELEASE);
	/* Output is last, after lifecycle and recovery continuations exist. */
	for (i = 0; i < HOOK_COUNT; ++i) {
		hooks[i].uid = taiHookFunctionExportForKernel(KERNEL_PID, &hooks[i].ref,
			"SceAudio", TAI_ANY_LIBRARY, hooks[i].nid, hooks[i].function);
		if (hooks[i].uid < 0) {
			int error = hooks[i].uid;
			int rollback;
			uac_log(LOG_PREFIX "hook %d install failed: 0x%08x\n", i, error);
			rollback = release_hooks();
			if (rollback < 0) {
				uac_log(LOG_PREFIX "hook rollback incomplete; capture disabled\n");
				return 0; /* Keep the module resident while a hook remains. */
			}
			return error;
		}
	}
	__atomic_store_n(&capture_enabled, 1, __ATOMIC_RELEASE);
	uac_log(LOG_PREFIX "audio hooks installed\n");
	return 0;
}

int uac_audio_hooks_fini(void)
{
	int result;
	int i;
	__atomic_store_n(&capture_enabled, 0, __ATOMIC_RELEASE);
	lock();
	for (i = 0; i < UAC_AUDIO_SOURCE_SLOTS; ++i) {
		uac_mixer_source_close(i);
		clear_port(i);
	}
	unlock();
	result = release_hooks();
	return result;
}
