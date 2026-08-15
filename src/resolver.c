/*
 * Locating Sony's private audio internals.
 *
 * SceAVConfig's route internals are private and profile-gated below.  SceAudio's
 * three RAM-output functions are real exports, so they are resolved by NID
 * instead of assuming their text offsets.
 *
 * AVConfig offsets are safe because its text is byte-identical across every
 * firmware in known_avconfig[] -- established by diffing decrypted modules.
 * An unlisted module NID is refused rather than resolved on faith.  SceAudio is
 * independent of that profile: its three functions are resolved as exports.
 */

#include "resolver.h"

#include "log.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <taihen.h>

#define LOG_PREFIX "[uac-pstv-resolver] "

/*
 * SceAudio is exported at this boundary.  Walking one export library is both
 * shorter-lived and safer than pinning private text addresses: a firmware may
 * move code while preserving these exported NIDs.
 */
#define AUDIO_LIBRARY_NID 0x15D711C1u
#define RAM_RATE_NID 0x134C96C1u
#define RAM_CHANNELS_NID 0xFC375BACu
#define RAM_SUBMIT_NID 0x81EB0AE5u

#define AVCONFIG_EXPORTS_OFFSET 0x4DFCu
#define ROUTE_DIRTY_OFFSET 0x0050u
#define TRANSPORT_READY_OFFSET 0x0068u
#define SELECTED_TARGET_OFFSET 0x00E0u
#define ROUTE_WORD_OFFSET 0x0248u
#define ROUTE_LOCK_OFFSET 0x02ACu
#define PAGE_A_OFFSET 0x0400u
#define PAGE_B_OFFSET 0x0C00u
/* 0x1400 is DataRecv configuration; its active flag is at 0x2440. */
#define RECV_WORKER_OFFSET 0x2440u
#define SEND_WORKER_OFFSET 0x1404u
#define ROUTE_WAKE_OFFSET 0x14ECu
#define CPU_UNLOCK_OFFSET 0x2FBCu
#define CPU_LOCK_OFFSET 0x2FCCu
#define DEVICE_STOP_OFFSET 0x3E78u
#define SEND_START_OFFSET 0x4C98u

/*
 * Firmware this plugin has been verified against.  SceAVConfig's text is
 * byte-identical on every one of them, so the single offset set above covers
 * the whole list and the module NID is the only thing telling the builds apart.
 * Adding a firmware means verifying its module and adding one row here.
 */
static const struct {
	uint32_t nid;
	const char *firmware;
} known_avconfig[] = {
	{ 0x222DDEB1u, "3.60" },
	{ 0xCC9A71FBu, "3.61" },
	{ 0x83636271u, "3.63" },
	{ 0x55A6E312u, "3.65" },
	{ 0x1A5B797Cu, "3.67" },
	{ 0xA1F08F46u, "3.68" },
	{ 0x5B294543u, "3.71" },
	{ 0x136D0561u, "3.73" }
};

/* 32-bit SCE export-table header used by kernel modules. */
typedef struct {
	uint16_t size;
	uint8_t version[2];
	uint16_t attributes;
	uint16_t function_count;
	uint32_t variable_count;
	uint32_t tls_variable_count;
	uint32_t library_nid;
	const char *library_name;
	const uint32_t *nid_table;
	const uintptr_t *entry_table;
} ModuleExports;

typedef char module_exports_size_must_be_0x20[
	(sizeof(ModuleExports) == 0x20u) ? 1 : -1];

/* Firmware name for a verified module NID, or NULL if we have not tested it. */
static const char *known_firmware(uint32_t nid)
{
	unsigned int i;

	for (i = 0; i < sizeof(known_avconfig) / sizeof(known_avconfig[0]); ++i) {
		if (known_avconfig[i].nid == nid)
			return known_avconfig[i].firmware;
	}
	return NULL;
}

static uint16_t read_u16(uintptr_t a)
{
	return (uint16_t)((const uint8_t *)a)[0] |
		(uint16_t)((const uint8_t *)a)[1] << 8;
}

/*
 * Recover the private data base from the movw/movt pair in the send-start
 * prologue.  This is derivation, not a check: the two segments are separate
 * allocations, so their runtime spacing cannot be assumed and the address has
 * to be read out of the code that uses it.
 */
static uintptr_t decode_data_base(uintptr_t movw, uintptr_t movt, unsigned int reg)
{
	uint16_t half[2];
	unsigned int i;

	for (i = 0; i < 2u; ++i) {
		uintptr_t at = i ? movt : movw;
		uint16_t first = read_u16(at);
		uint16_t second = read_u16(at + 2u);

		if ((first & 0xfbf0u) != (i ? 0xf2c0u : 0xf240u) ||
		    ((second >> 8) & 0x0fu) != reg)
			return 0;
		half[i] = (uint16_t)(((first & 0x000fu) << 12) |
			(((first >> 10) & 1u) << 11) |
			(((second >> 12) & 7u) << 8) | (second & 0x00ffu));
	}
	return ((uintptr_t)half[1] << 16) | half[0];
}

static uintptr_t find_audio_export(const tai_module_info_t *module,
	uint32_t function_nid)
{
	uintptr_t current = module->exports_start;

	while (current < module->exports_end) {
		const ModuleExports *exports = (const ModuleExports *)current;
		uintptr_t available = module->exports_end - current;
		uint32_t index;

		if (available < sizeof(*exports) || exports->size < sizeof(*exports) ||
		    exports->size > available)
			return 0;
		if (exports->library_nid == AUDIO_LIBRARY_NID) {
			if (exports->nid_table == NULL || exports->entry_table == NULL ||
			    ((uintptr_t)exports->nid_table & 3u) != 0u ||
			    ((uintptr_t)exports->entry_table & 3u) != 0u)
				return 0;
			for (index = 0; index < exports->function_count; ++index) {
				if (exports->nid_table[index] == function_nid)
					return exports->entry_table[index];
			}
		}
		current += exports->size;
	}
	return 0;
}

static int resolve_audio(AudioLayout *layout)
{
	tai_module_info_t module = {0};
	uintptr_t rate;
	uintptr_t channels;
	uintptr_t submit;

	module.size = sizeof(module);
	if (taiGetModuleInfoForKernel(KERNEL_PID, "SceAudio", &module) < 0 ||
	    module.exports_start == 0u || module.exports_start >= module.exports_end)
		return -1;
	rate = find_audio_export(&module, RAM_RATE_NID);
	channels = find_audio_export(&module, RAM_CHANNELS_NID);
	submit = find_audio_export(&module, RAM_SUBMIT_NID);
	if ((rate & 1u) == 0u || (channels & 1u) == 0u || (submit & 1u) == 0u)
		return -1;

	layout->ram_rate = (AudioRamRateFn)rate;
	layout->ram_channels = (AudioRamChannelsFn)channels;
	layout->ram_submit = (AudioRamSubmitFn)submit;
	return 0;
}

int resolver_open(AudioLayout *layout)
{
	tai_module_info_t module = {0};
	const char *firmware;
	uintptr_t text;
	uintptr_t data;

	if (layout == NULL)
		return -1;
	memset(layout, 0, sizeof(*layout));
	layout->module_id = -1;
	module.size = sizeof(module);
	if (taiGetModuleInfoForKernel(KERNEL_PID, "SceAVConfig", &module) < 0)
		return -1;
	if (module.modid < 0 || module.exports_start == 0u ||
	    module.exports_start >= module.exports_end)
		return -1;
	firmware = known_firmware(module.module_nid);
	if (firmware == NULL) {
		uac_log(LOG_PREFIX "AVConfig nid=0x%08x is not a verified firmware; "
			"report this NID\n", module.module_nid);
		return -1;
	}

	/*
	 * Underflow here would make text a wild pointer that the reads below then
	 * dereference, so bound it before subtracting.
	 */
	if (module.exports_start < AVCONFIG_EXPORTS_OFFSET)
		return -1;
	text = module.exports_start - AVCONFIG_EXPORTS_OFFSET;

	/*
	 * One halfword per hook site.  The NID already fixed which firmware this
	 * is; this only has to notice a SceAVConfig modified after load, and taiHEN
	 * patches a branch over the entry, so the first instruction is what moves.
	 */
	if (read_u16(text + SEND_START_OFFSET) != 0xb5f8u ||
	    read_u16(text + DEVICE_STOP_OFFSET) != 0xb510u) {
		uac_log(LOG_PREFIX "AVConfig %s hook entries failed validation\n",
			firmware);
		return -1;
	}

	/* Rejects both a failed decode (0) and a base that would underflow. */
	data = decode_data_base(text + SEND_START_OFFSET + 0x26u,
		text + SEND_START_OFFSET + 0x34u, 3u);
	if (data < SEND_WORKER_OFFSET) {
		uac_log(LOG_PREFIX "AVConfig %s data-base decode failed\n",
			firmware);
		return -1;
	}
	data -= SEND_WORKER_OFFSET;

#define DATA_U32(off) ((volatile uint32_t *)(data + (off)))
	layout->module_id = module.modid;
	layout->send_start_offset = SEND_START_OFFSET;
	layout->device_stop_offset = DEVICE_STOP_OFFSET;
	layout->route_word = DATA_U32(ROUTE_WORD_OFFSET);
	layout->selected_target = DATA_U32(SELECTED_TARGET_OFFSET);
	layout->transport_ready = DATA_U32(TRANSPORT_READY_OFFSET);
	layout->route_dirty = DATA_U32(ROUTE_DIRTY_OFFSET);
	layout->send_worker_active = DATA_U32(SEND_WORKER_OFFSET);
	layout->recv_worker_active = DATA_U32(RECV_WORKER_OFFSET);
	layout->route_lock = (void *)(data + ROUTE_LOCK_OFFSET);
	layout->page[0] = (void *)(data + PAGE_A_OFFSET);
	layout->page[1] = (void *)(data + PAGE_B_OFFSET);
	/* cpu_lock/cpu_unlock are ARM import stubs; route_wake is Thumb code. */
	layout->cpu_lock = (AudioCpuLockFn)(text + CPU_LOCK_OFFSET);
	layout->cpu_unlock = (AudioCpuUnlockFn)(text + CPU_UNLOCK_OFFSET);
	layout->route_wake = (AudioRouteWakeFn)((text + ROUTE_WAKE_OFFSET) | 1u);
#undef DATA_U32

	if (resolve_audio(layout) < 0) {
		uac_log(LOG_PREFIX "SceAudio RAM exports unavailable\n");
		resolver_close(layout);
		return -1;
	}
	uac_log(LOG_PREFIX "AVConfig nid=0x%08x firmware %s resolved\n",
		module.module_nid, firmware);
	return 0;
}

void resolver_close(AudioLayout *layout)
{
	if (layout != NULL) {
		memset(layout, 0, sizeof(*layout));
		layout->module_id = -1;
	}
}
