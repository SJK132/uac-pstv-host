/*
 * Locating Sony's private audio internals.
 *
 * Both SceAVConfig and SceAudio are byte-identical across every firmware in
 * known_avconfig[] -- established by diffing decrypted modules, which differ in
 * five bytes apiece: the module NID and one version digit.  So one offset set
 * covers the range for both, and the AVConfig NID is the single gate: an
 * unlisted one is refused rather than resolved on faith.
 *
 * What cannot be a constant is where either module's data segment lands, since
 * that is a runtime allocation.  decode_data_base() reads it out of the code
 * that uses it.
 */
#include "resolver.h"

#include "log.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <taihen.h>

#define LOG_PREFIX "[uac-pstv-resolver] "

/*
 * SceAudio's three RAM-output functions, by fixed text offset -- same evidence
 * as the AVConfig offsets below, and the same NID gate covers both.
 *
 * SCEAUDIO_EXPORTS_OFFSET was inferred, not read out: SceAudio's module NID
 * sits 0x28 ahead of its first export descriptor, as AVConfig's does.  The
 * prologue halfwords are what keep that safe -- a wrong text base fails the
 * comparison and resolution is refused rather than called into.
 */
#define SCEAUDIO_EXPORTS_OFFSET 0x768Cu
#define RAM_RATE_OFFSET 0x2E80u
#define RAM_CHANNELS_OFFSET 0x2ED8u
#define RAM_SUBMIT_OFFSET 0x2F28u
#define RAM_RATE_PROLOGUE 0xb570u
#define RAM_CHANNELS_PROLOGUE 0x1e43u
#define RAM_SUBMIT_PROLOGUE 0xb508u

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
	{ 0x4069C16Du, "3.69" },
	{ 0x3F226D11u, "3.70" },
	{ 0x5B294543u, "3.71" },
	{ 0x0790F1A9u, "3.72" },
	{ 0x136D0561u, "3.73" }
};

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

static int resolve_audio(AudioLayout *layout)
{
	tai_module_info_t module = {0};
	uintptr_t text;

	module.size = sizeof(module);
	if (taiGetModuleInfoForKernel(KERNEL_PID, "SceAudio", &module) < 0)
		return -1;
	/* Bound before subtracting; see the same guard on the AVConfig side. */
	if (module.exports_start < SCEAUDIO_EXPORTS_OFFSET)
		return -1;
	text = module.exports_start - SCEAUDIO_EXPORTS_OFFSET;

	if (read_u16(text + RAM_RATE_OFFSET) != RAM_RATE_PROLOGUE ||
	    read_u16(text + RAM_CHANNELS_OFFSET) != RAM_CHANNELS_PROLOGUE ||
	    read_u16(text + RAM_SUBMIT_OFFSET) != RAM_SUBMIT_PROLOGUE)
		return -1;

	/* All three are Thumb, so the entry points carry the low bit. */
	layout->ram_rate = (AudioRamRateFn)((text + RAM_RATE_OFFSET) | 1u);
	layout->ram_channels =
		(AudioRamChannelsFn)((text + RAM_CHANNELS_OFFSET) | 1u);
	layout->ram_submit = (AudioRamSubmitFn)((text + RAM_SUBMIT_OFFSET) | 1u);
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
		uac_log(LOG_PREFIX "SceAudio RAM functions failed validation\n");
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
