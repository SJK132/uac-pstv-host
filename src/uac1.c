/*
 * USB device lifecycle: descriptor discovery, session setup, session teardown.
 *
 * Two things worth knowing before changing anything here.
 *
 * Callbacks must not block.  probe/attach/detach run on USBD's own thread, so
 * setup is a state machine driven by control-transfer completions (CLAIMED ->
 * SET_CONFIGURATION -> SET_INTERFACE -> SET_RATE -> STREAMING) and teardown is
 * handed to a dedicated worker.  Anything that can take milliseconds belongs on
 * that worker, not in a callback.
 *
 * Callbacks carry a generation token, never a pointer.  A completion that
 * arrives after its session was retired sees a changed generation and returns
 * without touching anything, which is what makes rapid replug safe.
 */

#include "uac1.h"
#include "log.h"
#include "stream.h"

#include <stddef.h>
#include <stdint.h>

#include <psp2kern/kernel/cpu/cache.h>
#include <psp2kern/kernel/cpu/spinlock.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/usbd.h>

#define LOG_PREFIX "[uac-pstv] "

/* UAC1 descriptor constants used by this parser. */
#define USB_DT_CS_INTERFACE 0x24u
#define USB_DT_CS_ENDPOINT 0x25u
#define UAC_SUBCLASS_CONTROL 1u
#define UAC_SUBCLASS_STREAMING 2u
#define UAC_CS_HEADER 1u
#define UAC_AS_GENERAL 1u
#define UAC_AS_FORMAT_TYPE 2u
#define UAC_FORMAT_TYPE_I 1u
#define UAC_FORMAT_PCM 0x0001u
#define UAC_EP_GENERAL 1u
#define UAC_VERSION_1_0 0x0100u

/* UAC1 endpoint sampling-frequency control. */
#define UAC_SET_CUR 1u
#define UAC_EP_SAMPLING_FREQ 0x0100u
#define UAC_EP_FREQ_CONTROL 0x01u

/* Fixed descriptor sizes from USB 2.0, table 9-8 through table 9-13. */
#define USB_DEVICE_DESCRIPTOR_SIZE 18u
#define USB_CONFIGURATION_DESCRIPTOR_SIZE 9u
#define USB_INTERFACE_DESCRIPTOR_SIZE 9u
#define USB_ENDPOINT_DESCRIPTOR_SIZE 7u

/* Isochronous synchronization type bits in bmAttributes. */
#define USB_ISO_SYNC_MASK 0x0cu
#define USB_ISO_SYNC_ADAPTIVE 0x08u
#define USB_ISO_SYNC_SYNCHRONOUS 0x0cu

/* Fixed USB transport: 48 kHz, stereo, signed 16-bit PCM, 1 ms packets. */
#define TARGET_RATE 48000u
#define TARGET_CHANNELS 2u
#define TARGET_SUBFRAME_BYTES 2u
#define TARGET_BITS 16u
#define TARGET_PACKET_BYTES 192u

/* High bit blocks new setup users; low bits count users already inside. */
#define SETUP_CLOSING 0x80000000u
#define SETUP_REFS 0x7fffffffu

typedef enum {
	UAC1_STATE_IDLE = 0,
	UAC1_STATE_CLAIMED,
	UAC1_STATE_SET_CONFIGURATION,
	UAC1_STATE_SET_INTERFACE,
	UAC1_STATE_SET_RATE,
	UAC1_STATE_STREAMING,
	UAC1_STATE_CLEANING,
	UAC1_STATE_FINALIZING,
} Uac1State;

/* What the descriptor walk found: the one AS interface we know how to drive. */
typedef struct {
	uint8_t configuration;
	uint8_t interface_number;
	uint8_t alternate_setting;
	uint8_t endpoint_address;
	uint8_t interval;
	uint8_t speed;
	uint8_t frequency_control;
	uint16_t max_packet_size;
	uint16_t packet_bytes;
	SceUsbdEndpointDescriptor *endpoint;
} Uac1Stream;

/* Live session state, owned by the attach/detach lifecycle. */
typedef struct {
	int device_id;
	int control_pipe;
	int stream_pipe;
	int state;
	int cancelled;
	uint32_t generation;
	Uac1Stream stream;
} Uac1Session;

static Uac1Session active = {
	.device_id = -1,
	.control_pipe = -1,
	.stream_pipe = -1,
	.state = UAC1_STATE_IDLE,
};

static uint32_t generation_counter;
static uint32_t setup_guard;
static SceKernelSpinlock session_lock;
static int retired_device_id = -1;
static int retired_stream_pipe = -1;

/*
 * An attach USBD offered while a teardown was still in progress.
 *
 * USBD offers a device exactly once.  Refusing an attach therefore loses it for
 * good: the device stays enumerated with no session behind it, and only a
 * physical replug brings it back.  Recording the ID and retrying once the setup
 * gate reopens is what closes that window.  It opens widest coming out of sleep
 * with audio streaming, since that is when teardown takes longest.
 */
static int deferred_attach = -1;
static uint8_t rate_buffer[64] __attribute__((aligned(64)));

/*
 * Session teardown stops the feeder, releases the AVConfig route, removes the
 * hooks and closes both pipes.  That is bounded by audio_tap's own timeouts and
 * can run into seconds in the worst case, so it must never execute on the USBD
 * callback thread: blocking detach is exactly what makes rapid replug drop
 * events.  Callbacks only ever mark the session and poke this worker.
 */
#define TEARDOWN_WORK_BIT 0x00000001u
#define TEARDOWN_EXIT_BIT 0x00000002u

/* Must exceed one full teardown: see FEEDER_STOP_TIMEOUT_US in stream.c. */
#define TEARDOWN_JOIN_TIMEOUT_US 6000000u

/* One control transfer on a live device; short because failure is harmless. */
#define ALT0_DONE_BIT 0x00000001u
#define ALT0_TIMEOUT_US 250000u

static SceUID teardown_event = -1;
static SceUID teardown_thread = -1;
static SceUID alt0_event = -1;

/*
 * Callback tracing. The counters make replug storms readable in the log by
 * numbering each callback, and compile out entirely in release.
 */
#ifdef UAC_PSTV_ENABLE_LOGGING
static uint32_t probe_count;
static uint32_t attach_count;
static uint32_t detach_count;

#define TRACE_CALLBACK(counter, fmt, ...) \
	uac_log(LOG_PREFIX fmt, \
		__atomic_add_fetch(&(counter), 1, __ATOMIC_RELAXED), __VA_ARGS__)
#else
#define TRACE_CALLBACK(counter, fmt, ...) ((void)0)
#endif

static uint16_t read_le16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le24(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16);
}

/*
 * Descriptor walking.
 *
 * ksceUsbdScanStaticDescriptor() returns one descriptor at a time, so the walk
 * is expressed as "next X after Y".  Nesting is enforced by descriptor_before():
 * each inner loop stops once it reaches its parent's next sibling.  A NULL
 * boundary means there is no next sibling, i.e. run to the end of the set.
 */
typedef struct {
	int device_id;
	uint8_t speed;
} DeviceScan;

static void *next_descriptor(const DeviceScan *scan, const void *after,
	uint8_t type)
{
	return ksceUsbdScanStaticDescriptor(scan->device_id, (void *)after,
		(SceUsbdDescriptorType)type);
}

static SceUsbdConfigurationDescriptor *next_config(const DeviceScan *scan,
	const void *after)
{
	return next_descriptor(scan, after, SCE_USBD_DESCRIPTOR_CONFIGURATION);
}

static SceUsbdInterfaceDescriptor *next_interface(const DeviceScan *scan,
	const void *after)
{
	return next_descriptor(scan, after, SCE_USBD_DESCRIPTOR_INTERFACE);
}

static SceUsbdEndpointDescriptor *next_endpoint(const DeviceScan *scan,
	const void *after)
{
	return next_descriptor(scan, after, SCE_USBD_DESCRIPTOR_ENDPOINT);
}

static int descriptor_before(const void *descriptor, const void *boundary)
{
	return descriptor != NULL &&
	       (boundary == NULL || (uintptr_t)descriptor < (uintptr_t)boundary);
}

static int open_device_scan(int device_id, DeviceScan *scan)
{
	uint32_t guarded_speed = 0xffffffffu;
	int result;

	/* Some Vita firmware writes a word despite VitaSDK declaring uint8_t*. */
	result = ksceUsbdGetDeviceSpeed(device_id, (uint8_t *)&guarded_speed);
	if (result < 0)
		return result;

	scan->device_id = device_id;
	scan->speed = (uint8_t)guarded_speed;
	if (scan->speed != SCE_USBD_DEVICE_SPEED_FS &&
	    scan->speed != SCE_USBD_DEVICE_SPEED_HS)
		return -1;

	return 0;
}

static uint16_t endpoint_capacity(uint16_t raw, uint8_t speed)
{
	uint16_t payload = raw & 0x07ffu;
	uint16_t transactions;

	if (payload == 0)
		return 0;
	if (speed != SCE_USBD_DEVICE_SPEED_HS)
		return payload;

	/* HS bits 12..11 encode 1-3 transactions; value 3 is reserved. */
	transactions = (raw >> 11) & 0x03u;
	if (transactions == 3u)
		return 0;

	return payload * (transactions + 1u);
}

static uint16_t stream_packet_bytes(uint8_t speed, uint8_t interval)
{
	/* stream.c queues one 192-byte request per millisecond. */
	if ((speed == SCE_USBD_DEVICE_SPEED_FS && interval == 1) ||
	    (speed == SCE_USBD_DEVICE_SPEED_HS && interval == 4))
		return TARGET_PACKET_BYTES;

	return 0;
}

static int uac1_header_valid(const uint8_t *header)
{
	uint8_t count;

	if (header[0] < 8 || header[2] != UAC_CS_HEADER ||
	    read_le16(header + 3) != UAC_VERSION_1_0)
		return 0;

	/* bInCollection is followed by that many baInterfaceNr entries. */
	count = header[7];
	return count != 0 && (uint32_t)header[0] >= 8u + (uint32_t)count;
}

static int header_contains_interface(const uint8_t *header, uint8_t number)
{
	uint8_t index;

	for (index = 0; index < header[7]; ++index) {
		if (header[8u + index] == number)
			return 1;
	}
	return 0;
}

static int format_supports_target(const uint8_t *format)
{
	uint8_t count;
	uint8_t index;
	uint32_t required;

	if (format[0] < 8 || format[2] != UAC_AS_FORMAT_TYPE ||
	    format[3] != UAC_FORMAT_TYPE_I ||
	    format[4] != TARGET_CHANNELS ||
	    format[5] != TARGET_SUBFRAME_BYTES || format[6] != TARGET_BITS)
		return 0;

	count = format[7];
	if (count == 0) {
		/* Continuous minimum/maximum rate range. */
		return format[0] >= 14 && read_le24(format + 8) <= TARGET_RATE &&
		       read_le24(format + 11) >= TARGET_RATE;
	}

	/* Keep this wide: narrowing 8 + count * 3 could wrap on bad input. */
	required = 8u + (uint32_t)count * 3u;
	if ((uint32_t)format[0] < required)
		return 0;

	for (index = 0; index < count; ++index) {
		if (read_le24(format + 8u + (uint32_t)index * 3u) == TARGET_RATE)
			return 1;
	}
	return 0;
}

/* An AS interface must advertise PCM and a format we can actually send. */
static int interface_supports_target(const DeviceScan *scan,
	const SceUsbdInterfaceDescriptor *interface,
	const SceUsbdInterfaceDescriptor *next_if)
{
	uint8_t *descriptor = next_descriptor(scan, interface,
		USB_DT_CS_INTERFACE);
	int pcm = 0;
	int format = 0;

	for (; descriptor_before(descriptor, next_if);
	     descriptor = next_descriptor(scan, descriptor,
		USB_DT_CS_INTERFACE)) {
		if (descriptor[0] >= 7 && descriptor[2] == UAC_AS_GENERAL &&
		    read_le16(descriptor + 5) == UAC_FORMAT_PCM)
			pcm = 1;
		if (format_supports_target(descriptor))
			format = 1;
	}

	return pcm && format;
}

/* A CS_ENDPOINT belongs only to the standard endpoint immediately before it. */
static int endpoint_has_freq_control(const DeviceScan *scan,
	const SceUsbdEndpointDescriptor *endpoint,
	const SceUsbdEndpointDescriptor *next_ep,
	const SceUsbdInterfaceDescriptor *next_if)
{
	uint8_t *cs = next_descriptor(scan, endpoint, USB_DT_CS_ENDPOINT);

	return descriptor_before(cs, next_ep) && descriptor_before(cs, next_if) &&
	       cs[0] >= 4 && cs[2] == UAC_EP_GENERAL &&
	       (cs[3] & UAC_EP_FREQ_CONTROL) != 0;
}

static int build_stream_candidate(const DeviceScan *scan,
	const SceUsbdConfigurationDescriptor *configuration,
	const SceUsbdInterfaceDescriptor *interface,
	const SceUsbdInterfaceDescriptor *next_if,
	SceUsbdEndpointDescriptor *endpoint,
	Uac1Stream *found)
{
	SceUsbdEndpointDescriptor *next_ep;
	uint8_t sync_type;
	uint16_t capacity;
	uint16_t packet_bytes;

	if ((endpoint->bEndpointAddress & SCE_USBD_ENDPOINT_DIRECTION_BITS) !=
		SCE_USBD_ENDPOINT_DIRECTION_OUT ||
	    (endpoint->bmAttributes & SCE_USBD_ENDPOINT_TRANSFER_TYPE_BITS) !=
		SCE_USBD_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS)
		return 0;

	/* The fixed packet pump has no explicit or implicit feedback support. */
	sync_type = endpoint->bmAttributes & USB_ISO_SYNC_MASK;
	if (sync_type != USB_ISO_SYNC_ADAPTIVE &&
	    sync_type != USB_ISO_SYNC_SYNCHRONOUS) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: sync type 0x%02x\n",
			endpoint->bEndpointAddress, sync_type);
		return 0;
	}

	capacity = endpoint_capacity(endpoint->wMaxPacketSize, scan->speed);
	if (capacity == 0) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: invalid max packet 0x%04x\n",
			endpoint->bEndpointAddress, endpoint->wMaxPacketSize);
		return 0;
	}

	packet_bytes = stream_packet_bytes(scan->speed, endpoint->bInterval);
	if (packet_bytes == 0) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: speed %u interval %u\n",
			endpoint->bEndpointAddress, scan->speed, endpoint->bInterval);
		return 0;
	}
	if (packet_bytes > capacity) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: needs %u bytes, max %u\n",
			endpoint->bEndpointAddress, packet_bytes, capacity);
		return 0;
	}

	next_ep = next_endpoint(scan, endpoint);
	found->configuration = configuration->bConfigurationValue;
	found->interface_number = interface->bInterfaceNumber;
	found->alternate_setting = interface->bAlternateSetting;
	found->endpoint_address = endpoint->bEndpointAddress;
	found->interval = endpoint->bInterval;
	found->speed = scan->speed;
	found->frequency_control = (uint8_t)endpoint_has_freq_control(
		scan, endpoint, next_ep, next_if);
	found->max_packet_size = capacity;
	found->packet_bytes = packet_bytes;
	found->endpoint = endpoint;
	return 1;
}

/* Search the AS interfaces this control header claims in its baInterfaceNr[]. */
static int find_stream_for_function(const DeviceScan *scan,
	const SceUsbdConfigurationDescriptor *configuration,
	const SceUsbdConfigurationDescriptor *config_end,
	const uint8_t *control_header,
	Uac1Stream *found)
{
	SceUsbdInterfaceDescriptor *interface;
	const void *cursor = configuration;

	while ((interface = next_interface(scan, cursor)) != NULL &&
	       descriptor_before(interface, config_end)) {
		SceUsbdInterfaceDescriptor *next_if = next_interface(scan, interface);
		SceUsbdEndpointDescriptor *endpoint;

		cursor = interface;
		if (interface->bLength < USB_INTERFACE_DESCRIPTOR_SIZE ||
		    !header_contains_interface(control_header,
			interface->bInterfaceNumber) ||
		    interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
		    interface->bInterfaceSubclass != UAC_SUBCLASS_STREAMING ||
		    interface->bAlternateSetting == 0 ||
		    interface->bNumEndpoints == 0 ||
		    !interface_supports_target(scan, interface, next_if))
			continue;

		endpoint = next_endpoint(scan, interface);
		while (descriptor_before(endpoint, next_if)) {
			SceUsbdEndpointDescriptor *next_ep = next_endpoint(scan, endpoint);

			if (endpoint->bLength >= USB_ENDPOINT_DESCRIPTOR_SIZE &&
			    build_stream_candidate(scan, configuration, interface,
				next_if, endpoint, found))
				return 1;
			endpoint = next_ep;
		}
	}

	return 0;
}

/* Walk every AC interface of every configuration looking for a UAC1 function. */
static int find_target_stream(int device_id, Uac1Stream *found)
{
	SceUsbdConfigurationDescriptor *configuration;
	const void *config_cursor = NULL;
	DeviceScan scan;
	int result;

	result = open_device_scan(device_id, &scan);
	if (result < 0) {
		uac_log(LOG_PREFIX "device speed unavailable: 0x%08x\n", result);
		return 0;
	}

	while ((configuration = next_config(&scan, config_cursor)) != NULL) {
		SceUsbdConfigurationDescriptor *config_end =
			next_config(&scan, configuration);
		SceUsbdInterfaceDescriptor *interface;
		const void *if_cursor = configuration;

		config_cursor = configuration;
		if (configuration->bLength < USB_CONFIGURATION_DESCRIPTOR_SIZE)
			continue;
		while ((interface = next_interface(&scan, if_cursor)) != NULL &&
		       descriptor_before(interface, config_end)) {
			SceUsbdInterfaceDescriptor *next_if =
				next_interface(&scan, interface);
			uint8_t *descriptor;

			if_cursor = interface;
			if (interface->bLength < USB_INTERFACE_DESCRIPTOR_SIZE ||
			    interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
			    interface->bInterfaceSubclass != UAC_SUBCLASS_CONTROL)
				continue;

			for (descriptor = next_descriptor(&scan, interface,
				USB_DT_CS_INTERFACE);
			     descriptor_before(descriptor, next_if);
			     descriptor = next_descriptor(&scan, descriptor,
				USB_DT_CS_INTERFACE)) {
				if (uac1_header_valid(descriptor) &&
				    find_stream_for_function(&scan, configuration,
					config_end, descriptor, found))
					return 1;
			}
		}
	}

	return 0;
}

static uint32_t next_generation(void)
{
	uint32_t generation;

	/* Zero is reserved so NULL is never a valid callback token. */
	do {
		generation = __atomic_add_fetch(
			&generation_counter, 1, __ATOMIC_RELAXED);
	} while (generation == 0);
	return generation;
}

static void *generation_arg(uint32_t generation)
{
	return (void *)(uintptr_t)generation;
}

static uint32_t arg_generation(void *arg)
{
	return (uint32_t)(uintptr_t)arg;
}

static int active_in_state(uint32_t generation, Uac1State state)
{
	return __atomic_load_n(&active.generation, __ATOMIC_ACQUIRE) == generation &&
	       __atomic_load_n(&active.state, __ATOMIC_ACQUIRE) == (int)state;
}

static int transition_state(
	uint32_t generation, Uac1State old_state, Uac1State new_state)
{
	int expected = old_state;

	if (__atomic_load_n(&active.generation, __ATOMIC_ACQUIRE) != generation)
		return 0;
	return __atomic_compare_exchange_n(&active.state, &expected, new_state, 0,
		__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static void try_finalize_cleanup(void);
static int request_cleanup(uint32_t generation, const char *reason);
static void interface_done(int32_t result, int32_t count, void *arg);

/*
 * Safe to call spuriously: try_finalize_cleanup() only acts once the setup
 * guard is fully closed and it wins the CLEANING -> FINALIZING transition.
 */
static void signal_teardown(void)
{
	if (teardown_event >= 0)
		(void)ksceKernelSetEventFlag(teardown_event, TEARDOWN_WORK_BIT);
}

static int teardown_worker(SceSize args, void *argp)
{
	(void)args;
	(void)argp;
	for (;;) {
		uint32_t matched = 0;

		if (ksceKernelWaitEventFlag(teardown_event,
			TEARDOWN_WORK_BIT | TEARDOWN_EXIT_BIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, NULL) < 0)
			return -1;
		if (matched & TEARDOWN_WORK_BIT)
			try_finalize_cleanup();
		if (matched & TEARDOWN_EXIT_BIT)
			return 0;
	}
}

int uac1_lifecycle_init(void)
{
	int result;

	if (teardown_event < 0) {
		result = ksceKernelCreateEventFlag("uac_teardown", 0, 0, NULL);
		if (result < 0)
			return result;
		teardown_event = result;
	}
	if (alt0_event < 0) {
		result = ksceKernelCreateEventFlag("uac_alt0", 0, 0, NULL);
		if (result < 0)
			goto fail;
		alt0_event = result;
	}
	if (teardown_thread >= 0)
		return 0;
	result = ksceKernelCreateThread("uac_teardown", teardown_worker, 0x40,
		0x2000, 0, 0, NULL);
	if (result < 0)
		goto fail;
	teardown_thread = result;
	result = ksceKernelStartThread(teardown_thread, 0, NULL);
	if (result < 0) {
		(void)ksceKernelDeleteThread(teardown_thread);
		teardown_thread = -1;
		goto fail;
	}
	return 0;

fail:
	if (alt0_event >= 0 && ksceKernelDeleteEventFlag(alt0_event) >= 0)
		alt0_event = -1;
	if (ksceKernelDeleteEventFlag(teardown_event) >= 0)
		teardown_event = -1;
	return result;
}

/* Call only after the USB driver is unregistered, so no callback can requeue. */
int uac1_lifecycle_shutdown(void)
{
	SceUInt timeout = TEARDOWN_JOIN_TIMEOUT_US;
	int status;
	int result;

	/* Finish any session the last detach left pending, on this thread. */
	try_finalize_cleanup();

	if (teardown_thread >= 0) {
		if (teardown_event >= 0)
			(void)ksceKernelSetEventFlag(teardown_event,
				TEARDOWN_EXIT_BIT);
		result = ksceKernelWaitThreadEnd(teardown_thread, &status, &timeout);
		if (result < 0)
			return result;
		result = ksceKernelDeleteThread(teardown_thread);
		if (result < 0)
			return result;
		teardown_thread = -1;
	}
	if (alt0_event >= 0) {
		result = ksceKernelDeleteEventFlag(alt0_event);
		if (result < 0)
			return result;
		alt0_event = -1;
	}
	if (teardown_event >= 0) {
		result = ksceKernelDeleteEventFlag(teardown_event);
		if (result < 0)
			return result;
		teardown_event = -1;
	}
	return 0;
}

static int setup_enter(void)
{
	uint32_t old;

	for (;;) {
		old = __atomic_load_n(&setup_guard, __ATOMIC_ACQUIRE);
		if ((old & SETUP_CLOSING) || (old & SETUP_REFS) == SETUP_REFS)
			return 0;
		if (__atomic_compare_exchange_n(&setup_guard, &old, old + 1u, 0,
			__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			return 1;
	}
}

static void setup_leave(void)
{
	uint32_t old = __atomic_fetch_sub(&setup_guard, 1u, __ATOMIC_ACQ_REL);

	if (old == SETUP_CLOSING + 1u)
		signal_teardown();
}

static void alt0_done(int32_t result, int32_t count, void *arg)
{
	(void)count;
	(void)arg;
	(void)result; /* uac_log may compile out. */
	uac_log(LOG_PREFIX "select alt 0: 0x%08x\n", result);
	if (alt0_event >= 0)
		(void)ksceKernelSetEventFlag(alt0_event, ALT0_DONE_BIT);
}

/*
 * UAC1 reserves alternate setting 0 as the zero-bandwidth setting: selecting it
 * is how the host says "no audio is flowing".  Closing the pipe is host-side
 * only -- the device never hears about it, so it keeps the streaming interface
 * active and its clock domain locked to USB.  On an external DAC that shows up
 * as the clock source never falling back to internal.  Pointless once the
 * device is physically gone, so the caller gates on that.
 */
static void release_streaming_interface(int control_pipe)
{
	SceUInt timeout = ALT0_TIMEOUT_US;
	uint32_t matched;
	int submit;

	if (control_pipe < 0 || alt0_event < 0 || active.stream.alternate_setting == 0)
		return;
	(void)ksceKernelClearEventFlag(alt0_event, 0);
	submit = ksceUsbdSetInterface(control_pipe, active.stream.interface_number, 0,
		alt0_done, NULL);
	uac_log(LOG_PREFIX "select alt 0 submit: 0x%08x\n", submit);
	if (submit < 0)
		return;
	(void)ksceKernelWaitEventFlag(alt0_event, ALT0_DONE_BIT,
		SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR, &matched, &timeout);
}

/*
 * Runs on the teardown worker, never on a USBD callback, so re-running setup
 * here is safe.  If the device really went away while we were asleep the pipe
 * opens fail and request_cleanup() unwinds normally -- no worse than now.
 */
static void retry_deferred_attach(void)
{
	int device = __atomic_exchange_n(&deferred_attach, -1, __ATOMIC_ACQ_REL);

	if (device >= 0) {
		uac_log(LOG_PREFIX "retrying deferred attach: device %d\n", device);
		(void)uac1_attach(device);
	}
}

static void try_finalize_cleanup(void)
{
	SceKernelIntrStatus lock_state;
	int expected = UAC1_STATE_CLEANING;
	int stream_pipe;
	int control_pipe;
	int device_id;
	int pending_pipe = -1;
	int retire_pipe = -1;
	int was_detached;
	int wait_for_detach;

	if (__atomic_load_n(&setup_guard, __ATOMIC_ACQUIRE) != SETUP_CLOSING)
		return;
	if (!__atomic_compare_exchange_n(&active.state, &expected,
		UAC1_STATE_FINALIZING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return;

	uac_stream_stop();
	stream_pipe = __atomic_exchange_n(
		&active.stream_pipe, -1, __ATOMIC_ACQ_REL);
	control_pipe = __atomic_exchange_n(
		&active.control_pipe, -1, __ATOMIC_ACQ_REL);
	if (stream_pipe >= 0) {
		int close_result = ksceUsbdClosePipe(stream_pipe);

		uac_log(LOG_PREFIX "stream pipe close: 0x%08x\n", close_result);
		was_detached = __atomic_load_n(&active.cancelled, __ATOMIC_ACQUIRE);
		if (close_result >= 0 || was_detached)
			uac_stream_pipe_closed(stream_pipe);
		else
			pending_pipe = stream_pipe;
	}
	/* Only worth telling a device that is still there. */
	if (!__atomic_load_n(&active.cancelled, __ATOMIC_ACQUIRE))
		release_streaming_interface(control_pipe);
	if (control_pipe >= 0)
		ksceUsbdClosePipe(control_pipe);

	lock_state = ksceKernelSpinlockLowLockCpuSuspendIntr(&session_lock);
	device_id = __atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE);

	/*
	 * Keep SETUP_CLOSING set until every field is ready for the next attach.
	 * Publish the retired ID and any unclosed stream pipe before releasing
	 * device ownership. A late detach then proves the pipe is gone and can
	 * finish stream retirement before the setup gate reopens.
	 */
	__atomic_store_n(&retired_device_id, device_id, __ATOMIC_RELEASE);
	__atomic_store_n(&retired_stream_pipe, pending_pipe, __ATOMIC_RELEASE);
	__atomic_store_n(&active.generation, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&active.device_id, -1, __ATOMIC_RELEASE);
	was_detached = __atomic_load_n(&active.cancelled, __ATOMIC_ACQUIRE);
	if (was_detached) {
		int expected_device = device_id;

		(void)__atomic_compare_exchange_n(&retired_device_id,
			&expected_device, -1, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
		retire_pipe = __atomic_exchange_n(
			&retired_stream_pipe, -1, __ATOMIC_ACQ_REL);
	}
	wait_for_detach = __atomic_load_n(
		&retired_stream_pipe, __ATOMIC_ACQUIRE) >= 0;
	__atomic_store_n(&active.cancelled, 0, __ATOMIC_RELEASE);
	ksceKernelSpinlockLowUnlockCpuResumeIntr(&session_lock, lock_state);

	if (retire_pipe >= 0)
		uac_stream_pipe_closed(retire_pipe);
	if (!wait_for_detach) {
		__atomic_store_n(&active.state, UAC1_STATE_IDLE, __ATOMIC_RELEASE);
		/* Opening the guard is the final publication step. */
		__atomic_store_n(&setup_guard, 0, __ATOMIC_RELEASE);
		retry_deferred_attach();
	}
}

static int request_cleanup(uint32_t generation, const char *reason)
{
	int state;
	int expected;
	uint32_t old_guard;

	(void)reason; /* uac_log may compile out. */
	if (__atomic_load_n(&active.generation, __ATOMIC_ACQUIRE) != generation)
		return 0;

	for (;;) {
		state = __atomic_load_n(&active.state, __ATOMIC_ACQUIRE);
		if (state == UAC1_STATE_IDLE || state == UAC1_STATE_FINALIZING)
			return 0;
		if (state == UAC1_STATE_CLEANING)
			break;

		expected = state;
		if (__atomic_compare_exchange_n(&active.state, &expected,
			UAC1_STATE_CLEANING, 0, __ATOMIC_ACQ_REL,
			__ATOMIC_ACQUIRE)) {
			uac_log(LOG_PREFIX "cleanup generation %u: %s\n", generation,
				reason != NULL ? reason : "unspecified");
			break;
		}
	}

	/* Once closing is set, late callbacks cannot acquire a setup reference. */
	old_guard = __atomic_fetch_or(&setup_guard, SETUP_CLOSING,
		__ATOMIC_ACQ_REL);
	if ((old_guard & SETUP_REFS) == 0)
		signal_teardown();
	return 1;
}

/*
 * Suspend is not an unplug: the device is still on the bus, so active.cancelled
 * stays clear.  That is what lets try_finalize_cleanup() send
 * SET_INTERFACE(alt 0) before closing the control pipe, so an external DAC is
 * told the stream ended rather than being left with the interface nominally
 * active.  Returns immediately; the worker does the blocking part.
 *
 * Sleep does also deliver a real detach, but this beats it: the detach arrives
 * to find the session already retired and takes the late-detach branch.  That
 * ordering is the point.  Drop this and the detach wins instead, which sets
 * cancelled and skips alt 0 -- the device then keeps its clock domain locked to
 * USB across the sleep, which on a DAC with a switchable clock source is
 * visible as never falling back to internal.
 */
void uac1_suspend_session(void)
{
	uint32_t generation = __atomic_load_n(&active.generation, __ATOMIC_ACQUIRE);

	if (generation != 0)
		(void)request_cleanup(generation, "system suspend");
}

/* Re-drive a teardown that suspend interrupted. No-ops when nothing pends. */
void uac1_resume_retry(void)
{
	signal_teardown();
}

static void start_stream(uint32_t generation, Uac1State old_state)
{
	int result;

	if (!transition_state(generation, old_state, UAC1_STATE_STREAMING))
		return;

	result = uac_stream_start(
		__atomic_load_n(&active.stream_pipe, __ATOMIC_ACQUIRE));
	if (result < 0) {
		uac_log(LOG_PREFIX "stream start failed: 0x%08x\n", result);
		request_cleanup(generation, "stream start failed");
	}
}

static void rate_done(int32_t result, int32_t count, void *arg)
{
	uint32_t generation = arg_generation(arg);

	(void)count;
	if (!setup_enter())
		return;
	if (!active_in_state(generation, UAC1_STATE_SET_RATE))
		goto out;

	uac_log(LOG_PREFIX "set fixed 48 kHz: 0x%08x\n", result);
	if (result < 0)
		request_cleanup(generation, "48 kHz SET_CUR failed");
	else
		start_stream(generation, UAC1_STATE_SET_RATE);

out:
	setup_leave();
}

static void interface_done(int32_t result, int32_t count, void *arg)
{
	uint32_t generation = arg_generation(arg);
	SceUsbdDeviceRequest request;
	int submit;

	(void)count;
	if (!setup_enter())
		return;
	if (!active_in_state(generation, UAC1_STATE_SET_INTERFACE))
		goto out;

	uac_log(LOG_PREFIX "set interface %u/%u: 0x%08x\n",
		active.stream.interface_number, active.stream.alternate_setting, result);
	if (result < 0) {
		request_cleanup(generation, "SET_INTERFACE failed");
		goto out;
	}
	if (!active.stream.frequency_control) {
		start_stream(generation, UAC1_STATE_SET_INTERFACE);
		goto out;
	}
	if (!transition_state(generation, UAC1_STATE_SET_INTERFACE,
		UAC1_STATE_SET_RATE))
		goto out;

	rate_buffer[0] = (uint8_t)TARGET_RATE;
	rate_buffer[1] = (uint8_t)(TARGET_RATE >> 8);
	rate_buffer[2] = (uint8_t)(TARGET_RATE >> 16);
	ksceKernelDcacheCleanRange(rate_buffer, sizeof(rate_buffer));

	request.bmRequestType = SCE_USBD_REQTYPE_DIR_TO_DEVICE |
		SCE_USBD_REQTYPE_TYPE_CLASS | SCE_USBD_REQTYPE_RECIP_ENDPOINT;
	request.bRequest = UAC_SET_CUR;
	request.wValue = UAC_EP_SAMPLING_FREQ;
	request.wIndex = active.stream.endpoint_address;
	request.wLength = 3;

	submit = ksceUsbdControlTransfer(
		__atomic_load_n(&active.control_pipe, __ATOMIC_ACQUIRE), &request,
		rate_buffer, rate_done, generation_arg(generation));
	uac_log(LOG_PREFIX "fixed 48 kHz submit: 0x%08x\n", submit);
	if (submit < 0)
		request_cleanup(generation, "48 kHz SET_CUR submit failed");

out:
	setup_leave();
}

static void configuration_done(int32_t result, int32_t count, void *arg)
{
	uint32_t generation = arg_generation(arg);
	int submit;

	(void)count;
	if (!setup_enter())
		return;
	if (!active_in_state(generation, UAC1_STATE_SET_CONFIGURATION))
		goto out;

	uac_log(LOG_PREFIX "set configuration: 0x%08x\n", result);
	if (result < 0) {
		request_cleanup(generation, "SET_CONFIGURATION failed");
		goto out;
	}
	if (!transition_state(generation, UAC1_STATE_SET_CONFIGURATION,
		UAC1_STATE_SET_INTERFACE))
		goto out;

	submit = ksceUsbdSetInterface(
		__atomic_load_n(&active.control_pipe, __ATOMIC_ACQUIRE),
		active.stream.interface_number, active.stream.alternate_setting, interface_done,
		generation_arg(generation));
	uac_log(LOG_PREFIX "select stream submit: 0x%08x\n", submit);
	if (submit < 0)
		request_cleanup(generation, "SET_INTERFACE submit failed");

out:
	setup_leave();
}

int uac1_probe(int device_id)
{
	Uac1Stream found = {0};
	SceUsbdDeviceDescriptor *device;

	TRACE_CALLBACK(probe_count, "probe callback #%u: device %d\n", device_id);

	device = ksceUsbdScanStaticDescriptor(device_id, NULL,
		SCE_USBD_DESCRIPTOR_DEVICE);
	if (device == NULL || device->bLength < USB_DEVICE_DESCRIPTOR_SIZE) {
		uac_log(LOG_PREFIX "probe %d: invalid device descriptor\n", device_id);
		return SCE_USBD_PROBE_FAILED;
	}

	uac_log(LOG_PREFIX "probe %04x:%04x\n", device->idVendor,
		device->idProduct);
	if (!find_target_stream(device_id, &found)) {
		uac_log(LOG_PREFIX "probe rejected %04x:%04x\n", device->idVendor,
			device->idProduct);
		return SCE_USBD_PROBE_FAILED;
	}

	uac_log(LOG_PREFIX
		"UAC1 %04x:%04x, speed %u, if %u alt %u, ep 0x%02x, "
		"interval %u, max %u, stream %u, freq_ctl %u\n",
		device->idVendor, device->idProduct, found.speed,
		found.interface_number, found.alternate_setting,
		found.endpoint_address, found.interval, found.max_packet_size,
		found.packet_bytes, found.frequency_control);
	return SCE_USBD_PROBE_SUCCEEDED;
}

int uac1_attach(int device_id)
{
	Uac1Stream found = {0};
	SceKernelIntrStatus lock_state;
	uint32_t generation;
	int control_pipe;
	int stream_pipe;
	int submit;
	int result = SCE_USBD_ATTACH_FAILED;

	TRACE_CALLBACK(attach_count, "attach callback #%u: device %d\n", device_id);

	if (!setup_enter()) {
		/* Remember it; try_finalize_cleanup() retries once the gate reopens. */
		__atomic_store_n(&deferred_attach, device_id, __ATOMIC_RELEASE);
		uac_log(LOG_PREFIX "attach deferred: device %d, teardown in progress\n",
			device_id);
		return SCE_USBD_ATTACH_FAILED;
	}

	lock_state = ksceKernelSpinlockLowLockCpuSuspendIntr(&session_lock);
	if (__atomic_load_n(&active.device_id, __ATOMIC_RELAXED) != -1) {
		uac_log(LOG_PREFIX "attach rejected: device %d already active\n",
			__atomic_load_n(&active.device_id, __ATOMIC_RELAXED));
		ksceKernelSpinlockLowUnlockCpuResumeIntr(&session_lock, lock_state);
		goto out;
	}
	__atomic_store_n(&active.device_id, device_id, __ATOMIC_RELAXED);
	generation = next_generation();
	__atomic_store_n(&active.generation, generation, __ATOMIC_RELEASE);
	__atomic_store_n(&active.state, UAC1_STATE_CLAIMED, __ATOMIC_RELEASE);
	__atomic_store_n(&active.control_pipe, -1, __ATOMIC_RELEASE);
	__atomic_store_n(&active.stream_pipe, -1, __ATOMIC_RELEASE);
	__atomic_store_n(&active.cancelled, 0, __ATOMIC_RELEASE);
	ksceKernelSpinlockLowUnlockCpuResumeIntr(&session_lock, lock_state);

	/* Probe and attach are separate callbacks, so rediscover the stream. */
	if (!find_target_stream(device_id, &found)) {
		request_cleanup(generation, "stream rediscovery failed");
		goto out;
	}
	if (!active_in_state(generation, UAC1_STATE_CLAIMED))
		goto out;

	active.stream = found;

	control_pipe = ksceUsbdOpenPipe(device_id, NULL);
	if (control_pipe < 0) {
		uac_log(LOG_PREFIX "control pipe open failed: 0x%08x\n",
			control_pipe);
		request_cleanup(generation, "control pipe open failed");
		goto out;
	}
	__atomic_store_n(&active.control_pipe, control_pipe, __ATOMIC_RELEASE);
	if (!active_in_state(generation, UAC1_STATE_CLAIMED))
		goto out;

	stream_pipe = ksceUsbdOpenPipe(device_id, active.stream.endpoint);
	if (stream_pipe < 0) {
		uac_log(LOG_PREFIX "stream pipe open failed: 0x%08x\n",
			stream_pipe);
		request_cleanup(generation, "stream pipe open failed");
		goto out;
	}
	__atomic_store_n(&active.stream_pipe, stream_pipe, __ATOMIC_RELEASE);

	if (!transition_state(generation, UAC1_STATE_CLAIMED,
		UAC1_STATE_SET_CONFIGURATION)) {
		request_cleanup(generation, "setup state lost before configuration");
		goto out;
	}

	submit = ksceUsbdSetConfiguration(control_pipe, active.stream.configuration,
		configuration_done, generation_arg(generation));
	uac_log(LOG_PREFIX "configuration submit: 0x%08x\n", submit);
	if (submit < 0) {
		request_cleanup(generation, "SET_CONFIGURATION submit failed");
		goto out;
	}

	result = SCE_USBD_ATTACH_SUCCEEDED;

out:
	setup_leave();
	return result;
}

int uac1_detach(int device_id)
{
	SceKernelIntrStatus lock_state;
	uint32_t generation;
	int retired_pipe;
	int result;

	TRACE_CALLBACK(detach_count, "detach callback #%u: device %d, active %d\n",
		device_id, __atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE));

	lock_state = ksceKernelSpinlockLowLockCpuSuspendIntr(&session_lock);
	if (device_id != __atomic_load_n(&active.device_id, __ATOMIC_RELAXED)) {
		if (device_id == __atomic_load_n(&retired_device_id,
			__ATOMIC_RELAXED)) {
			__atomic_store_n(&retired_device_id, -1, __ATOMIC_RELAXED);
			retired_pipe = __atomic_exchange_n(
				&retired_stream_pipe, -1, __ATOMIC_RELAXED);
			ksceKernelSpinlockLowUnlockCpuResumeIntr(
				&session_lock, lock_state);
			if (retired_pipe >= 0) {
				uac_stream_pipe_closed(retired_pipe);
				__atomic_store_n(&active.state, UAC1_STATE_IDLE,
					__ATOMIC_RELEASE);
				__atomic_store_n(&setup_guard, 0, __ATOMIC_RELEASE);
			}
			uac_log(LOG_PREFIX "accepted late detach for device %d\n",
				device_id);
			return SCE_USBD_DETACH_SUCCEEDED;
		}
		ksceKernelSpinlockLowUnlockCpuResumeIntr(&session_lock, lock_state);
		return SCE_USBD_DETACH_FAILED;
	}

	__atomic_store_n(&active.cancelled, 1, __ATOMIC_RELEASE);
	generation = __atomic_load_n(&active.generation, __ATOMIC_ACQUIRE);
	ksceKernelSpinlockLowUnlockCpuResumeIntr(&session_lock, lock_state);
	if (generation != 0)
		result = request_cleanup(generation, "device detached");
	else
		result = 0;

	uac_log(LOG_PREFIX "detached\n");
	(void)result;
	return SCE_USBD_DETACH_SUCCEEDED;
}
