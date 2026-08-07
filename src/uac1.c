#include "uac1.h"
#include "log.h"
#include "stream.h"

#include <stddef.h>
#include <stdint.h>

#include <psp2kern/kernel/cpu/cache.h>
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

typedef struct {
	int device_id;
	int control_pipe;
	int stream_pipe;
	int state;
	int cancelled;
	uint32_t generation;
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
} Uac1Device;

static Uac1Device active = {
	.device_id = -1,
	.control_pipe = -1,
	.stream_pipe = -1,
	.state = UAC1_STATE_IDLE,
};

static uint32_t generation_counter;
static uint32_t setup_guard;
static int retired_device_id = -1;
static uint8_t rate_buffer[64] __attribute__((aligned(64)));

#ifdef UAC_PSTV_ENABLE_LOGGING
static uint32_t probe_count;
static uint32_t attach_count;
static uint32_t detach_count;
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

static void *scan_descriptor(int device_id, const void *after, uint8_t type)
{
	return ksceUsbdScanStaticDescriptor(device_id, (void *)after,
		(SceUsbdDescriptorType)type);
}

static SceUsbdConfigurationDescriptor *next_configuration(
	int device_id, const void *after)
{
	return (SceUsbdConfigurationDescriptor *)scan_descriptor(
		device_id, after, SCE_USBD_DESCRIPTOR_CONFIGURATION);
}

static SceUsbdInterfaceDescriptor *next_interface(int device_id, const void *after)
{
	return (SceUsbdInterfaceDescriptor *)scan_descriptor(
		device_id, after, SCE_USBD_DESCRIPTOR_INTERFACE);
}

static SceUsbdEndpointDescriptor *next_endpoint(int device_id, const void *after)
{
	return (SceUsbdEndpointDescriptor *)scan_descriptor(
		device_id, after, SCE_USBD_DESCRIPTOR_ENDPOINT);
}

static int descriptor_before(const void *descriptor, const void *boundary)
{
	return descriptor != NULL &&
	       (boundary == NULL || (uintptr_t)descriptor < (uintptr_t)boundary);
}

static int descriptor_belongs_to_endpoint(
	const void *descriptor,
	const SceUsbdEndpointDescriptor *next_ep,
	const SceUsbdInterfaceDescriptor *next_if)
{
	/* A CS_ENDPOINT belongs only to the standard endpoint immediately before it. */
	return descriptor_before(descriptor, next_ep) &&
	       descriptor_before(descriptor, next_if);
}

static int get_device_speed(int device_id, uint8_t *speed)
{
	uint32_t guarded_speed = 0xffffffffu;
	int result;

	/* Some Vita firmware writes a word despite VitaSDK declaring uint8_t*. */
	result = ksceUsbdGetDeviceSpeed(device_id, (uint8_t *)&guarded_speed);
	if (result < 0)
		return result;

	*speed = (uint8_t)guarded_speed;
	if (*speed != SCE_USBD_DEVICE_SPEED_FS &&
	    *speed != SCE_USBD_DEVICE_SPEED_HS)
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

static int interface_supports_target(
	int device_id,
	SceUsbdInterfaceDescriptor *interface,
	SceUsbdInterfaceDescriptor *next_if)
{
	uint8_t *descriptor;
	int pcm = 0;
	int format = 0;

	descriptor = (uint8_t *)scan_descriptor(
		device_id, interface, USB_DT_CS_INTERFACE);
	while (descriptor_before(descriptor, next_if)) {
		if (descriptor[0] >= 7 && descriptor[2] == UAC_AS_GENERAL &&
		    read_le16(descriptor + 5) == UAC_FORMAT_PCM)
			pcm = 1;
		if (format_supports_target(descriptor))
			format = 1;

		descriptor = (uint8_t *)scan_descriptor(
			device_id, descriptor, USB_DT_CS_INTERFACE);
	}

	return pcm && format;
}

static int build_endpoint_candidate(
	int device_id,
	const SceUsbdConfigurationDescriptor *configuration,
	const SceUsbdInterfaceDescriptor *interface,
	SceUsbdInterfaceDescriptor *next_if,
	SceUsbdEndpointDescriptor *endpoint,
	uint8_t speed,
	Uac1Device *found)
{
	SceUsbdEndpointDescriptor *next_ep;
	uint8_t *class_endpoint;
	uint8_t sync_type;
	uint8_t frequency_control = 0;
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

	capacity = endpoint_capacity(endpoint->wMaxPacketSize, speed);
	if (capacity == 0) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: invalid max packet 0x%04x\n",
			endpoint->bEndpointAddress, endpoint->wMaxPacketSize);
		return 0;
	}

	packet_bytes = stream_packet_bytes(speed, endpoint->bInterval);
	if (packet_bytes == 0) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: speed %u interval %u\n",
			endpoint->bEndpointAddress, speed, endpoint->bInterval);
		return 0;
	}
	if (packet_bytes > capacity) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: needs %u bytes, max %u\n",
			endpoint->bEndpointAddress, packet_bytes, capacity);
		return 0;
	}

	next_ep = next_endpoint(device_id, endpoint);
	class_endpoint = (uint8_t *)scan_descriptor(
		device_id, endpoint, USB_DT_CS_ENDPOINT);
	if (descriptor_belongs_to_endpoint(class_endpoint, next_ep, next_if) &&
	    class_endpoint[0] >= 4 && class_endpoint[2] == UAC_EP_GENERAL &&
	    (class_endpoint[3] & UAC_EP_FREQ_CONTROL))
		frequency_control = 1;

	found->configuration = configuration->bConfigurationValue;
	found->interface_number = interface->bInterfaceNumber;
	found->alternate_setting = interface->bAlternateSetting;
	found->endpoint_address = endpoint->bEndpointAddress;
	found->interval = endpoint->bInterval;
	found->speed = speed;
	found->frequency_control = frequency_control;
	found->max_packet_size = capacity;
	found->packet_bytes = packet_bytes;
	found->endpoint = endpoint;
	return 1;
}

static int find_stream_for_function(
	int device_id,
	const SceUsbdConfigurationDescriptor *configuration,
	const SceUsbdConfigurationDescriptor *next_config,
	const uint8_t *control_header,
	uint8_t speed,
	Uac1Device *found)
{
	SceUsbdInterfaceDescriptor *interface;
	const void *cursor = configuration;

	while ((interface = next_interface(device_id, cursor)) != NULL &&
	       descriptor_before(interface, next_config)) {
		SceUsbdInterfaceDescriptor *next_if = next_interface(device_id, interface);
		SceUsbdEndpointDescriptor *endpoint;

		cursor = interface;
		if (!header_contains_interface(control_header,
			interface->bInterfaceNumber) ||
		    interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
		    interface->bInterfaceSubclass != UAC_SUBCLASS_STREAMING ||
		    interface->bAlternateSetting == 0 || interface->bNumEndpoints == 0 ||
		    !interface_supports_target(device_id, interface, next_if))
			continue;

		endpoint = next_endpoint(device_id, interface);
		while (descriptor_before(endpoint, next_if)) {
			SceUsbdEndpointDescriptor *next_ep = next_endpoint(device_id, endpoint);

			if (build_endpoint_candidate(device_id, configuration, interface,
				next_if, endpoint, speed, found))
				return 1;
			endpoint = next_ep;
		}
	}

	return 0;
}

static int find_target_stream(int device_id, Uac1Device *found)
{
	SceUsbdConfigurationDescriptor *configuration;
	const void *config_cursor = NULL;
	uint8_t speed;
	int result;

	result = get_device_speed(device_id, &speed);
	if (result < 0) {
		uac_log(LOG_PREFIX "device speed unavailable: 0x%08x\n", result);
		return 0;
	}

	/* Each UAC1 header owns only the AS interfaces in its baInterfaceNr[]. */
	while ((configuration = next_configuration(device_id, config_cursor)) != NULL) {
		SceUsbdConfigurationDescriptor *next_config =
			next_configuration(device_id, configuration);
		SceUsbdInterfaceDescriptor *interface;
		const void *if_cursor = configuration;

		config_cursor = configuration;
		while ((interface = next_interface(device_id, if_cursor)) != NULL &&
		       descriptor_before(interface, next_config)) {
			SceUsbdInterfaceDescriptor *next_if =
				next_interface(device_id, interface);
			uint8_t *descriptor;

			if_cursor = interface;
			if (interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
			    interface->bInterfaceSubclass != UAC_SUBCLASS_CONTROL)
				continue;

			descriptor = (uint8_t *)scan_descriptor(
				device_id, interface, USB_DT_CS_INTERFACE);
			while (descriptor_before(descriptor, next_if)) {
				if (uac1_header_valid(descriptor) &&
				    find_stream_for_function(device_id, configuration,
					next_config, descriptor, speed, found))
					return 1;

				descriptor = (uint8_t *)scan_descriptor(
					device_id, descriptor, USB_DT_CS_INTERFACE);
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
	uint32_t old;
	uint32_t next;

	for (;;) {
		old = __atomic_load_n(&setup_guard, __ATOMIC_ACQUIRE);
		if ((old & SETUP_REFS) == 0)
			return;
		next = (old & SETUP_CLOSING) | ((old & SETUP_REFS) - 1u);
		if (__atomic_compare_exchange_n(&setup_guard, &old, next, 0,
			__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
			break;
	}

	if (next == SETUP_CLOSING)
		try_finalize_cleanup();
}

static void clear_active_fields(void)
{
	active.endpoint = NULL;
	active.configuration = 0;
	active.interface_number = 0;
	active.alternate_setting = 0;
	active.endpoint_address = 0;
	active.interval = 0;
	active.speed = 0;
	active.frequency_control = 0;
	active.max_packet_size = 0;
	active.packet_bytes = 0;
}

static void try_finalize_cleanup(void)
{
	int expected = UAC1_STATE_CLEANING;
	int stream_pipe;
	int control_pipe;
	int device_id;
	int was_detached;

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
	if (stream_pipe >= 0)
		ksceUsbdClosePipe(stream_pipe);
	if (control_pipe >= 0)
		ksceUsbdClosePipe(control_pipe);

	device_id = __atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE);
	was_detached = __atomic_load_n(&active.cancelled, __ATOMIC_ACQUIRE);
	clear_active_fields();

	/* Async failure may be followed later by the real detach callback. */
	if (!was_detached)
		__atomic_store_n(&retired_device_id, device_id, __ATOMIC_RELEASE);

	/*
	 * Keep SETUP_CLOSING set until every field is ready for the next attach.
	 * A detach racing finalization may still set cancelled while device_id is
	 * owned, so release device_id first and clear cancelled afterwards.
	 */
	__atomic_store_n(&active.generation, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&active.device_id, -1, __ATOMIC_RELEASE);
	__atomic_store_n(&active.cancelled, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&active.state, UAC1_STATE_IDLE, __ATOMIC_RELEASE);

	/* Opening the guard is the final publication step. */
	__atomic_store_n(&setup_guard, 0, __ATOMIC_RELEASE);
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
		try_finalize_cleanup();
	return 1;
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
		active.interface_number, active.alternate_setting, result);
	if (result < 0) {
		request_cleanup(generation, "SET_INTERFACE failed");
		goto out;
	}
	if (!active.frequency_control) {
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
	request.wIndex = active.endpoint_address;
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
		active.interface_number, active.alternate_setting, interface_done,
		generation_arg(generation));
	uac_log(LOG_PREFIX "select stream submit: 0x%08x\n", submit);
	if (submit < 0)
		request_cleanup(generation, "SET_INTERFACE submit failed");

out:
	setup_leave();
}

int uac1_probe(int device_id)
{
	Uac1Device found = {0};
	SceUsbdDeviceDescriptor *device;

#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t count = __atomic_add_fetch(&probe_count, 1, __ATOMIC_RELAXED);
	uac_log(LOG_PREFIX "probe callback #%u: device %d\n", count, device_id);
#endif

	device = (SceUsbdDeviceDescriptor *)scan_descriptor(
		device_id, NULL, SCE_USBD_DESCRIPTOR_DEVICE);
	if (device == NULL) {
		uac_log(LOG_PREFIX "probe %d: no device descriptor\n", device_id);
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
	Uac1Device found = {0};
	uint32_t generation;
	int expected_device = -1;
	int control_pipe;
	int stream_pipe;
	int submit;
	int result = SCE_USBD_ATTACH_FAILED;

#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t count = __atomic_add_fetch(&attach_count, 1, __ATOMIC_RELAXED);
	uac_log(LOG_PREFIX "attach callback #%u: device %d\n", count, device_id);
#endif

	if (!setup_enter())
		return SCE_USBD_ATTACH_FAILED;

	/* device_id is the ownership token; only one attach can claim -1. */
	if (!__atomic_compare_exchange_n(&active.device_id, &expected_device,
		device_id, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
		uac_log(LOG_PREFIX "attach rejected: device %d already active\n",
			expected_device);
		goto out;
	}

	generation = next_generation();
	__atomic_store_n(&active.generation, generation, __ATOMIC_RELEASE);
	__atomic_store_n(&active.state, UAC1_STATE_CLAIMED, __ATOMIC_RELEASE);
	__atomic_store_n(&active.control_pipe, -1, __ATOMIC_RELEASE);
	__atomic_store_n(&active.stream_pipe, -1, __ATOMIC_RELEASE);

	/* A detach can race the short claim-initialization window. */
	if (__atomic_load_n(&active.cancelled, __ATOMIC_ACQUIRE)) {
		request_cleanup(generation, "detached during attach");
		goto out;
	}

	/* Probe and attach are separate callbacks, so rediscover the stream. */
	if (!find_target_stream(device_id, &found)) {
		request_cleanup(generation, "stream rediscovery failed");
		goto out;
	}
	if (!active_in_state(generation, UAC1_STATE_CLAIMED))
		goto out;

	active.configuration = found.configuration;
	active.interface_number = found.interface_number;
	active.alternate_setting = found.alternate_setting;
	active.endpoint_address = found.endpoint_address;
	active.interval = found.interval;
	active.speed = found.speed;
	active.frequency_control = found.frequency_control;
	active.max_packet_size = found.max_packet_size;
	active.packet_bytes = found.packet_bytes;
	active.endpoint = found.endpoint;

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

	stream_pipe = ksceUsbdOpenPipe(device_id, active.endpoint);
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

	submit = ksceUsbdSetConfiguration(control_pipe, active.configuration,
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
	uint32_t generation;

#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t count = __atomic_add_fetch(&detach_count, 1, __ATOMIC_RELAXED);
	uac_log(LOG_PREFIX "detach callback #%u: device %d, active %d\n", count,
		device_id, __atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE));
#endif

	if (device_id != __atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE)) {
		int expected = device_id;

		if (__atomic_compare_exchange_n(&retired_device_id, &expected, -1, 0,
			__ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
			uac_log(LOG_PREFIX "accepted late detach for device %d\n",
				device_id);
			return SCE_USBD_DETACH_SUCCEEDED;
		}
		return SCE_USBD_DETACH_FAILED;
	}

	/* Mark cancellation first in case attach has claimed but not initialized. */
	__atomic_store_n(&active.cancelled, 1, __ATOMIC_RELEASE);
	generation = __atomic_load_n(&active.generation, __ATOMIC_ACQUIRE);
	if (generation != 0)
		request_cleanup(generation, "device detached");

	uac_log(LOG_PREFIX "detached\n");
	return SCE_USBD_DETACH_SUCCEEDED;
}
