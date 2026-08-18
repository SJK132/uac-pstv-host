/*
 * UAC1 descriptor discovery and the three USBD driver callbacks.
 *
 * Everything here is either a pure function of a device's descriptors or a
 * callback that posts one command to the session thread and returns.  Callbacks
 * run on USBD's own thread and must not block, which used to shape this whole
 * file; now it costs nothing, because the work they ask for happens in
 * session.c on a thread that is allowed to take its time.
 *
 * The descriptor walk is the part to be careful with.  It reads memory USBD
 * owns, so every step is bounded by the enclosing descriptor's declared length
 * before it is followed -- see descriptor_fits() and descriptor_before().
 */

#include "uac1.h"
#include "log.h"
#include "session.h"

#include <stddef.h>
#include <stdint.h>

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

/* UAC1 endpoint sampling-frequency control bit, in bmaControls[0]. */
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

static const void *first_boundary(const void *a, const void *b)
{
	if (a == NULL)
		return b;
	if (b == NULL)
		return a;
	return (uintptr_t)a < (uintptr_t)b ? a : b;
}

static int descriptor_fits(const void *descriptor, uint32_t minimum,
	const void *parent_end)
{
	const uint8_t *bytes = descriptor;
	uintptr_t start;
	uintptr_t end;

	if (bytes == NULL || !descriptor_before(bytes, parent_end) ||
	    bytes[0] < minimum)
		return 0;
	start = (uintptr_t)bytes;
	end = start + bytes[0];
	return end >= start &&
	       (parent_end == NULL || end <= (uintptr_t)parent_end);
}

static const void *configuration_end(
	const SceUsbdConfigurationDescriptor *configuration,
	const void *next_configuration)
{
	uintptr_t start = (uintptr_t)configuration;
	uintptr_t end;

	if (!descriptor_fits(configuration, USB_CONFIGURATION_DESCRIPTOR_SIZE,
		next_configuration) ||
	    configuration->wTotalLength < configuration->bLength)
		return NULL;
	end = start + configuration->wTotalLength;
	if (end < start || (next_configuration != NULL &&
	    end > (uintptr_t)next_configuration))
		return NULL;
	return (const void *)end;
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

	if (payload == 0 || (raw & 0xe000u) != 0)
		return 0;
	if (speed != SCE_USBD_DEVICE_SPEED_HS)
		return payload <= 1023u && (raw & 0x1800u) == 0 ? payload : 0;
	if (payload > 1024u)
		return 0;

	/* HS bits 12..11 encode 1-3 transactions; value 3 is reserved. */
	transactions = (raw >> 11) & 0x03u;
	if (transactions == 3u)
		return 0;

	return payload * (transactions + 1u);
}

/* stream.c queues one 192-byte request per millisecond, so the endpoint has to
 * be serviced exactly that often. */
static int services_one_packet_per_ms(uint8_t speed, uint8_t interval)
{
	return (speed == SCE_USBD_DEVICE_SPEED_FS && interval == 1) ||
	       (speed == SCE_USBD_DEVICE_SPEED_HS && interval == 4);
}

static int uac1_header_valid(const uint8_t *header, const void *parent_end)
{
	uint8_t count;

	if (!descriptor_fits(header, 8, parent_end) ||
	    header[2] != UAC_CS_HEADER ||
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

static int format_supports_target(const uint8_t *format,
	const void *parent_end)
{
	uint8_t count;
	uint8_t index;
	uint32_t required;

	if (!descriptor_fits(format, 8, parent_end) ||
	    format[2] != UAC_AS_FORMAT_TYPE ||
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
	const void *interface_end)
{
	uint8_t *descriptor = next_descriptor(scan, interface,
		USB_DT_CS_INTERFACE);
	int pcm = 0;
	int format = 0;

	for (; descriptor_before(descriptor, interface_end);
	     descriptor = next_descriptor(scan, descriptor,
		USB_DT_CS_INTERFACE)) {
		if (descriptor_fits(descriptor, 7, interface_end) &&
		    descriptor[2] == UAC_AS_GENERAL &&
		    read_le16(descriptor + 5) == UAC_FORMAT_PCM)
			pcm = 1;
		if (format_supports_target(descriptor, interface_end))
			format = 1;
	}

	return pcm && format;
}

/* A CS_ENDPOINT belongs only to the standard endpoint immediately before it. */
static int endpoint_has_freq_control(const DeviceScan *scan,
	const SceUsbdEndpointDescriptor *endpoint,
	const SceUsbdEndpointDescriptor *next_ep,
	const void *interface_end)
{
	uint8_t *cs = next_descriptor(scan, endpoint, USB_DT_CS_ENDPOINT);
	const void *endpoint_end = first_boundary(next_ep, interface_end);

	return descriptor_fits(cs, 4, endpoint_end) &&
	       cs[2] == UAC_EP_GENERAL &&
	       (cs[3] & UAC_EP_FREQ_CONTROL) != 0;
}

static int build_stream_candidate(const DeviceScan *scan,
	const SceUsbdConfigurationDescriptor *configuration,
	const SceUsbdInterfaceDescriptor *interface,
	const void *interface_end,
	SceUsbdEndpointDescriptor *endpoint,
	Uac1Stream *found)
{
	SceUsbdEndpointDescriptor *next_ep;
	uint8_t sync_type;
	uint16_t capacity;

	if ((endpoint->bEndpointAddress & 0x0fu) == 0 ||
	    (endpoint->bEndpointAddress & 0x70u) != 0 ||
	    (endpoint->bEndpointAddress & SCE_USBD_ENDPOINT_DIRECTION_BITS) !=
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

	if (!services_one_packet_per_ms(scan->speed, endpoint->bInterval)) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: speed %u interval %u\n",
			endpoint->bEndpointAddress, scan->speed, endpoint->bInterval);
		return 0;
	}
	if (capacity < TARGET_PACKET_BYTES) {
		uac_log(LOG_PREFIX "reject ep 0x%02x: needs %u bytes, max %u\n",
			endpoint->bEndpointAddress, TARGET_PACKET_BYTES, capacity);
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
		scan, endpoint, next_ep, interface_end);
	found->max_packet_size = capacity;
	found->endpoint = endpoint;
	return 1;
}

/* Search the AS interfaces this control header claims in its baInterfaceNr[]. */
static int find_stream_for_function(const DeviceScan *scan,
	const SceUsbdConfigurationDescriptor *configuration,
	const void *config_end,
	const uint8_t *control_header,
	Uac1Stream *found)
{
	SceUsbdInterfaceDescriptor *interface;
	const void *cursor = configuration;

	while ((interface = next_interface(scan, cursor)) != NULL &&
	       descriptor_before(interface, config_end)) {
		SceUsbdInterfaceDescriptor *next_if = next_interface(scan, interface);
		const void *interface_end = first_boundary(next_if, config_end);
		SceUsbdEndpointDescriptor *endpoint;

		cursor = interface;
		if (!descriptor_fits(interface, USB_INTERFACE_DESCRIPTOR_SIZE,
			config_end) ||
		    !header_contains_interface(control_header,
			interface->bInterfaceNumber) ||
		    interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
		    interface->bInterfaceSubclass != UAC_SUBCLASS_STREAMING ||
		    interface->bAlternateSetting == 0 ||
		    interface->bNumEndpoints == 0 ||
		    !interface_supports_target(scan, interface, interface_end))
			continue;

		endpoint = next_endpoint(scan, interface);
		while (descriptor_before(endpoint, interface_end)) {
			SceUsbdEndpointDescriptor *next_ep = next_endpoint(scan, endpoint);

			if (descriptor_fits(endpoint, USB_ENDPOINT_DESCRIPTOR_SIZE,
				interface_end) &&
			    build_stream_candidate(scan, configuration, interface,
				interface_end, endpoint, found))
				return 1;
			endpoint = next_ep;
		}
	}

	return 0;
}

/* Walk every AC interface of every configuration looking for a UAC1 function. */
int uac1_find_stream(int device_id, Uac1Stream *found)
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
		SceUsbdConfigurationDescriptor *next_configuration =
			next_config(&scan, configuration);
		const void *config_end = configuration_end(configuration,
			next_configuration);
		SceUsbdInterfaceDescriptor *interface;
		const void *if_cursor = configuration;

		config_cursor = configuration;
		if (config_end == NULL)
			continue;
		while ((interface = next_interface(&scan, if_cursor)) != NULL &&
		       descriptor_before(interface, config_end)) {
			SceUsbdInterfaceDescriptor *next_if =
				next_interface(&scan, interface);
			const void *interface_end = first_boundary(next_if,
				config_end);
			uint8_t *descriptor;

			if_cursor = interface;
			if (!descriptor_fits(interface,
				USB_INTERFACE_DESCRIPTOR_SIZE, config_end) ||
			    interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
			    interface->bInterfaceSubclass != UAC_SUBCLASS_CONTROL)
				continue;

			for (descriptor = next_descriptor(&scan, interface,
				USB_DT_CS_INTERFACE);
			     descriptor_before(descriptor, interface_end);
			     descriptor = next_descriptor(&scan, descriptor,
				USB_DT_CS_INTERFACE)) {
				if (uac1_header_valid(descriptor, interface_end) &&
				    find_stream_for_function(&scan, configuration,
					config_end, descriptor, found))
					return 1;
			}
		}
	}

	return 0;
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
	if (!uac1_find_stream(device_id, &found)) {
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
		TARGET_PACKET_BYTES, found.frequency_control);
	return SCE_USBD_PROBE_SUCCEEDED;
}

/* SUCCEEDED means "this driver owns the device", not "the stream is up": setup
 * runs afterwards on the session thread. */
int uac1_attach(int device_id)
{
	TRACE_CALLBACK(attach_count, "attach callback #%u: device %d\n", device_id);

	if (!session_start(device_id))
		return SCE_USBD_ATTACH_FAILED;
	return SCE_USBD_ATTACH_SUCCEEDED;
}

/*
 * Always SUCCEEDED, including for a device we never owned.
 *
 * A detach for a session that has already retired is not an error: suspend
 * retires ahead of the detach that follows it, so the ordinary sleep path
 * arrives here with nothing left to do.  Reporting failure for that would make
 * a routine sequence look like a fault in the log.
 */
int uac1_detach(int device_id)
{
	TRACE_CALLBACK(detach_count, "detach callback #%u: device %d, active %d\n",
		device_id, session_device());

	session_stop_detached(device_id);
	return SCE_USBD_DETACH_SUCCEEDED;
}

void uac1_stream_failed(void)
{
	session_stop("stream failed");
}
