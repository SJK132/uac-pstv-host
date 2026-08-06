#include "uac1.h"
#include "log.h"
#include "stream.h"

#include <psp2kern/usbd.h>
#include <psp2kern/kernel/cpu/cache.h>
#include <stdint.h>

#define LOG_PREFIX "[uac-pstv] "

#define USB_DT_CS_INTERFACE 0x24
#define USB_DT_CS_ENDPOINT 0x25
#define UAC_SUBCLASS_CONTROL 1
#define UAC_SUBCLASS_STREAMING 2
#define UAC_CS_HEADER 1
#define UAC_AS_FORMAT_TYPE 2
#define UAC_FORMAT_TYPE_I 1
#define UAC_SET_CUR 1
#define UAC_EP_SAMPLING_FREQ 0x0100

#define TARGET_RATE 48000u
#define TARGET_CHANNELS 2u
#define TARGET_SUBFRAME_BYTES 2u
#define TARGET_BITS 16u
#define TARGET_FRAME_BYTES 192u

typedef struct {
	int device_id;
	int control_pipe;
	int stream_pipe;
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
};
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

static uint16_t stream_packet_bytes(const Uac1Device *device)
{
	uint32_t bytes;
	uint32_t period;

	if (device->speed == SCE_USBD_DEVICE_SPEED_FS)
		return TARGET_FRAME_BYTES;
	if (device->speed != SCE_USBD_DEVICE_SPEED_HS ||
		device->interval == 0 || device->interval > 16)
		return 0;
	period = 1u << (device->interval - 1u);
	bytes = ((TARGET_RATE * TARGET_CHANNELS * TARGET_SUBFRAME_BYTES * period) +
		7999u) / 8000u;
	return bytes > 0xffffu ? 0 : (uint16_t)bytes;
}

static SceUsbdInterfaceDescriptor *next_interface(int device_id,
	SceUsbdInterfaceDescriptor *interface)
{
	return (SceUsbdInterfaceDescriptor *)ksceUsbdScanStaticDescriptor(
		device_id, interface, SCE_USBD_DESCRIPTOR_INTERFACE);
}

static int descriptor_belongs_to_interface(const void *descriptor,
	const SceUsbdInterfaceDescriptor *next)
{
	return descriptor != NULL &&
	       (next == NULL || (uintptr_t)descriptor < (uintptr_t)next);
}

static int device_is_uac1(int device_id)
{
	SceUsbdInterfaceDescriptor *interface = NULL;

	while ((interface = next_interface(device_id, interface)) != NULL) {
		SceUsbdInterfaceDescriptor *next;
		uint8_t *class_descriptor;

		if (interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
		    interface->bInterfaceSubclass != UAC_SUBCLASS_CONTROL)
			continue;

		next = next_interface(device_id, interface);
		class_descriptor = (uint8_t *)ksceUsbdScanStaticDescriptor(
			device_id, interface, USB_DT_CS_INTERFACE);
		while (descriptor_belongs_to_interface(class_descriptor, next)) {
			if (class_descriptor[0] >= 5 &&
			    class_descriptor[2] == UAC_CS_HEADER)
				return read_le16(class_descriptor + 3) == 0x0100;
			class_descriptor = (uint8_t *)ksceUsbdScanStaticDescriptor(
				device_id, class_descriptor, USB_DT_CS_INTERFACE);
		}
	}
	return 0;
}

static int format_supports_target(const uint8_t *format)
{
	uint8_t count;
	uint8_t index;

	if (format[0] < 8 || format[2] != UAC_AS_FORMAT_TYPE ||
	    format[3] != UAC_FORMAT_TYPE_I || format[4] != TARGET_CHANNELS ||
	    format[5] != TARGET_SUBFRAME_BYTES || format[6] != TARGET_BITS)
		return 0;

	count = format[7];
	if (count == 0) {
		if (format[0] < 14)
			return 0;
		return read_le24(format + 8) <= TARGET_RATE &&
		       read_le24(format + 11) >= TARGET_RATE;
	}
	if (format[0] < (uint8_t)(8 + count * 3))
		return 0;
	for (index = 0; index < count; ++index)
		if (read_le24(format + 8 + index * 3) == TARGET_RATE)
			return 1;
	return 0;
}

static int find_target_stream(int device_id, Uac1Device *found)
{
	SceUsbdConfigurationDescriptor *configuration;
	SceUsbdInterfaceDescriptor *interface = NULL;

	configuration = (SceUsbdConfigurationDescriptor *)
		ksceUsbdScanStaticDescriptor(device_id, NULL,
			SCE_USBD_DESCRIPTOR_CONFIGURATION);
	if (configuration == NULL)
		return 0;
	if (!device_is_uac1(device_id))
		return 0;

	while ((interface = next_interface(device_id, interface)) != NULL) {
		SceUsbdInterfaceDescriptor *next;
		SceUsbdEndpointDescriptor *endpoint;
		uint8_t *class_descriptor;
		int format_ok = 0;

		if (interface->bInterfaceClass != SCE_USBD_CLASS_AUDIO ||
			interface->bInterfaceSubclass != UAC_SUBCLASS_STREAMING ||
			interface->bAlternateSetting == 0 || interface->bNumEndpoints == 0)
			continue;
		next = next_interface(device_id, interface);
		class_descriptor = (uint8_t *)ksceUsbdScanStaticDescriptor(
			device_id, interface, USB_DT_CS_INTERFACE);
		while (descriptor_belongs_to_interface(class_descriptor, next)) {
			if (format_supports_target(class_descriptor)) {
				format_ok = 1;
			}
			class_descriptor = (uint8_t *)ksceUsbdScanStaticDescriptor(
				device_id, class_descriptor, USB_DT_CS_INTERFACE);
		}
		if (!format_ok)
			continue;
		endpoint = (SceUsbdEndpointDescriptor *)ksceUsbdScanStaticDescriptor(
			device_id, interface, SCE_USBD_DESCRIPTOR_ENDPOINT);
		while (descriptor_belongs_to_interface(endpoint, next)) {
			uint8_t *class_endpoint;
			uint32_t speed_result;
			if ((endpoint->bEndpointAddress & SCE_USBD_ENDPOINT_DIRECTION_BITS) ==
					SCE_USBD_ENDPOINT_DIRECTION_OUT &&
				(endpoint->bmAttributes & SCE_USBD_ENDPOINT_TRANSFER_TYPE_BITS) ==
					SCE_USBD_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS &&
				    (endpoint->wMaxPacketSize & 0x07ffu) >= 24u) {
				found->configuration = configuration->bConfigurationValue;
				found->interface_number = interface->bInterfaceNumber;
				found->alternate_setting = interface->bAlternateSetting;
				found->endpoint_address = endpoint->bEndpointAddress;
				found->interval = endpoint->bInterval;
				found->max_packet_size = endpoint->wMaxPacketSize & 0x07ffu;
				found->frequency_control = 0;
				class_endpoint = (uint8_t *)ksceUsbdScanStaticDescriptor(
					device_id, endpoint,
					(SceUsbdDescriptorType)USB_DT_CS_ENDPOINT);
				if (descriptor_belongs_to_interface(class_endpoint, next) &&
					class_endpoint[0] >= 4 && class_endpoint[2] == 1 &&
					(class_endpoint[3] & 1u))
					found->frequency_control = 1;
				/* The public declaration says uint8_t, but this firmware writes a
				 * full word.  Use guarded word storage so it cannot overwrite the
				 * adjacent packet-size fields. */
				speed_result = 0xa5a5a501u;
				ksceUsbdGetDeviceSpeed(device_id, (uint8_t *)&speed_result);
				found->speed = (uint8_t)speed_result;
				found->packet_bytes = stream_packet_bytes(found);
				if (found->max_packet_size == 0 && found->packet_bytes != 0) {
					/* Some Vita descriptor reads return zero here even though the
					 * endpoint accepts the fixed 48 kHz packet size. */
					found->max_packet_size = found->packet_bytes;
				}
				if (found->packet_bytes == 0 &&
					(found->speed != SCE_USBD_DEVICE_SPEED_FS &&
					 found->speed != SCE_USBD_DEVICE_SPEED_HS)) {
					/* Some Vita USB revisions do not report bus speed.  This
					 * adapter is a 48 kHz full-speed-style 192-byte stream. */
					found->packet_bytes = TARGET_FRAME_BYTES;
				}
				if (found->packet_bytes == 0 ||
					found->packet_bytes > found->max_packet_size) {
					endpoint = (SceUsbdEndpointDescriptor *)ksceUsbdScanStaticDescriptor(
						device_id, endpoint, SCE_USBD_DESCRIPTOR_ENDPOINT);
					continue;
				}
				found->endpoint = endpoint;
				return 1;
			}
			endpoint = (SceUsbdEndpointDescriptor *)ksceUsbdScanStaticDescriptor(
				device_id, endpoint, SCE_USBD_DESCRIPTOR_ENDPOINT);
		}
	}
	return 0;
}

static void start_stream(Uac1Device *device)
{
	int result;
	if (__atomic_load_n(&device->device_id, __ATOMIC_ACQUIRE) < 0 ||
		device->stream_pipe < 0)
		return;
	result = uac_stream_start(device->stream_pipe, device->packet_bytes,
		device->speed, device->interval);
	if (result < 0)
		uac_log(LOG_PREFIX "stream start failed: 0x%08x\n", result);
}

static void rate_done(int32_t result, int32_t count, void *arg)
{
	Uac1Device *device = (Uac1Device *)arg;
	(void)result;
	(void)count;
	uac_log(LOG_PREFIX "set fixed 48 kHz after interface: 0x%08x\n", result);
	start_stream(device);
}

static void interface_done(int32_t result, int32_t count, void *arg)
{
	Uac1Device *device = (Uac1Device *)arg;
	SceUsbdDeviceRequest request;
	int submit;
	(void)count;
	uac_log(LOG_PREFIX "set interface %u/%u: 0x%08x\n",
		device->interface_number, device->alternate_setting, result);
	if (result < 0)
		return;
	if (device->frequency_control) {
		rate_buffer[0] = (uint8_t)TARGET_RATE;
		rate_buffer[1] = (uint8_t)(TARGET_RATE >> 8);
		rate_buffer[2] = (uint8_t)(TARGET_RATE >> 16);
		ksceKernelDcacheCleanRange(rate_buffer, sizeof(rate_buffer));
		request.bmRequestType = SCE_USBD_REQTYPE_DIR_TO_DEVICE |
			SCE_USBD_REQTYPE_TYPE_CLASS |
			SCE_USBD_REQTYPE_RECIP_ENDPOINT;
		request.bRequest = UAC_SET_CUR;
		request.wValue = UAC_EP_SAMPLING_FREQ;
		request.wIndex = device->endpoint_address;
		request.wLength = 3;
		submit = ksceUsbdControlTransfer(device->control_pipe, &request,
			rate_buffer, rate_done, device);
		uac_log(LOG_PREFIX "fixed 48 kHz submit after interface: 0x%08x\n",
			submit);
		if (submit >= 0)
			return;
	}
	start_stream(device);
}

static void configuration_done(int32_t result, int32_t count, void *arg)
{
	Uac1Device *device = (Uac1Device *)arg;
	int submit;
	(void)count;

	uac_log(LOG_PREFIX "set configuration: 0x%08x\n", result);
	if (result < 0)
		return;
	uac_log(LOG_PREFIX "selecting fixed 48 kHz stream\n");
	submit = ksceUsbdSetInterface(device->control_pipe,
		device->interface_number, device->alternate_setting,
		interface_done, device);
	(void)submit;
	uac_log(LOG_PREFIX "select stream submit: 0x%08x\n", submit);
}

int uac1_probe(int device_id)
{
	Uac1Device found = {0};
	SceUsbdDeviceDescriptor *device;
#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t count = __atomic_add_fetch(&probe_count, 1,
		__ATOMIC_RELAXED);

	uac_log(LOG_PREFIX "probe callback #%u: device %d\n", count,
		device_id);
#endif

	device = (SceUsbdDeviceDescriptor *)ksceUsbdScanStaticDescriptor(
		device_id, NULL, SCE_USBD_DESCRIPTOR_DEVICE);
	if (device == NULL) {
		uac_log(LOG_PREFIX "probe %d: no device descriptor\n", device_id);
		return SCE_USBD_PROBE_FAILED;
	}
	uac_log(LOG_PREFIX "probe %04x:%04x\n", device->idVendor,
		device->idProduct);
	if (!find_target_stream(device_id, &found)) {
		uac_log(LOG_PREFIX "probe rejected %04x:%04x\n",
			device->idVendor, device->idProduct);
		return SCE_USBD_PROBE_FAILED;
	}

	uac_log(LOG_PREFIX
		"UAC1 %04x:%04x, speed %u, if %u alt %u, ep 0x%02x, interval %u, max %u, stream %u, freq_ctl %u\n",
		device->idVendor, device->idProduct, found.speed,
		found.interface_number, found.alternate_setting, found.endpoint_address,
		found.interval, found.max_packet_size, found.packet_bytes,
		found.frequency_control);
	return SCE_USBD_PROBE_SUCCEEDED;
}

int uac1_attach(int device_id)
{
	int expected = -1;
	int submit;
#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t count = __atomic_add_fetch(&attach_count, 1,
		__ATOMIC_RELAXED);

	uac_log(LOG_PREFIX "attach callback #%u: device %d\n", count,
		device_id);
#endif

	/* Claim the single active slot atomically: a plain load-then-store
	 * would let two concurrent attach callbacks both pass the check. */
	if (!__atomic_compare_exchange_n(&active.device_id, &expected, device_id,
		0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
		return SCE_USBD_ATTACH_FAILED;

	active.control_pipe = -1;
	active.stream_pipe = -1;
	if (!find_target_stream(device_id, &active))
		goto fail;

	active.control_pipe = ksceUsbdOpenPipe(device_id, NULL);
	if (active.control_pipe < 0)
		goto fail;
	active.stream_pipe = ksceUsbdOpenPipe(device_id, active.endpoint);
	if (active.stream_pipe < 0)
		goto fail;

	submit = ksceUsbdSetConfiguration(active.control_pipe,
		active.configuration, configuration_done, &active);
	uac_log(LOG_PREFIX "configuration submit: 0x%08x\n", submit);
	if (submit < 0)
		goto fail;
	return SCE_USBD_ATTACH_SUCCEEDED;

fail:
	if (active.stream_pipe >= 0)
		ksceUsbdClosePipe(active.stream_pipe);
	if (active.control_pipe >= 0)
		ksceUsbdClosePipe(active.control_pipe);
	__atomic_store_n(&active.device_id, -1, __ATOMIC_RELEASE);
	active.stream_pipe = -1;
	active.control_pipe = -1;
	return SCE_USBD_ATTACH_FAILED;
}

int uac1_detach(int device_id)
{
#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t count = __atomic_add_fetch(&detach_count, 1,
		__ATOMIC_RELAXED);

	uac_log(LOG_PREFIX "detach callback #%u: device %d, active %d\n",
		count, device_id,
		__atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE));
#endif
	if (device_id != __atomic_load_n(&active.device_id, __ATOMIC_ACQUIRE))
		return SCE_USBD_DETACH_FAILED;
	uac_stream_stop();
	if (active.stream_pipe >= 0)
		ksceUsbdClosePipe(active.stream_pipe);
	if (active.control_pipe >= 0)
		ksceUsbdClosePipe(active.control_pipe);
	__atomic_store_n(&active.device_id, -1, __ATOMIC_RELEASE);
	active.stream_pipe = -1;
	active.control_pipe = -1;
	uac_log(LOG_PREFIX "detached\n");
	return SCE_USBD_DETACH_SUCCEEDED;
}
