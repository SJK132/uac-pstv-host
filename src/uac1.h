#ifndef UAC_PSTV_UAC1_H
#define UAC_PSTV_UAC1_H

#include <stdint.h>

#include <psp2kern/usbd.h>

/* What the descriptor walk found: the one AS interface we know how to drive. */
typedef struct {
	uint8_t configuration;
	uint8_t interface_number;
	uint8_t alternate_setting;
	uint8_t endpoint_address;
	uint8_t interval;
	uint8_t speed;
	uint8_t frequency_control;
	/* Locked to the bus frame rather than recovering its clock from the rate
	 * we deliver at, so an unfed frame is a hole it must conceal and not a
	 * rate it can follow. */
	uint8_t synchronous;
	uint16_t max_packet_size;
	SceUsbdEndpointDescriptor *endpoint;
} Uac1Stream;

/* Walk a device's descriptors for a stream this transport can drive. */
int uac1_find_stream(int device_id, Uac1Stream *found);

/* USBD driver callbacks. Each one posts to the session thread and returns. */
int uac1_probe(int device_id);
int uac1_attach(int device_id);
int uac1_detach(int device_id);

/* Reported by the transport when a live stream fails. Never blocks. */
void uac1_stream_failed(void);

#endif
