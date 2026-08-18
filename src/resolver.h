#ifndef UAC_PSTV_RESOLVER_H
#define UAC_PSTV_RESOLVER_H

#include <stdint.h>

#include <psp2common/types.h>

typedef uint32_t (*AudioCpuLockFn)(void *lock);
typedef void (*AudioCpuUnlockFn)(void *lock, uint32_t token);
typedef int (*AudioRouteWakeFn)(uint32_t bits);
typedef int (*AudioRamRateFn)(int rate);
typedef int (*AudioRamChannelsFn)(int channels);
typedef int (*AudioRamSubmitFn)(int target, void *buffer,
	unsigned int frames);

typedef struct {
	SceUID module_id;
	uint32_t send_start_offset;
	uint32_t device_stop_offset;
	volatile uint32_t *route_word;
	volatile uint32_t *selected_target;
	volatile uint32_t *transport_ready;
	volatile uint32_t *route_dirty;
	volatile uint32_t *send_worker_active;
	volatile uint32_t *recv_worker_active;
	void *route_lock;
	void *page[2];
	AudioCpuLockFn cpu_lock;
	AudioCpuUnlockFn cpu_unlock;
	AudioRouteWakeFn route_wake;
	AudioRamRateFn ram_rate;
	AudioRamChannelsFn ram_channels;
	AudioRamSubmitFn ram_submit;
} AudioLayout;

int resolver_open(AudioLayout *layout);
void resolver_close(AudioLayout *layout);

#endif
