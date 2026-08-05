#ifndef UAC_PSTV_MSVC_ATOMIC_COMPAT_H
#define UAC_PSTV_MSVC_ATOMIC_COMPAT_H

/* Single-threaded Windows test shim. The race harness forces the critical
 * interleaving with UAC_MIXER_TEST, so it does not need host atomic intrinsics. */
#ifdef _MSC_VER
#include <stdint.h>

#define __attribute__(value)
#define __ATOMIC_RELAXED 0
#define __ATOMIC_ACQUIRE 1
#define __ATOMIC_RELEASE 2

#define __atomic_load_n(pointer, order) (*(pointer))
#define __atomic_store_n(pointer, value, order) (*(pointer) = (value))
#define __atomic_thread_fence(order) ((void)0)

static inline int msvc_test_exchange_int(int *pointer, int value)
{
	int previous = *pointer;

	*pointer = value;
	return previous;
}

static inline int msvc_test_compare_exchange_u32(uint32_t *pointer,
	uint32_t *expected, uint32_t desired)
{
	if (*pointer == *expected) {
		*pointer = desired;
		return 1;
	}
	*expected = *pointer;
	return 0;
}

#define __atomic_exchange_n(pointer, value, order) \
	msvc_test_exchange_int((int *)(pointer), (int)(value))
#define __atomic_compare_exchange_n(pointer, expected, desired, weak, \
	success_order, failure_order) \
	msvc_test_compare_exchange_u32((uint32_t *)(pointer), \
		(uint32_t *)(expected), (uint32_t)(desired))
#endif

#endif
