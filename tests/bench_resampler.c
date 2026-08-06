/* Host-side relative benchmark for the producer and consumer paths. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "mixer.h"

#define PACKET_FRAMES 48
#define BLOCK_PACKETS 10
#define ITERATIONS 20000

int ksceKernelCopyFromUserProc(int pid, void *dst, const void *src,
	unsigned int len)
{
	(void)pid;
	memcpy(dst, src, len);
	return 0;
}

void uac_log(const char *format, ...)
{
	(void)format;
}

static double benchmark(uint32_t rate, uint32_t input_frames)
{
	static int16_t input[480 * 2];
	int16_t output[PACKET_FRAMES * 2];
	struct timespec start;
	struct timespec finish;
	int iteration;
	uint32_t frame;

	for (frame = 0; frame < input_frames * 2u; ++frame)
		input[frame] = (int16_t)(frame * 7919u);
	uac_mixer_start();
	uac_mixer_source_open(0, rate, 1, 1);
	clock_gettime(CLOCK_MONOTONIC, &start);
	for (iteration = 0; iteration < ITERATIONS; ++iteration) {
		uac_mixer_source_push(0, 1, input, input_frames, 1);
		for (int packet = 0; packet < BLOCK_PACKETS; ++packet)
			uac_mixer_fill(output);
	}
	clock_gettime(CLOCK_MONOTONIC, &finish);
	uac_mixer_source_close(0);
	uac_mixer_stop();
	return ((finish.tv_sec - start.tv_sec) * 1e9 +
		(finish.tv_nsec - start.tv_nsec)) / ITERATIONS;
}

int main(void)
{
	double native = benchmark(48000, 480);
	double fir = benchmark(44100, 441);

	printf("  10 ms block, native 48 kHz: %.0f ns\n", native);
	printf("  10 ms block, 44.1 kHz FIR:  %.0f ns\n", fir);
	printf("  FIR/native host ratio:      %.2fx\n", fir / native);
	printf("  FIR host real-time margin:  %.0fx\n", 10000000.0 / fir);
	return 0;
}
