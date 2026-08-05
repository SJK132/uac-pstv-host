/* Deterministic detector for the drain-vs-lock-free-reader race.
 *
 * The test hook drains the source after uac_mixer_fill() copies its PCM but
 * before it commits read_frame. A plain store would move read_frame backward,
 * expose the second half of the stale seed, and replay it on the next fill.
 * Compare-exchange must instead let the drain win and silence both packets.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mixer.h"

int ksceKernelCopyFromUserProc(int pid, void *dst, const void *src,
	unsigned int len);
void uac_log(const char *format, ...);

int ksceKernelCopyFromUserProc(int pid, void *dst, const void *src,
	unsigned int len)
{
	(void)pid;
	memcpy(dst, src, len);
	return 0;
}

void uac_log(const char *format, ...) { (void)format; }

#define FRAMES 48
#define SEED_FRAMES (FRAMES * 2)

static void drain_before_commit(void)
{
	uac_mixer_test_before_commit = NULL;
	uac_mixer_source_open(0, 48000, 1, 1);
}

static int packet_is_silent(const int16_t *packet)
{
	int index;

	for (index = 0; index < FRAMES * 2; ++index)
		if (packet[index] != 0)
			return 0;
	return 1;
}

int main(void)
{
	int16_t seed[SEED_FRAMES * 2];
	int16_t output[FRAMES * 2];
	int index;

	for (index = 0; index < SEED_FRAMES * 2; ++index)
		seed[index] = 1234;

	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	uac_mixer_source_push(0, 1, seed, SEED_FRAMES, 1);
	uac_mixer_test_before_commit = drain_before_commit;

	uac_mixer_fill(output);
	if (!packet_is_silent(output)) {
		puts("  [FAIL] drain-crossing packet was not suppressed");
		return 1;
	}

	uac_mixer_fill(output);
	if (!packet_is_silent(output)) {
		puts("  [FAIL] stale PCM replayed after drain");
		return 1;
	}

	uac_mixer_stop();
	puts("  [PASS] drain won; no stale packet or counter rollback");
	return 0;
}
