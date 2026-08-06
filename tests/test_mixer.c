/* Host-side harness for src/mixer.c. Stubs the two kernel calls it makes and
 * exercises ring wrap, mixing, saturation, partial fills, drain-vs-reader,
 * and the resampler. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

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

void uac_log(const char *format, ...)
{
	(void)format;
}

#define FRAMES 48
#define SAMPLES (FRAMES * 2)
#define PI 3.14159265358979323846

static int failures;

static void check(int ok, const char *what)
{
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
	if (!ok)
		failures++;
}

/* Fill a stereo buffer with a ramp so we can spot misalignment. */
static void ramp(int16_t *buf, int frames, int base)
{
	for (int i = 0; i < frames; ++i) {
		buf[i * 2] = (int16_t)(base + i);
		buf[i * 2 + 1] = (int16_t)(-(base + i));
	}
}

static void test_silence_when_idle(void)
{
	int16_t out[SAMPLES];
	printf("test: no source -> silence\n");
	memset(out, 0x5a, sizeof(out));
	uac_mixer_start();
	uac_mixer_fill(out);
	int ok = 1;
	for (int i = 0; i < SAMPLES; ++i)
		if (out[i] != 0)
			ok = 0;
	check(ok, "output fully zeroed");
	uac_mixer_stop();
}

static void test_single_source_passthrough(void)
{
	int16_t in[FRAMES * 2], out[SAMPLES];
	printf("test: single 48k stereo source passes through bit-exact\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1 /* STEREO */, 1);
	ramp(in, FRAMES, 100);
	uac_mixer_source_push(0, 1, in, FRAMES, 1);
	uac_mixer_fill(out);
	check(memcmp(in, out, sizeof(in)) == 0, "output == input");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static void test_partial_fill_zero_tail(void)
{
	int16_t in[10 * 2], out[SAMPLES];
	printf("test: short source zero-pads the rest of the packet\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	ramp(in, 10, 7);
	uac_mixer_source_push(0, 1, in, 10, 1);
	memset(out, 0x5a, sizeof(out));
	uac_mixer_fill(out);
	check(memcmp(in, out, sizeof(in)) == 0, "first 10 frames copied");
	int ok = 1;
	for (int i = 10 * 2; i < SAMPLES; ++i)
		if (out[i] != 0)
			ok = 0;
	check(ok, "frames 10..47 are silence");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static void test_two_source_mix(void)
{
	int16_t a[FRAMES * 2], b[FRAMES * 2], out[SAMPLES];
	printf("test: two sources sum\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	uac_mixer_source_open(1, 48000, 1, 1);
	for (int i = 0; i < SAMPLES; ++i) {
		a[i] = 1000;
		b[i] = 337;
	}
	uac_mixer_source_push(0, 1, a, FRAMES, 1);
	uac_mixer_source_push(1, 1, b, FRAMES, 1);
	uac_mixer_fill(out);
	int ok = 1;
	for (int i = 0; i < SAMPLES; ++i)
		if (out[i] != 1337)
			ok = 0;
	check(ok, "every sample == 1000 + 337");
	uac_mixer_source_close(0);
	uac_mixer_source_close(1);
	uac_mixer_stop();
}

static void test_saturation(void)
{
	int16_t a[FRAMES * 2], b[FRAMES * 2], out[SAMPLES];
	printf("test: mix saturates instead of wrapping\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	uac_mixer_source_open(1, 48000, 1, 1);
	for (int i = 0; i < SAMPLES; ++i) {
		a[i] = (i & 1) ? -30000 : 30000;
		b[i] = (i & 1) ? -30000 : 30000;
	}
	uac_mixer_source_push(0, 1, a, FRAMES, 1);
	uac_mixer_source_push(1, 1, b, FRAMES, 1);
	uac_mixer_fill(out);
	int ok = 1;
	for (int i = 0; i < SAMPLES; ++i) {
		int16_t want = (i & 1) ? -32768 : 32767;
		if (out[i] != want)
			ok = 0;
	}
	check(ok, "clamped to +32767 / -32768, no wraparound");
	uac_mixer_source_close(0);
	uac_mixer_source_close(1);
	uac_mixer_stop();
}

/* Push/drain many packets so write_cursor crosses the 4096-frame ring
 * boundary repeatedly and the two-run split gets exercised. */
static void test_ring_wrap(void)
{
	int16_t in[FRAMES * 2], out[SAMPLES];
	printf("test: ring wraparound stays bit-exact over 500 packets\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	int ok = 1;
	for (int round = 0; round < 500; ++round) {
		ramp(in, FRAMES, round * 3);
		uac_mixer_source_push(0, 1, in, FRAMES, 1);
		uac_mixer_fill(out);
		if (memcmp(in, out, sizeof(in)) != 0) {
			printf("    mismatch at round %d\n", round);
			ok = 0;
			break;
		}
	}
	check(ok, "500 push/fill rounds bit-exact across wrap");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

/* The bug this guards: draining used to zero read/write, so a reader that had
 * already sampled read_frame saw (write - read) underflow to ~4 billion. */
static void test_drain_no_underflow(void)
{
	int16_t in[FRAMES * 2], out[SAMPLES];
	printf("test: reopen mid-stream does not replay the whole ring\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	for (int round = 0; round < 40; ++round) {
		ramp(in, FRAMES, round);
		uac_mixer_source_push(0, 1, in, FRAMES, 1);
	}
	/* Reopen (what a rate change does) then drain one packet. */
	uac_mixer_source_open(0, 48000, 1, 1);
	memset(out, 0x5a, sizeof(out));
	uac_mixer_fill(out);
	int ok = 1;
	for (int i = 0; i < SAMPLES; ++i)
		if (out[i] != 0)
			ok = 0;
	check(ok, "post-reopen packet is silence, not stale ring data");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static void test_mono_upmix(void)
{
	int16_t in[FRAMES], out[SAMPLES];
	printf("test: mono source duplicates to both channels\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 0 /* MONO */, 1);
	for (int i = 0; i < FRAMES; ++i)
		in[i] = (int16_t)(500 + i);
	uac_mixer_source_push(0, 1, in, FRAMES, 1);
	uac_mixer_fill(out);
	int ok = 1;
	for (int i = 0; i < FRAMES; ++i)
		if (out[i * 2] != 500 + i || out[i * 2 + 1] != 500 + i)
			ok = 0;
	check(ok, "L == R == mono sample");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

/* 44.1k -> 48k must produce ~48/44.1 as many frames and stay in range. */
static void test_resample_rate(void)
{
	static int16_t in[441 * 2];
	int16_t out[SAMPLES];
	printf("test: 44.1k -> 48k resampler output rate and range\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 44100, 1, 1);
	for (int i = 0; i < 441; ++i) {
		in[i * 2] = (int16_t)(8000);
		in[i * 2 + 1] = (int16_t)(-8000);
	}
	uac_mixer_source_push(0, 1, in, 441, 1);
	int produced = 0, ok = 1;
	for (int packet = 0; packet < 20; ++packet) {
		uac_mixer_fill(out);
		for (int i = 0; i < FRAMES; ++i) {
			if (out[i * 2] == 0 && out[i * 2 + 1] == 0)
				continue;
			produced++;
			if (out[i * 2] < 7000 || out[i * 2] > 9000)
				ok = 0;
			if (out[i * 2 + 1] > -7000 || out[i * 2 + 1] < -9000)
				ok = 0;
		}
	}
	/* 441 input frames at 44.1k == 10 ms == ~480 output frames at 48k. */
	printf("    produced %d frames (expect ~480)\n", produced);
	check(produced > 455 && produced < 485, "output frame count ~= 480");
	check(ok, "all samples near the input amplitude (no clicks/garbage)");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

/* An 8 kHz source expands 6x; make sure a big grain does not corrupt the
 * ring even if it has to drop. */
static void test_upsample_burst_no_corruption(void)
{
	static int16_t in[1024 * 2];
	int16_t out[SAMPLES];
	printf("test: 8k grain-1024 burst (6x expansion) degrades cleanly\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 8000, 1, 1);
	for (int i = 0; i < 1024; ++i) {
		in[i * 2] = 12000;
		in[i * 2 + 1] = 12000;
	}
	uac_mixer_source_push(0, 1, in, 1024, 1);
	int ok = 1;
	for (int packet = 0; packet < 200; ++packet) {
		uac_mixer_fill(out);
		for (int i = 0; i < SAMPLES; ++i)
			if (out[i] != 0 && (out[i] < 11000 || out[i] > 13000))
				ok = 0;
	}
	check(ok, "no garbage samples after ring overrun");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static void test_inactive_push_ignored(void)
{
	int16_t in[FRAMES * 2], out[SAMPLES];
	printf("test: push while stopped is ignored\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 1);
	uac_mixer_stop();
	ramp(in, FRAMES, 42);
	uac_mixer_source_push(0, 1, in, FRAMES, 1);
	uac_mixer_start();
	uac_mixer_fill(out);
	int ok = 1;
	for (int i = 0; i < SAMPLES; ++i)
		if (out[i] != 0)
			ok = 0;
	check(ok, "nothing buffered while inactive");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static int expected_resampled_frames(uint32_t rate, uint32_t input_frames)
{
	uint32_t input_count = input_frames;
	uint32_t input_index = 0;
	uint32_t phase = 0;
	int outputs = 0;

	while ((uint32_t)(input_count - input_index) > 12u) {
		outputs++;
		phase += rate;
		if (phase >= 48000u) {
			phase -= 48000u;
			input_index++;
		}
	}
	return outputs;
}

static void test_all_resample_rates(void)
{
	static const uint32_t rates[] = {
		8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100
	};
	static int16_t in[441 * 2];
	int16_t out[SAMPLES];
	int ok = 1;
	uint32_t rate_index;

	printf("test: every supported source rate has exact DC gain and cadence\n");
	for (rate_index = 0; rate_index < sizeof(rates) / sizeof(rates[0]);
		++rate_index) {
		uint32_t rate = rates[rate_index];
		uint32_t input_frames = rate / 100u;
		int expected = expected_resampled_frames(rate, input_frames);
		int produced = 0;
		int packet;
		uint32_t frame;

		for (frame = 0; frame < input_frames; ++frame) {
			in[frame * 2u] = 6000;
			in[frame * 2u + 1u] = -6000;
		}
		uac_mixer_start();
		uac_mixer_source_open(0, rate, 1, rate_index + 1u);
		uac_mixer_source_push(0, 1, in, input_frames,
			rate_index + 1u);
		for (packet = 0; packet < 12; ++packet) {
			uac_mixer_fill(out);
			for (frame = 0; frame < FRAMES; ++frame) {
				if (out[frame * 2u] == 0 &&
					out[frame * 2u + 1u] == 0)
					continue;
				produced++;
				if (out[frame * 2u] < 5980 ||
					out[frame * 2u] > 6020 ||
					out[frame * 2u + 1u] < -6020 ||
					out[frame * 2u + 1u] > -5980)
					ok = 0;
			}
		}
		if (produced != expected) {
			printf("    %u Hz produced %d, expected %d\n",
				rate, produced, expected);
			ok = 0;
		}
		uac_mixer_source_close(0);
		uac_mixer_stop();
	}
	check(ok, "8-44.1 kHz cadence and constant-signal gain are correct");
}

static void collect_resampled(const int16_t *input, int frames,
	const int *chunks, int chunk_count, int16_t *output, int output_frames)
{
	int consumed = 0;
	int chunk = 0;
	int written = 0;
	int16_t packet[SAMPLES];

	uac_mixer_start();
	uac_mixer_source_open(0, 44100, 1, 100);
	while (consumed < frames) {
		int take = chunks == NULL ? frames : chunks[chunk++ % chunk_count];
		if (take > frames - consumed)
			take = frames - consumed;
		uac_mixer_source_push(0, 1, input + consumed * 2, take, 100);
		consumed += take;
	}
	while (written < output_frames) {
		int take = output_frames - written;
		if (take > FRAMES)
			take = FRAMES;
		uac_mixer_fill(packet);
		memcpy(output + written * 2, packet, take * 4u);
		written += take;
	}
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static void test_resample_chunk_continuity(void)
{
	static int16_t input[441 * 2];
	static int16_t whole[480 * 2];
	static int16_t split[480 * 2];
	static const int chunks[] = { 1, 7, 31, 2, 64, 5, 113, 19 };
	int frame;

	printf("test: resampler state is independent of input buffer boundaries\n");
	for (frame = 0; frame < 441; ++frame) {
		input[frame * 2] = (int16_t)((frame * 7919) & 0x7fff);
		input[frame * 2 + 1] = (int16_t)-input[frame * 2];
	}
	collect_resampled(input, 441, NULL, 0, whole, 480);
	collect_resampled(input, 441, chunks,
		sizeof(chunks) / sizeof(chunks[0]), split, 480);
	check(memcmp(whole, split, sizeof(whole)) == 0,
		"arbitrary buffer splits are bit-identical");
}

static double resampler_tone_snr(uint32_t rate, int frequency)
{
	static int16_t input[480 * 2];
	static int16_t output[3840 * 2];
	int16_t packet[SAMPLES];
	double sine_sum = 0.0;
	double cosine_sum = 0.0;
	double sine_square = 0.0;
	double cosine_square = 0.0;
	double cross = 0.0;
	double sine_amplitude;
	double cosine_amplitude;
	double signal_power = 0.0;
	double noise_power = 0.0;
	int output_frame = 0;
	int block;
	int frame;
	const int first = 480;
	const int final = 3840;
	const int input_block = rate / 100u;

	uac_mixer_start();
	uac_mixer_source_open(0, rate, 1, 200);
	for (block = 0; block < 10; ++block) {
		for (frame = 0; frame < input_block; ++frame) {
			double angle = 2.0 * PI * frequency *
				(block * input_block + frame) / rate;
			int16_t sample = (int16_t)lrint(12000.0 * sin(angle));
			input[frame * 2] = sample;
			input[frame * 2 + 1] = sample;
		}
		uac_mixer_source_push(0, 1, input, input_block, 200);
		for (int packet_index = 0; packet_index < 8; ++packet_index) {
			uac_mixer_fill(packet);
			memcpy(output + output_frame * 2, packet, sizeof(packet));
			output_frame += FRAMES;
		}
	}
	for (frame = first; frame < final; ++frame) {
		double angle = 2.0 * PI * frequency * frame / 48000.0;
		double sample = output[frame * 2];
		double sine = sin(angle);
		double cosine = cos(angle);
		sine_sum += sample * sine;
		cosine_sum += sample * cosine;
		sine_square += sine * sine;
		cosine_square += cosine * cosine;
		cross += sine * cosine;
	}
	{
		double determinant = sine_square * cosine_square - cross * cross;
		sine_amplitude = (sine_sum * cosine_square -
			cosine_sum * cross) / determinant;
		cosine_amplitude = (cosine_sum * sine_square -
			sine_sum * cross) / determinant;
	}
	for (frame = first; frame < final; ++frame) {
		double angle = 2.0 * PI * frequency * frame / 48000.0;
		double expected = sine_amplitude * sin(angle) +
			cosine_amplitude * cos(angle);
		double error = output[frame * 2] - expected;
		signal_power += expected * expected;
		noise_power += error * error;
	}
	uac_mixer_source_close(0);
	uac_mixer_stop();
	return 10.0 * log10(signal_power / noise_power);
}

static void test_resampler_spectral_quality(void)
{
	static const uint32_t rates[] = {
		8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100
	};
	double snr_10k;
	double snr_18k;
	int all_rates_ok = 1;
	uint32_t index;

	printf("test: 44.1k -> 48k FIR suppresses interpolation hiss\n");
	snr_10k = resampler_tone_snr(44100, 10000);
	snr_18k = resampler_tone_snr(44100, 18000);
	printf("    residual SNR: 10 kHz %.1f dB, 18 kHz %.1f dB\n",
		snr_10k, snr_18k);
	check(snr_10k > 50.0, "10 kHz residual is below -50 dB");
	check(snr_18k > 38.0, "18 kHz residual is below -38 dB");
	printf("test: every resampled rate suppresses near-Nyquist images\n");
	for (index = 0; index < sizeof(rates) / sizeof(rates[0]); ++index) {
		int frequency = (int)(rates[index] * 2u / 5u);
		double snr = resampler_tone_snr(rates[index], frequency);

		printf("    %5u Hz source, %5d Hz tone: %.1f dB\n",
			rates[index], frequency, snr);
		if (snr < 35.0)
			all_rates_ok = 0;
	}
	check(all_rates_ok, "all non-native rates exceed 35 dB at 80% Nyquist");
}

static void test_stale_generation_ignored(void)
{
	int16_t in[FRAMES * 2], out[SAMPLES];
	int ok = 1;
	int i;

	printf("test: stale port generation cannot write a reused slot\n");
	uac_mixer_start();
	uac_mixer_source_open(0, 48000, 1, 2);
	ramp(in, FRAMES, 42);
	uac_mixer_source_push(0, 1, in, FRAMES, 1);
	uac_mixer_fill(out);
	for (i = 0; i < SAMPLES; ++i)
		if (out[i] != 0)
			ok = 0;
	check(ok, "old snapshot rejected after slot reuse");
	uac_mixer_source_close(0);
	uac_mixer_stop();
}

static void test_out_of_range_slot(void)
{
	int16_t in[FRAMES * 2];
	printf("test: out-of-range slot indices are rejected\n");
	uac_mixer_start();
	ramp(in, FRAMES, 1);
	uac_mixer_source_open(-1, 48000, 1, 1);
	uac_mixer_source_open(99, 48000, 1, 1);
	uac_mixer_source_push(-1, 1, in, FRAMES, 1);
	uac_mixer_source_push(99, 1, in, FRAMES, 1);
	uac_mixer_source_close(-1);
	uac_mixer_source_close(99);
	check(1, "no crash on bad slot index");
	uac_mixer_stop();
}

int main(void)
{
	test_silence_when_idle();
	test_single_source_passthrough();
	test_partial_fill_zero_tail();
	test_two_source_mix();
	test_saturation();
	test_ring_wrap();
	test_drain_no_underflow();
	test_mono_upmix();
	test_resample_rate();
	test_all_resample_rates();
	test_resample_chunk_continuity();
	test_resampler_spectral_quality();
	test_upsample_burst_no_corruption();
	test_inactive_push_ignored();
	test_stale_generation_ignored();
	test_out_of_range_slot();

	printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASS",
		failures, failures == 1 ? "" : "s");
	return failures != 0;
}
