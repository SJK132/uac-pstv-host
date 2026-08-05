/* Hot-path A/B: the committed fill algorithm vs the rewritten one.
 * Both operate on identical ring data; only the algorithm differs. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define SLOTS 8
#define RING 4096
#define FRAMES 48
#define BYTES 192

static int16_t pcm[SLOTS][RING * 2];
static int32_t acc[FRAMES * 2];
static uint32_t rd[SLOTS], wr[SLOTS];
static uint8_t en[SLOTS];

/* ---- OLD: always memset the accumulator, mask every frame, clamp all 96 ---- */
static void fill_old(int16_t *out)
{
	memset(acc, 0, sizeof(acc));
	for (uint32_t s = 0; s < SLOTS; ++s) {
		if (!en[s])
			continue;
		uint32_t read = rd[s];
		uint32_t avail = wr[s] - read;
		if (avail == 0)
			continue;
		if (avail > RING)
			avail = RING;
		uint32_t take = avail < FRAMES ? avail : FRAMES;
		for (uint32_t f = 0; f < take; ++f) {
			uint32_t rf = (read + f) & (RING - 1u);
			acc[f * 2] += pcm[s][rf * 2];
			acc[f * 2 + 1] += pcm[s][rf * 2 + 1];
		}
		rd[s] = read + take;
	}
	for (uint32_t i = 0; i < FRAMES * 2u; ++i) {
		int32_t m = acc[i];
		if (m > 32767) m = 32767;
		else if (m < -32768) m = -32768;
		out[i] = (int16_t)m;
	}
}

/* ---- NEW: fast paths + contiguous runs (mirrors src/mixer.c) ---- */
static inline void split(uint32_t read, uint32_t take, uint32_t *st,
	uint32_t *a, uint32_t *b)
{
	uint32_t o = read & (RING - 1u);
	uint32_t c = RING - o;
	*st = o; *a = take < c ? take : c; *b = take - *a;
}

static void fill_new(int16_t *out)
{
	struct { const int16_t *p; uint32_t s; uint32_t r; uint32_t t; } in[SLOTS];
	uint32_t n = 0, longest = 0;

	for (uint32_t s = 0; s < SLOTS; ++s) {
		if (!en[s]) continue;
		uint32_t read = rd[s];
		uint32_t avail = wr[s] - read;
		if (avail == 0) continue;
		if (avail > RING) avail = RING;
		uint32_t take = avail < FRAMES ? avail : FRAMES;
		in[n].p = pcm[s]; in[n].s = s; in[n].r = read; in[n].t = take;
		if (take > longest) longest = take;
		++n;
	}
	if (n == 0) { memset(out, 0, BYTES); return; }

	if (n == 1) {
		uint32_t st, a, b;
		split(in[0].r, in[0].t, &st, &a, &b);
		memcpy(out, in[0].p + st * 2u, a * 4u);
		if (b) memcpy(out + a * 2u, in[0].p, b * 4u);
	} else {
		uint32_t st, a, b, i;
		split(in[0].r, in[0].t, &st, &a, &b);
		const int16_t *p = in[0].p + st * 2u;
		for (i = 0; i < a * 2u; ++i) acc[i] = p[i];
		p = in[0].p;
		for (i = 0; i < b * 2u; ++i) acc[a * 2u + i] = p[i];
		for (i = in[0].t * 2u; i < longest * 2u; ++i) acc[i] = 0;
		for (uint32_t k = 1; k < n; ++k) {
			split(in[k].r, in[k].t, &st, &a, &b);
			p = in[k].p + st * 2u;
			for (i = 0; i < a * 2u; ++i) acc[i] += p[i];
			p = in[k].p;
			for (i = 0; i < b * 2u; ++i) acc[a * 2u + i] += p[i];
		}
		for (i = 0; i < longest * 2u; ++i) {
			int32_t m = acc[i];
			if (m > 32767) m = 32767;
			else if (m < -32768) m = -32768;
			out[i] = (int16_t)m;
		}
	}
	if (longest < FRAMES)
		memset(out + longest * 2u, 0, (FRAMES - longest) * 4u);
	for (uint32_t k = 0; k < n; ++k) rd[in[k].s] = in[k].r + in[k].t;
}

static double bench(void (*fn)(int16_t *), int active, long iters)
{
	int16_t out[FRAMES * 2];
	struct timespec t0, t1;

	for (int s = 0; s < SLOTS; ++s) {
		en[s] = s < active;
		rd[s] = wr[s] = 0;
		for (int i = 0; i < RING * 2; ++i)
			pcm[s][i] = (int16_t)(i * 7 + s);
	}
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (long i = 0; i < iters; ++i) {
		for (int s = 0; s < active; ++s)
			wr[s] += FRAMES;          /* keep a packet available */
		fn(out);
	}
	clock_gettime(CLOCK_MONOTONIC, &t1);
	return ((t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec)) / iters;
}

int main(void)
{
	const long N = 2000000;
	printf("  sources |    old ns |    new ns | speedup\n");
	printf("  --------+-----------+-----------+--------\n");
	for (int a = 0; a <= 3; ++a) {
		double o = bench(fill_old, a, N);
		double x = bench(fill_new, a, N);
		printf("  %7d | %9.1f | %9.1f | %5.2fx\n", a, o, x, o / x);
	}
	double o8 = bench(fill_old, 8, N), x8 = bench(fill_new, 8, N);
	printf("  %7d | %9.1f | %9.1f | %5.2fx\n", 8, o8, x8, o8 / x8);
	return 0;
}
