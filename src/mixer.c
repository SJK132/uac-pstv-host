#include "mixer.h"
#include "log.h"
#include "resampler_coeffs.h"

#include <psp2/audioout.h>
#include <psp2kern/kernel/sysmem/data_transfers.h>
#include <string.h>

#define LOG_PREFIX "[uac-pstv-audio] "

/* One uac_mixer_fill() produces exactly one UAC1 OUT packet: 192 bytes =
 * 48 stereo 16-bit frames = 1 ms at 48 kHz. */
#define MIXER_FRAME_BYTES 192
#define MIXER_FRAMES (MIXER_FRAME_BYTES / 4)


/* Per-source ring, counted in 48 kHz output frames, power of two so the wrap
 * is a mask. sceAudioOutOutput blocks until the previous buffer drains, so a
 * port delivers one grain per grain-period and the ring only has to absorb
 * that burst: grain * 48000/rate output frames after resampling. 4096 covers
 * the worst realistic case (grain 1024 down to 12 kHz) and costs 16 KiB per
 * slot. Dropping it to 2048 halves that to 64 KiB total and still covers
 * every source at 24 kHz or above, which is all a PSV title realistically
 * opens; it is a one-line change if kernel memory matters more than the
 * exotic low-rate cases. */
#define AUDIO_SOURCE_FRAMES 4096

/* Staging buffer for the mono / non-48 kHz path. Only needs to be large
 * enough to amortise the ksceKernelCopyFromUserProc call overhead. */
#define AUDIO_COPY_FRAMES 128

typedef struct {
	/* Shared with the lock-free consumer. read_frame and write_frame are
	 * free-running counters, never reset backwards -- see source_drain. */
	uint32_t read_frame;
	uint32_t write_frame;
	uint8_t enabled;

	/* Producer-private: only touched while holding producer_lock. */
	uint32_t write_cursor;
	uint32_t write_room;
	uint32_t rate;
	uint32_t phase;
	uint32_t resample_index;
	uint32_t input_count;
#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t dropped_frames;
#endif
	uint32_t generation;
	int mode;
	uint8_t history_primed;

	int producer_lock;
} __attribute__((aligned(64))) AudioSource;

static int mixer_active;
static AudioSource audio_sources[UAC_AUDIO_SOURCE_SLOTS];
static int16_t audio_pcm[UAC_AUDIO_SOURCE_SLOTS][AUDIO_SOURCE_FRAMES * 2]
	__attribute__((aligned(64)));
static int16_t input_scratch[UAC_AUDIO_SOURCE_SLOTS][AUDIO_COPY_FRAMES * 2]
	__attribute__((aligned(64)));
static int16_t resample_history[UAC_AUDIO_SOURCE_SLOTS]
	[UAC_RESAMPLER_HISTORY * 2] __attribute__((aligned(64)));
static int32_t mix_accumulator[MIXER_FRAMES * 2] __attribute__((aligned(64)));

#ifdef UAC_MIXER_TEST
void (*uac_mixer_test_before_commit)(void);
#endif

/* Non-blocking: both the producer (app thread) and the open/close paths run
 * where sleeping is not an option, so a contended source simply skips a
 * round. Acquire/release rather than __sync_*, which emits a full barrier. */
static int try_source(AudioSource *source)
{
	return __atomic_exchange_n(&source->producer_lock, 1,
		__ATOMIC_ACQUIRE) == 0;
}

static void unlock_source(AudioSource *source)
{
	__atomic_store_n(&source->producer_lock, 0, __ATOMIC_RELEASE);
}

/* Drop everything queued without moving the counters backwards. Zeroing them
 * would race the lock-free consumer: it samples read_frame and write_frame
 * separately, so a reset between the two loads makes the unsigned difference
 * underflow into a huge "available" and replay the whole stale ring. */
static void source_drain(AudioSource *source)
{
	uint32_t write = __atomic_load_n(&source->write_frame, __ATOMIC_RELAXED);

	__atomic_store_n(&source->read_frame, write, __ATOMIC_RELEASE);
	source->write_cursor = write;
	source->write_room = AUDIO_SOURCE_FRAMES;
	source->phase = 0;
	source->resample_index = 0;
	source->input_count = 0;
	source->history_primed = 0;
}

static void source_refresh_room(AudioSource *source)
{
	uint32_t used = source->write_cursor -
		__atomic_load_n(&source->read_frame, __ATOMIC_ACQUIRE);

	source->write_room = used < AUDIO_SOURCE_FRAMES
		? AUDIO_SOURCE_FRAMES - used : 0;
}

/*
 * Consumer. Runs in the USB isochronous completion callback every 1 ms, so it
 * takes no locks and touches only the free-running counters.
 */

typedef struct {
	const int16_t *pcm;
	AudioSource *source;
	uint32_t read;
	uint32_t take;
} MixInput;

/* Split a ring read into the two contiguous runs it spans, so the copy and
 * accumulate loops below never mask per sample. */
static inline void run_split(uint32_t read, uint32_t take, uint32_t *start,
	uint32_t *first, uint32_t *second)
{
	uint32_t offset = read & (AUDIO_SOURCE_FRAMES - 1u);
	uint32_t contiguous = AUDIO_SOURCE_FRAMES - offset;

	*start = offset;
	*first = take < contiguous ? take : contiguous;
	*second = take - *first;
}

static uint32_t source_take(AudioSource *source, uint32_t *read_out)
{
	uint32_t read;
	uint32_t available;

	if (!__atomic_load_n(&source->enabled, __ATOMIC_RELAXED))
		return 0;
	read = __atomic_load_n(&source->read_frame, __ATOMIC_RELAXED);
	available = __atomic_load_n(&source->write_frame, __ATOMIC_RELAXED) -
		read;
	if (available == 0)
		return 0;
	/* Pairs with the release store of write_frame in uac_mixer_source_push:
	 * the PCM behind that counter must be visible before we read it. */
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	if (available > AUDIO_SOURCE_FRAMES)
		available = AUDIO_SOURCE_FRAMES;
	*read_out = read;
	return available < MIXER_FRAMES ? available : MIXER_FRAMES;
}

static void mix_store(int32_t *accumulator, const MixInput *input)
{
	uint32_t start;
	uint32_t first;
	uint32_t second;
	uint32_t index;
	const int16_t *pcm;

	run_split(input->read, input->take, &start, &first, &second);
	pcm = input->pcm + start * 2u;
	for (index = 0; index < first * 2u; ++index)
		accumulator[index] = pcm[index];
	pcm = input->pcm;
	for (index = 0; index < second * 2u; ++index)
		accumulator[first * 2u + index] = pcm[index];
}

static void mix_add(int32_t *accumulator, const MixInput *input)
{
	uint32_t start;
	uint32_t first;
	uint32_t second;
	uint32_t index;
	const int16_t *pcm;

	run_split(input->read, input->take, &start, &first, &second);
	pcm = input->pcm + start * 2u;
	for (index = 0; index < first * 2u; ++index)
		accumulator[index] += pcm[index];
	pcm = input->pcm;
	for (index = 0; index < second * 2u; ++index)
		accumulator[first * 2u + index] += pcm[index];
}

/* Single-source fast path: the ring already holds finished 16-bit stereo, so
 * it goes straight out with no accumulate and no saturation. */
static void mix_copy(int16_t *output, const MixInput *input)
{
	uint32_t start;
	uint32_t first;
	uint32_t second;

	run_split(input->read, input->take, &start, &first, &second);
	memcpy(output, input->pcm + start * 2u, first * 4u);
	if (second != 0)
		memcpy(output + first * 2u, input->pcm, second * 4u);
}

void uac_mixer_fill(int16_t *output)
{
	MixInput inputs[UAC_AUDIO_SOURCE_SLOTS];
	uint32_t count = 0;
	uint32_t longest = 0;
	uint32_t index;
	int drained = 0;

	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index) {
		AudioSource *source = &audio_sources[index];
		uint32_t read = 0;
		uint32_t take = source_take(source, &read);

		if (take == 0)
			continue;
		inputs[count].pcm = audio_pcm[index];
		inputs[count].source = source;
		inputs[count].read = read;
		inputs[count].take = take;
		if (take > longest)
			longest = take;
		++count;
	}

	if (count == 0) {
		memset(output, 0, MIXER_FRAME_BYTES);
		return;
	}

	if (count == 1) {
		mix_copy(output, &inputs[0]);
	} else {
		uint32_t sample;

		mix_store(mix_accumulator, &inputs[0]);
		for (sample = inputs[0].take * 2u; sample < longest * 2u; ++sample)
			mix_accumulator[sample] = 0;
		for (index = 1; index < count; ++index)
			mix_add(mix_accumulator, &inputs[index]);
		/* Compiles to SSAT on Cortex-A9. */
		for (sample = 0; sample < longest * 2u; ++sample) {
			int32_t mixed = mix_accumulator[sample];
			if (mixed > 32767)
				mixed = 32767;
			else if (mixed < -32768)
				mixed = -32768;
			output[sample] = (int16_t)mixed;
		}
	}

	/* Sources that ran short contribute silence for the remainder. */
	if (longest < MIXER_FRAMES)
		memset(output + longest * 2u, 0, (MIXER_FRAMES - longest) * 4u);

#ifdef UAC_MIXER_TEST
	if (uac_mixer_test_before_commit != NULL)
		uac_mixer_test_before_commit();
#endif

	for (index = 0; index < count; ++index) {
		uint32_t expected = inputs[index].read;

		/* Do not undo a close/reopen drain that advanced read_frame after
		 * source_take() sampled it. The drain wins when this CAS fails. */
		if (!__atomic_compare_exchange_n(&inputs[index].source->read_frame,
			&expected, inputs[index].read + inputs[index].take, 0,
			__ATOMIC_RELEASE, __ATOMIC_RELAXED))
			drained = 1;
	}
	/* A reopened producer may already be reusing its ring. Suppress this one
	 * transitional packet rather than sending a stale or partially replaced
	 * frame; successful sources remain consumed, so nothing is replayed. */
	if (drained)
		memset(output, 0, MIXER_FRAME_BYTES);
}

void uac_mixer_start(void)
{
	int index;

	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index) {
		AudioSource *source = &audio_sources[index];
		if (try_source(source)) {
			source_drain(source);
#ifdef UAC_PSTV_ENABLE_LOGGING
			source->dropped_frames = 0;
#endif
			unlock_source(source);
		}
	}
	__atomic_store_n(&mixer_active, 1, __ATOMIC_RELEASE);
	uac_log(LOG_PREFIX "mixer: %u sources x %u frames, %u frames/packet\n",
		UAC_AUDIO_SOURCE_SLOTS, AUDIO_SOURCE_FRAMES, MIXER_FRAMES);
}

void uac_mixer_stop(void)
{
#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t dropped = 0;
	int index;
#endif

	__atomic_store_n(&mixer_active, 0, __ATOMIC_RELEASE);
#ifdef UAC_PSTV_ENABLE_LOGGING
	for (index = 0; index < UAC_AUDIO_SOURCE_SLOTS; ++index)
		dropped += audio_sources[index].dropped_frames;
	if (dropped != 0)
		uac_log(LOG_PREFIX "mixer: %u frames dropped (ring overrun)\n",
			dropped);
#endif
}

/*
 * Producer.
 */

static int supported_rate(uint32_t rate)
{
	return rate == 8000 || rate == 11025 || rate == 12000 || rate == 16000 ||
		rate == 22050 || rate == 24000 || rate == 32000 ||
		rate == 44100 || rate == 48000;
}

int uac_mixer_source_open(int source, uint32_t rate, int mode,
	uint32_t generation)
{
	AudioSource *state;

	if (source < 0 || source >= UAC_AUDIO_SOURCE_SLOTS)
		return -1;
	state = &audio_sources[source];
	/* Stop the consumer before trying to drain. If a producer owns the
	 * slot, the caller leaves its port unpublished and retries later. */
	__atomic_store_n(&state->enabled, 0, __ATOMIC_RELEASE);
	if (!try_source(state))
		return -1;
	source_drain(state);
	state->rate = rate;
	state->mode = mode;
	state->generation = generation;
	__atomic_store_n(&state->enabled, supported_rate(rate) &&
		(mode == SCE_AUDIO_OUT_MODE_MONO ||
		 mode == SCE_AUDIO_OUT_MODE_STEREO), __ATOMIC_RELEASE);
	unlock_source(state);
	return 0;
}

void uac_mixer_source_close(int source)
{
	AudioSource *state;

	if (source < 0 || source >= UAC_AUDIO_SOURCE_SLOTS)
		return;
	state = &audio_sources[source];
	__atomic_store_n(&state->enabled, 0, __ATOMIC_RELEASE);
	if (!try_source(state))
		return;
	source_drain(state);
	unlock_source(state);
}

static inline void source_write_frame(AudioSource *source, int16_t *pcm,
	int16_t left, int16_t right)
{
	uint32_t ring_frame;

	if (source->write_room == 0) {
		/* The consumer may have drained since we last looked. */
		source_refresh_room(source);
		if (source->write_room == 0) {
#ifdef UAC_PSTV_ENABLE_LOGGING
			source->dropped_frames++;
#endif
			return;
		}
	}
	ring_frame = source->write_cursor & (AUDIO_SOURCE_FRAMES - 1u);
	pcm[ring_frame * 2u] = left;
	pcm[ring_frame * 2u + 1u] = right;
	source->write_cursor++;
	source->write_room--;
}

static uint32_t source_copy_native(AudioSource *source, int16_t *pcm,
	SceUID pid, const int16_t *input, uint32_t frames)
{
	uint32_t copied = 0;

	while (copied < frames) {
		uint32_t ring_frame = source->write_cursor &
			(AUDIO_SOURCE_FRAMES - 1u);
		uint32_t contiguous = AUDIO_SOURCE_FRAMES - ring_frame;
		uint32_t chunk = frames - copied;

		if (chunk > contiguous)
			chunk = contiguous;
		if (chunk > source->write_room) {
			source_refresh_room(source);
			if (chunk > source->write_room)
				chunk = source->write_room;
		}
		if (chunk == 0) {
#ifdef UAC_PSTV_ENABLE_LOGGING
			source->dropped_frames += frames - copied;
#endif
			return frames;
		}
		if (ksceKernelCopyFromUserProc(pid, pcm + ring_frame * 2u,
			input + copied * 2u, chunk * 4u) < 0)
			break;
		source->write_cursor += chunk;
		source->write_room -= chunk;
		copied += chunk;
	}
	return copied;
}

static inline const int16_t *resampler_row(uint32_t phase, int *step)
{
	if (phase <= UAC_RESAMPLER_PHASES / 2u) {
		*step = 1;
		return &uac_resampler_coefficients[phase][0];
	}
	*step = -1;
	return &uac_resampler_coefficients[UAC_RESAMPLER_PHASES - phase]
		[UAC_RESAMPLER_TAPS - 1];
}

static inline int16_t resampler_round(int32_t value)
{
	int32_t sample;

	if (value >= 0)
		sample = (value + (1 << (UAC_RESAMPLER_SHIFT - 1))) >>
			UAC_RESAMPLER_SHIFT;
	else
		sample = -((-value + (1 << (UAC_RESAMPLER_SHIFT - 1))) >>
			UAC_RESAMPLER_SHIFT);
	if (sample > 32767)
		sample = 32767;
	else if (sample < -32768)
		sample = -32768;
	return (int16_t)sample;
}

static void source_resampler_output(AudioSource *source, int16_t *pcm,
	const int16_t *history)
{
	uint32_t scaled = source->phase * UAC_RESAMPLER_PHASES;
	uint32_t coefficient_phase = scaled / 48000u;
	uint32_t remainder = scaled - coefficient_phase * 48000u;
	const int16_t *row0;
	const int16_t *row1 = NULL;
	uint32_t blend = 0;
	int step0;
	int step1 = 0;
	int32_t left = 0;
	int32_t right = 0;
	uint32_t tap;

	if (source->write_room == 0) {
		source_refresh_room(source);
		if (source->write_room == 0) {
#ifdef UAC_PSTV_ENABLE_LOGGING
			source->dropped_frames++;
#endif
			goto advance;
		}
	}
	row0 = resampler_row(coefficient_phase, &step0);
	if (remainder != 0) {
		row1 = resampler_row(coefficient_phase + 1u, &step1);
		blend = (remainder * 65536u + 24000u) / 48000u;
	}
	for (tap = 0; tap < UAC_RESAMPLER_TAPS; ++tap) {
		uint32_t frame = (source->resample_index + tap -
			UAC_RESAMPLER_LEFT) & (UAC_RESAMPLER_HISTORY - 1u);
		int32_t coefficient = row0[(int)tap * step0];

		if (remainder != 0) {
			int32_t next = row1[(int)tap * step1];
			coefficient += ((next - coefficient) * (int32_t)blend) >> 16;
		}
		left += history[frame * 2u] * coefficient;
		right += history[frame * 2u + 1u] * coefficient;
	}
	source_write_frame(source, pcm, resampler_round(left),
		resampler_round(right));

advance:
	source->phase += source->rate;
	if (source->phase >= 48000u) {
		source->phase -= 48000u;
		source->resample_index++;
	}
}

static inline void source_resample_frame(AudioSource *source, int16_t *pcm,
	int16_t *history, int16_t left, int16_t right)
{
	uint32_t frame;

	if (!source->history_primed) {
		for (frame = 0; frame < UAC_RESAMPLER_HISTORY; ++frame) {
			history[frame * 2u] = left;
			history[frame * 2u + 1u] = right;
		}
		source->history_primed = 1;
		source->input_count = 1;
		return;
	}
	frame = source->input_count & (UAC_RESAMPLER_HISTORY - 1u);
	history[frame * 2u] = left;
	history[frame * 2u + 1u] = right;
	source->input_count++;
	while ((uint32_t)(source->input_count - source->resample_index) >
		UAC_RESAMPLER_RIGHT)
		source_resampler_output(source, pcm, history);
}

void uac_mixer_source_push(int source, int pid, const void *buffer,
	uint32_t frames, uint32_t generation)
{
	AudioSource *state;
	int16_t *pcm;
	int16_t *history;
	uint32_t rate;
	uint32_t channels;
	uint32_t copied = 0;
	int16_t *scratch;

	if (source < 0 || source >= UAC_AUDIO_SOURCE_SLOTS || buffer == NULL ||
		frames == 0 || !__atomic_load_n(&mixer_active, __ATOMIC_ACQUIRE))
		return;
	state = &audio_sources[source];
	pcm = audio_pcm[source];
	history = resample_history[source];
	scratch = input_scratch[source];
	if (!try_source(state))
		return;
	if (!__atomic_load_n(&state->enabled, __ATOMIC_RELAXED) ||
		state->generation != generation)
		goto done;
	rate = state->rate;
	channels = state->mode == SCE_AUDIO_OUT_MODE_MONO ? 1u : 2u;
	state->write_cursor = __atomic_load_n(&state->write_frame,
		__ATOMIC_RELAXED);
	source_refresh_room(state);

	if (rate == 48000u && channels == 2u) {
		/* Already in output format: straight into the ring. */
		copied = source_copy_native(state, pcm, pid, buffer, frames);
	} else while (copied < frames) {
		uint32_t chunk = frames - copied;
		uint32_t frame;

		if (chunk > AUDIO_COPY_FRAMES)
			chunk = AUDIO_COPY_FRAMES;
		if (ksceKernelCopyFromUserProc(pid, scratch,
			(const int16_t *)buffer + copied * channels,
			chunk * channels * sizeof(scratch[0])) < 0)
			break;
		if (rate == 48000u)
			for (frame = 0; frame < chunk; ++frame)
				source_write_frame(state, pcm, scratch[frame],
					scratch[frame]);
		else
			for (frame = 0; frame < chunk; ++frame) {
				int16_t left = scratch[frame * channels];
				int16_t right = channels == 1u
					? left : scratch[frame * channels + 1u];
				source_resample_frame(state, pcm, history, left, right);
			}
		copied += chunk;
	}
	__atomic_store_n(&state->write_frame, state->write_cursor,
		__ATOMIC_RELEASE);
done:
	unlock_source(state);
}
