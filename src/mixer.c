#include "mixer.h"
#include "log.h"
#include "resampler_coeffs.h"

#include <psp2/audioout.h>
#include <psp2kern/kernel/sysmem/data_transfers.h>
#include <string.h>

#define LOG_PREFIX "[uac-pstv-audio] "

#define MIXER_RATE 48000u
#define MIXER_FRAMES 48u
#define MIXER_BYTES (MIXER_FRAMES * 4u)
#define SOURCE_FRAMES 4096u
#define SOURCE_MASK (SOURCE_FRAMES - 1u)
#define COPY_FRAMES 128u
#define MIN_SOURCE_RATE 8000u

/* fill_lock guarantees at most one reader, so two bits are sufficient. */
#define READER_ACTIVE 0x01u
#define READER_RESETTING 0x02u

#if (SOURCE_FRAMES & SOURCE_MASK) != 0
#error SOURCE_FRAMES must be a power of two
#endif
#if (UAC_RESAMPLER_HISTORY & (UAC_RESAMPLER_HISTORY - 1u)) != 0
#error UAC_RESAMPLER_HISTORY must be a power of two
#endif

typedef struct {
	uint32_t read_frame;
	uint32_t write_frame;
	uint32_t reader_guard;
	uint8_t enabled;

	/* Protected by producer_lock. */
	uint32_t write_cursor;
	uint32_t write_room;
	uint32_t rate;
	uint32_t phase;
	uint32_t resample_index;
	uint32_t input_count;
	uint32_t generation;
	uint32_t desired_generation;
	uint32_t epoch;
	int mode;
	int producer_lock;
	uint8_t history_primed;

#ifdef UAC_PSTV_ENABLE_LOGGING
	uint32_t dropped_frames;
	uint32_t copy_errors;
	uint32_t busy_buffers;
#endif
} __attribute__((aligned(64))) AudioSource;

typedef struct {
	AudioSource *source;
	const int16_t *pcm;
	uint32_t read;
	uint32_t take;
} MixInput;

static int mixer_active;
static int fill_lock;
static uint32_t mixer_epoch;
static AudioSource sources[UAC_AUDIO_SOURCE_SLOTS];
static int16_t source_pcm[UAC_AUDIO_SOURCE_SLOTS][SOURCE_FRAMES * 2u]
	__attribute__((aligned(64)));
static int16_t copy_scratch[UAC_AUDIO_SOURCE_SLOTS][COPY_FRAMES * 2u]
	__attribute__((aligned(64)));
static int16_t resample_history[UAC_AUDIO_SOURCE_SLOTS][UAC_RESAMPLER_HISTORY * 2u]
	__attribute__((aligned(64)));
static int32_t mix_buffer[MIXER_FRAMES * 2u] __attribute__((aligned(64)));

#ifdef UAC_MIXER_TEST
void (*uac_mixer_test_before_commit)(void);
#endif

/* Hooks and USB callbacks use try-locks only; neither path may sleep. */
static int try_lock(int *lock)
{
	return __atomic_exchange_n(lock, 1, __ATOMIC_ACQUIRE) == 0;
}

static void unlock(int *lock)
{
	__atomic_store_n(lock, 0, __ATOMIC_RELEASE);
}

/* A reset blocks new leases and reclaims the ring after the current lease exits. */
static int reader_enter(AudioSource *source)
{
	uint32_t expected = 0;

	return __atomic_compare_exchange_n(&source->reader_guard, &expected,
		READER_ACTIVE, 0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED);
}

static int reader_leave(AudioSource *source)
{
	uint32_t old = __atomic_fetch_and(&source->reader_guard,
		~READER_ACTIVE, __ATOMIC_RELEASE);

	return (old & READER_RESETTING) != 0;
}

static void source_quiesce(AudioSource *source)
{
	__atomic_fetch_or(&source->reader_guard, READER_RESETTING, __ATOMIC_ACQ_REL);
	__atomic_store_n(&source->enabled, 0, __ATOMIC_RELEASE);
}

static int desired_is(const AudioSource *source, uint32_t generation)
{
	return __atomic_load_n(&source->desired_generation, __ATOMIC_ACQUIRE) ==
		generation;
}

/* Called with producer_lock held. Generation zero is the closed state. */
static int source_finish_reset(AudioSource *source, uint32_t epoch,
	uint32_t generation)
{
	uint32_t write;

	if (__atomic_load_n(&source->reader_guard, __ATOMIC_ACQUIRE) !=
	    READER_RESETTING || !desired_is(source, generation))
		return 0;

	write = __atomic_load_n(&source->write_frame, __ATOMIC_ACQUIRE);
	__atomic_store_n(&source->read_frame, write, __ATOMIC_RELEASE);
	source->write_cursor = write;
	source->write_room = SOURCE_FRAMES;
	source->phase = 0;
	source->resample_index = 0;
	source->input_count = 0;
	source->history_primed = 0;
	source->epoch = epoch;

	if (!desired_is(source, generation))
		return 0;

	__atomic_store_n(&source->enabled, generation != 0, __ATOMIC_RELAXED);
	__atomic_store_n(&source->reader_guard, 0, __ATOMIC_RELEASE);
	if (!desired_is(source, generation)) {
		source_quiesce(source);
		return 0;
	}
	return 1;
}

static void source_refresh_room(AudioSource *source)
{
	uint32_t read = __atomic_load_n(&source->read_frame, __ATOMIC_ACQUIRE);
	uint32_t used = source->write_cursor - read;

	source->write_room = used < SOURCE_FRAMES ? SOURCE_FRAMES - used : 0;
}

static int format_supported(uint32_t rate, int mode)
{
	return rate >= MIN_SOURCE_RATE && rate <= MIXER_RATE &&
		(mode == SCE_AUDIO_OUT_MODE_MONO || mode == SCE_AUDIO_OUT_MODE_STEREO);
}

#ifdef UAC_PSTV_ENABLE_LOGGING
static void note_drop(AudioSource *source, uint32_t frames)
{
	__atomic_fetch_add(&source->dropped_frames, frames, __ATOMIC_RELAXED);
}

static void note_copy_error(AudioSource *source)
{
	__atomic_fetch_add(&source->copy_errors, 1u, __ATOMIC_RELAXED);
}

static void stats_reset(void)
{
	uint32_t i;

	for (i = 0; i < UAC_AUDIO_SOURCE_SLOTS; ++i) {
		__atomic_store_n(&sources[i].dropped_frames, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&sources[i].copy_errors, 0, __ATOMIC_RELAXED);
		__atomic_store_n(&sources[i].busy_buffers, 0, __ATOMIC_RELAXED);
	}
}

static void stats_report(void)
{
	uint32_t dropped = 0, errors = 0, busy = 0, i;

	for (i = 0; i < UAC_AUDIO_SOURCE_SLOTS; ++i) {
		dropped += __atomic_load_n(&sources[i].dropped_frames, __ATOMIC_RELAXED);
		errors += __atomic_load_n(&sources[i].copy_errors, __ATOMIC_RELAXED);
		busy += __atomic_load_n(&sources[i].busy_buffers, __ATOMIC_RELAXED);
	}
	if (dropped)
		uac_log(LOG_PREFIX "mixer: %u frames dropped (ring full)\n", dropped);
	if (errors)
		uac_log(LOG_PREFIX "mixer: %u user-buffer copy failures\n", errors);
	if (busy)
		uac_log(LOG_PREFIX "mixer: %u buffers skipped (source busy)\n", busy);
}
#else
#define note_drop(source, frames) ((void)0)
#define note_copy_error(source) ((void)0)
#define stats_reset() ((void)0)
#define stats_report() ((void)0)
#endif

static uint32_t source_take(AudioSource *source, uint32_t epoch,
	uint32_t *read_out)
{
	uint32_t read, write, available;

	if (!reader_enter(source))
		return 0;
	if (!__atomic_load_n(&source->enabled, __ATOMIC_ACQUIRE) ||
	    source->epoch != epoch) {
		reader_leave(source);
		return 0;
	}

	read = __atomic_load_n(&source->read_frame, __ATOMIC_RELAXED);
	write = __atomic_load_n(&source->write_frame, __ATOMIC_ACQUIRE);
	available = write - read;
	if (available == 0) {
		reader_leave(source);
		return 0;
	}

	*read_out = read;
	return available < MIXER_FRAMES ? available : MIXER_FRAMES;
}

static void ring_parts(uint32_t read, uint32_t take, uint32_t *start,
	uint32_t *first)
{
	*start = read & SOURCE_MASK;
	*first = take < SOURCE_FRAMES - *start ? take : SOURCE_FRAMES - *start;
}

static void mix_samples(int32_t *dst, const int16_t *src, uint32_t count,
	int add)
{
	uint32_t i;

	if (add) {
		for (i = 0; i < count; ++i)
			dst[i] += src[i];
	} else {
		for (i = 0; i < count; ++i)
			dst[i] = src[i];
	}
}

static void mix_ring(int32_t *dst, const MixInput *input, int add)
{
	uint32_t start, first, second;

	ring_parts(input->read, input->take, &start, &first);
	second = input->take - first;
	mix_samples(dst, input->pcm + start * 2u, first * 2u, add);
	if (second)
		mix_samples(dst + first * 2u, input->pcm, second * 2u, add);
}

static void copy_ring(int16_t *dst, const MixInput *input)
{
	uint32_t start, first, second;

	ring_parts(input->read, input->take, &start, &first);
	second = input->take - first;
	memcpy(dst, input->pcm + start * 2u, first * 4u);
	if (second)
		memcpy(dst + first * 2u, input->pcm, second * 4u);
}

void uac_mixer_fill(int16_t *output)
{
	MixInput inputs[UAC_AUDIO_SOURCE_SLOTS];
	uint32_t epoch, count = 0, longest = 0, i;
	int reset_raced = 0;

	if (!try_lock(&fill_lock)) {
		memset(output, 0, MIXER_BYTES);
		return;
	}
	if (!__atomic_load_n(&mixer_active, __ATOMIC_ACQUIRE))
		goto silence;

	epoch = __atomic_load_n(&mixer_epoch, __ATOMIC_ACQUIRE);
	for (i = 0; i < UAC_AUDIO_SOURCE_SLOTS; ++i) {
		uint32_t read, take = source_take(&sources[i], epoch, &read);

		if (!take)
			continue;
		inputs[count] = (MixInput){&sources[i], source_pcm[i], read, take};
		if (take > longest)
			longest = take;
		count++;
	}
	if (!count)
		goto silence;

	if (count == 1) {
		copy_ring(output, &inputs[0]);
	} else {
		uint32_t sample;

		mix_ring(mix_buffer, &inputs[0], 0);
		memset(mix_buffer + inputs[0].take * 2u, 0,
			(longest - inputs[0].take) * 2u * sizeof(mix_buffer[0]));
		for (i = 1; i < count; ++i)
			mix_ring(mix_buffer, &inputs[i], 1);
		for (sample = 0; sample < longest * 2u; ++sample) {
			int32_t mixed = mix_buffer[sample];

			if (mixed > 32767)
				mixed = 32767;
			else if (mixed < -32768)
				mixed = -32768;
			output[sample] = (int16_t)mixed;
		}
	}
	if (longest < MIXER_FRAMES)
		memset(output + longest * 2u, 0, (MIXER_FRAMES - longest) * 4u);

#ifdef UAC_MIXER_TEST
	if (uac_mixer_test_before_commit)
		uac_mixer_test_before_commit();
#endif

	for (i = 0; i < count; ++i) {
		__atomic_store_n(&inputs[i].source->read_frame,
			inputs[i].read + inputs[i].take, __ATOMIC_RELEASE);
		reset_raced |= reader_leave(inputs[i].source);
	}
	if (reset_raced)
		memset(output, 0, MIXER_BYTES);
	unlock(&fill_lock);
	return;

silence:
	memset(output, 0, MIXER_BYTES);
	unlock(&fill_lock);
}

void uac_mixer_start(void)
{
	__atomic_store_n(&mixer_active, 0, __ATOMIC_RELEASE);
	__atomic_add_fetch(&mixer_epoch, 1u, __ATOMIC_ACQ_REL);
	stats_reset();
	__atomic_store_n(&mixer_active, 1, __ATOMIC_RELEASE);
	uac_log(LOG_PREFIX "mixer: %u sources x %u frames, %u frames/packet\n",
		UAC_AUDIO_SOURCE_SLOTS, SOURCE_FRAMES, MIXER_FRAMES);
}

void uac_mixer_stop(void)
{
	__atomic_store_n(&mixer_active, 0, __ATOMIC_RELEASE);
	__atomic_add_fetch(&mixer_epoch, 1u, __ATOMIC_ACQ_REL);
	stats_report();
}

int uac_mixer_source_open(int source, uint32_t rate, int mode,
	uint32_t generation)
{
	AudioSource *state;
	int accepted;

	if (source < 0 || source >= UAC_AUDIO_SOURCE_SLOTS)
		return -1;

	state = &sources[source];
	__atomic_store_n(&state->desired_generation, 0, __ATOMIC_RELEASE);
	source_quiesce(state);
	if (!generation || !format_supported(rate, mode)) {
		uac_log(LOG_PREFIX
			"mixer: reject source %d format %u Hz mode %d gen %u\n",
			source, rate, mode, generation);
		return -1;
	}
	if (!try_lock(&state->producer_lock))
		return -1;

	state->rate = rate;
	state->mode = mode;
	state->generation = generation;
	__atomic_store_n(&state->desired_generation, generation, __ATOMIC_RELEASE);
	source_finish_reset(state,
		__atomic_load_n(&mixer_epoch, __ATOMIC_ACQUIRE), generation);
	/* A live reader may defer reset until the first push; supersession may not. */
	accepted = desired_is(state, generation);
	unlock(&state->producer_lock);
	return accepted ? 0 : -1;
}

void uac_mixer_source_close(int source)
{
	AudioSource *state;

	if (source < 0 || source >= UAC_AUDIO_SOURCE_SLOTS)
		return;
	state = &sources[source];
	__atomic_store_n(&state->desired_generation, 0, __ATOMIC_RELEASE);
	source_quiesce(state);
	if (!try_lock(&state->producer_lock))
		return;

	state->rate = 0;
	state->mode = 0;
	state->generation = 0;
	source_finish_reset(state,
		__atomic_load_n(&mixer_epoch, __ATOMIC_ACQUIRE), 0);
	unlock(&state->producer_lock);
}

static int source_prepare_push(AudioSource *source, uint32_t generation,
	uint32_t epoch)
{
	uint32_t guard;

	if (!desired_is(source, generation))
		return 0;
	guard = __atomic_load_n(&source->reader_guard, __ATOMIC_ACQUIRE);
	if ((guard & READER_RESETTING) &&
	    !source_finish_reset(source, epoch, generation))
		return 0;
	if (!__atomic_load_n(&source->enabled, __ATOMIC_ACQUIRE) ||
	    source->generation != generation)
		return 0;
	if (source->epoch != epoch) {
		source_quiesce(source);
		if (!source_finish_reset(source, epoch, generation))
			return 0;
	}
	return desired_is(source, generation) &&
		__atomic_load_n(&source->enabled, __ATOMIC_ACQUIRE) &&
		source->generation == generation;
}

static void source_write_frame(AudioSource *source, int16_t *pcm,
	int16_t left, int16_t right)
{
	uint32_t frame;

	if (!source->write_room) {
		source_refresh_room(source);
		if (!source->write_room) {
			note_drop(source, 1);
			return;
		}
	}
	frame = source->write_cursor & SOURCE_MASK;
	pcm[frame * 2u] = left;
	pcm[frame * 2u + 1u] = right;
	source->write_cursor++;
	source->write_room--;
}

static void source_copy_native(AudioSource *source, int16_t *pcm, SceUID pid,
	const int16_t *input, uint32_t frames)
{
	uint32_t copied = 0;

	while (copied < frames) {
		uint32_t frame = source->write_cursor & SOURCE_MASK;
		uint32_t chunk = frames - copied;
		uint32_t contiguous = SOURCE_FRAMES - frame;

		if (chunk > contiguous)
			chunk = contiguous;
		if (chunk > source->write_room) {
			source_refresh_room(source);
			if (chunk > source->write_room)
				chunk = source->write_room;
		}
		if (!chunk) {
			note_drop(source, frames - copied);
			return;
		}
		if (ksceKernelCopyFromUserProc(pid, pcm + frame * 2u,
			input + copied * 2u, chunk * 4u) < 0) {
			note_copy_error(source);
			return;
		}
		source->write_cursor += chunk;
		source->write_room -= chunk;
		copied += chunk;
	}
}

static const int16_t *resampler_row(uint32_t phase, int *step)
{
	if (phase <= UAC_RESAMPLER_PHASES / 2u) {
		*step = 1;
		return uac_resampler_coefficients[phase];
	}
	*step = -1;
	return &uac_resampler_coefficients[UAC_RESAMPLER_PHASES - phase]
		[UAC_RESAMPLER_TAPS - 1u];
}

static int16_t resampler_round(int32_t value)
{
	int32_t half = 1 << (UAC_RESAMPLER_SHIFT - 1);
	int32_t sample = value >= 0 ? (value + half) >> UAC_RESAMPLER_SHIFT
		: -((-value + half) >> UAC_RESAMPLER_SHIFT);

	if (sample > 32767)
		return 32767;
	if (sample < -32768)
		return -32768;
	return (int16_t)sample;
}

static void source_resampler_output(AudioSource *source, int16_t *pcm,
	const int16_t *history)
{
	uint32_t scaled = source->phase * UAC_RESAMPLER_PHASES;
	uint32_t phase = scaled / MIXER_RATE;
	uint32_t remainder = scaled - phase * MIXER_RATE;
	const int16_t *row0, *row1 = NULL;
	uint32_t blend = 0, tap;
	int step0, step1 = 0;
	int32_t left = 0, right = 0;

	if (!source->write_room) {
		source_refresh_room(source);
		if (!source->write_room) {
			note_drop(source, 1);
			goto advance;
		}
	}

	row0 = resampler_row(phase, &step0);
	if (remainder) {
		row1 = resampler_row(phase + 1u, &step1);
		blend = (remainder * 65536u + MIXER_RATE / 2u) / MIXER_RATE;
	}
	for (tap = 0; tap < UAC_RESAMPLER_TAPS; ++tap) {
		uint32_t frame = (source->resample_index + tap - UAC_RESAMPLER_LEFT) &
			(UAC_RESAMPLER_HISTORY - 1u);
		int32_t coefficient = row0[(int)tap * step0];

		if (remainder) {
			int32_t next = row1[(int)tap * step1];
			coefficient += ((next - coefficient) * (int32_t)blend) >> 16;
		}
		left += history[frame * 2u] * coefficient;
		right += history[frame * 2u + 1u] * coefficient;
	}
	source_write_frame(source, pcm, resampler_round(left), resampler_round(right));

advance:
	source->phase += source->rate;
	if (source->phase >= MIXER_RATE) {
		source->phase -= MIXER_RATE;
		source->resample_index++;
	}
}

static void source_resample_frame(AudioSource *source, int16_t *pcm,
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

static void source_copy_converted(AudioSource *source, int16_t *pcm,
	int16_t *history, int16_t *scratch, SceUID pid, const int16_t *input,
	uint32_t frames, uint32_t channels)
{
	uint32_t copied = 0;

	while (copied < frames) {
		uint32_t chunk = frames - copied;
		uint32_t frame;

		if (chunk > COPY_FRAMES)
			chunk = COPY_FRAMES;
		if (ksceKernelCopyFromUserProc(pid, scratch, input + copied * channels,
			chunk * channels * sizeof(*scratch)) < 0) {
			note_copy_error(source);
			return;
		}
		if (source->rate == MIXER_RATE) {
			for (frame = 0; frame < chunk; ++frame)
				source_write_frame(source, pcm, scratch[frame], scratch[frame]);
		} else {
			for (frame = 0; frame < chunk; ++frame) {
				int16_t left = scratch[frame * channels];
				int16_t right = channels == 1u ? left
					: scratch[frame * channels + 1u];
				source_resample_frame(source, pcm, history, left, right);
			}
		}
		copied += chunk;
	}
}

void uac_mixer_source_push(int source, int pid, const void *buffer,
	uint32_t frames, uint32_t generation)
{
	AudioSource *state;
	uint32_t epoch, channels;

	if (source < 0 || source >= UAC_AUDIO_SOURCE_SLOTS || !buffer || !frames ||
	    !__atomic_load_n(&mixer_active, __ATOMIC_ACQUIRE))
		return;

	state = &sources[source];
	epoch = __atomic_load_n(&mixer_epoch, __ATOMIC_ACQUIRE);
	if (!try_lock(&state->producer_lock)) {
#ifdef UAC_PSTV_ENABLE_LOGGING
		__atomic_fetch_add(&state->busy_buffers, 1u, __ATOMIC_RELAXED);
#endif
		return;
	}
	if (!__atomic_load_n(&mixer_active, __ATOMIC_ACQUIRE) ||
	    __atomic_load_n(&mixer_epoch, __ATOMIC_ACQUIRE) != epoch ||
	    !source_prepare_push(state, generation, epoch))
		goto out;

	channels = state->mode == SCE_AUDIO_OUT_MODE_MONO ? 1u : 2u;
	state->write_cursor = __atomic_load_n(&state->write_frame, __ATOMIC_RELAXED);
	source_refresh_room(state);
	if (state->rate == MIXER_RATE && channels == 2u) {
		source_copy_native(state, source_pcm[source], (SceUID)pid, buffer, frames);
	} else {
		source_copy_converted(state, source_pcm[source], resample_history[source],
			copy_scratch[source], (SceUID)pid, buffer, frames, channels);
	}
	__atomic_store_n(&state->write_frame, state->write_cursor, __ATOMIC_RELEASE);

out:
	unlock(&state->producer_lock);
}
