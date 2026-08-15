# Changelog

## v1.2 - 2026-08-15

- Rewrote the session lifecycle around a single owning thread. Setup and
  teardown are straight-line code now instead of a chain of USB callbacks, which
  removed the state machine, reference counts and generation tokens that existed
  only to defend against them.
- Fixed a device attached before boot never being configured.
- USB host mode is no longer re-asserted on every system event.
- Improved system software stability and performance.

## v1.1 - 2026-08-14

- USB transport is now four fixed 1 ms contexts: three in flight, one staged
  behind them.
- Reworked the UAC1 and route-ownership state machines so every handoff has
  exactly one owner. This fixes a device that would refuse to start for the rest
  of the session once the console had used audio input.
- Improved system software stability and performance.

## v1.0.1 - 2026-08-12

Output latency cut from roughly 22 ms to 13 ms, a memory-ordering fix in the PCM
handoff, and recovery fixes for the sleep/wake path -- the last of these from a
report of no audio after a PSX game plus standby until the device was replugged.

### Latency

- Capture blocks halved to 240 frames (5 ms). Latency is about two block
  periods: one to fill a block, and one more because the consumer has to start a
  whole block behind to survive the discrete arrival cadence. Block size is
  therefore the only lever that moves it. Confirmed on hardware that Sony's
  RAM-output submit accepts the smaller page.
- PCM staging widened from two slots to four, restoring the stall tolerance that
  smaller blocks would otherwise have halved. Slot count costs no latency -- it
  sets how far the feeder may fall behind, not where it starts reading.
- A third transport context, taking the feeder's slack from one packet period to
  two in exchange for 1 ms of delay.
- The feeder thread runs at priority 0x20 instead of 0x40 and is pinned to the
  capture worker's core, so a freshly published block is still in L1 when it is
  read back out rather than costing fifteen cache-line transfers per block.

### Correctness

- Fixed memory ordering on the seqlock read side. The second guard read used an
  acquire load, which places its barrier *after* the read and leaves the data
  copy free to be reordered past the validation -- the classic way a torn read
  passes as valid. It now fences before a relaxed read, matching Linux's
  `read_seqretry()`. Same two instructions, correct order. The symptom would have
  been a rare, unreproducible click.
- The write side no longer uses a seq_cst store where a relaxed store plus a
  release fence suffice, removing one barrier per published block.

### Recovery

- An attach that arrives while a teardown is still running is now remembered and
  retried once teardown completes. USBD offers a device exactly once, so refusing
  that attach previously lost it for good: the device stayed enumerated with no
  session behind it and only a replug recovered it. The window is widest coming
  out of sleep with audio streaming, which is when teardown is slowest.
- Releasing the AVConfig route no longer waits for Sony to acknowledge through
  the hooked device-stop wrapper. Capture is already stopped by that point, so
  Sony may reasonably decide there is nothing to stop and never call it. Observed
  both ways across sessions, which is exactly why the release must not depend on
  it; when it did not come, the wait burned its full one-second timeout.

### Instrumentation

- Logging builds print the version and git revision as the first line of each
  session, and report per-session packet, starve, PCM-wait and resync counts plus
  the minimum producer lead, recording where each resync occurred. This
  instrumentation is compiled out of release builds.

Measured on 3.65. Route release settles in a repeatable ~401 ms. Suspend retires
the session ahead of the detach that follows, so `SET_INTERFACE(alt 0)` is still
delivered and an external DAC returns to its internal clock.

USB completions were found to stall for ~250 ms at a time, several times per
session, independent of our thread priorities (verified by A/B). The producer
runs on through such a stall and ends up ~50 blocks ahead; the latest-wins
staging discards the backlog and resumes at current audio. A FIFO would instead
queue it and add that 250 ms to output latency permanently, on every stall.

## v1.0 - 2026-08-12

Complete rework of how audio is captured. v0.1-v0.3 hooked individual SceAudio
ports, which meant carrying a mixer and a 44.1-to-48 kHz resampler and competing
with the audio pipeline for timing. v1.0 instead takes ownership of AVConfig's
RAM-output route -- the path Sony already uses to feed Bluetooth -- and consumes
system audio *after* Sony's own mixing and rate conversion.

- Removed `mixer.c` and `resampler_coeffs.h`. Both are unnecessary now: the
  captured stream is already mixed and already 48 kHz, whatever the source rate.
- Replaced port hooks with a 480-frame RAM-output A/B worker matching Sony's own
  DataSend worker, feeding a two-context isochronous transport with exactly one
  request in flight. This addresses the 1.4% ring-buffer sample loss noted in
  v0.3.
- Firmware support is now an enforced allowlist covering 3.60, 3.61, 3.63, 3.65,
  3.67, 3.68, 3.71 and 3.73. SceAVConfig's text is byte-identical across all
  eight, so one offset set covers the range; an unlisted module NID is refused
  and no hooks are installed.
- Session teardown moved to its own worker and off the USBD callback thread,
  fixing dropped events during rapid unplug/replug.
- `SET_INTERFACE(alt 0)` is now sent on clean teardown, so an external DAC is
  told the stream ended and returns to its internal clock.
- Suspend retires the live session; resume re-asserts USB host mode.
- The transport is primed with silence so the device sees a continuous stream
  from the first frame rather than a ~20 ms gap while capture spins up.
- Refuses to load on anything that is not a PlayStation TV.
- With no USB audio device attached the plugin is inert: it registers a USB
  driver and waits, never reading SceAVConfig and installing no hooks.


## v0.3 - 2026-08-06

- Refactored uac1, mixer, stream class, and renamed audio to audio_hooks.
- fixed a bug where psx game with sample rate of 44148 have no in game audio.

- Area needs improvement: currently the isochronous usb transfer function
  with callback can not satisfy 48Khz audio, during on device testing we are
  seeing 1.4% of dropped sample from ring buffer. 

## v0.2 - 2026-08-05

- Replaced the separate early-boot and audio modules with one self-contained
  `uac_pstv.skprx` loaded from taiHEN's `*KERNEL` section. The
  `boot_config.txt` helper is no longer required.
- Improved detection of adapters connected during boot when UAC PSTV is listed
  before StorageMgr. Hot-plug support remains available.
- Replaced the basic sample-rate conversion path with a compact 24-tap
  fixed-point polyphase FIR, reducing 44.1 kHz BGM interpolation hiss and
  aliasing while keeping native 48 kHz stereo bit-exact.
- Reduced capture capacity from eight simultaneous SceAudio ports to four,
  cutting static ring-buffer memory from 137,920 to 69,440 bytes while retaining
  one spare slot beyond the three-source pattern observed in testing.
- Simplified the USB streaming and mixer hot paths, retained saturating
  multi-source mixing, and kept inactive processing disabled when no supported
  USB adapter is attached.
- Expanded host-side tests with all-rate cadence, spectral and continuity
  coverage, deterministic drain/race checks, and resampler benchmarks.
- Simplified the build and release package to produce a single module and verify
  that release logging is compiled out.

## v0.1 - 2026-08-05

- Initial public release.
- Split early USB transport and late SceAudio capture into
  `uac_pstv_boot.skprx` and `uac_pstv_audio.skprx` so a connected UAC device
  could enumerate during boot.
- Added fixed UAC1 stereo, signed 16-bit, 48 kHz isochronous output.
- Captured game, LiveArea/system-effect, and BGM audio ports.
- Added 44.1-to-48 kHz conversion, saturating multi-source mixing, hot-plug,
  callback draining, and inactive fast paths.
- Removed release-time file logging and USB power-reset fallbacks.
- Added host-side mixer correctness and deterministic drain/race tests.
