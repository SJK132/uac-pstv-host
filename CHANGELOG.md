# Changelog

## v1.1 - 2026-08-14

A state-ownership pass over the whole plugin, one reproducible startup failure
fixed, and a set of scheduling and memory-layout changes on the 1 ms path. The
last of these are individually tiny and were nearly dropped as unmeasurable;
together they are audible as reduced clipping under PSP emulation, which is the
load case where the console actually runs short of the deadline.

### Fixed

- USB audio refused to start for the rest of the session once the console had
  used audio input. The receive-worker flag was read from the wrong AVConfig
  word -- the DataRecv sample rate rather than its active flag -- which stays
  non-zero after any capture and made route acquisition report the route busy
  forever. Confirmed against the module's own initialiser, which clears the two
  worker flags as a pair.
- A start request arriving inside the teardown window could clear the capture
  worker's stop flag, leaving it running after the route had been released. Stop
  and start now order against each other on a single atomic.
- A completion delayed across a rapid replug could act on context storage a
  later session had reused. Callbacks now carry an immutable
  generation-and-index token instead of a context pointer, so a stale one is
  rejected before anything is dereferenced.
- Retirement could be claimed twice, letting a late finaliser write a live
  session back to idle. One CAS now decides the winner.
- The packetizer's block distance was off by one across the sequence wrap, and
  a resync at the wrap placed the cursor on the newest block with no margin.

### Transport

- Completions submit the next request before waking the feeder. The wakeup is a
  kernel call that can reschedule on the spot, and it was running ahead of the
  submit that has to reach the controller inside the frame.
- The pump gate is a request counter rather than a boolean. A thread that loses
  the gate leaves a ticket the owner must consume, so work published just as the
  owner finished its last scan can no longer be stranded.
- Autonomous failures -- capture submit, transfer status, PCM wait -- now reach
  the UAC1 teardown worker. Previously the stream died while the lifecycle stayed
  in STREAMING with nothing to recover it.
- Startup silence is sent while route acquisition is polled, instead of leaving
  an already-selected isochronous endpoint with no requests on it.

### Layout

- Each 192-byte DMA buffer is aligned so it cannot straddle a 4 KiB page, which
  EHCI would otherwise have to describe with a second buffer-page pointer. It
  held by accident of where BSS landed; static asserts now make it a guarantee.
- Shared state is grouped by which thread touches it, with each group padded to
  own one 32-byte Cortex-A9 reservation granule. The per-millisecond PCM cursor
  had drifted into the same granule as the two counters the other core does
  ldrex/strex on every packet, where its plain stores were clearing reservations
  and forcing CAS retries. Nothing in the source had ever stated the requirement,
  so nothing warned when a compiler anchor group shifted.
- Removed a redundant barrier per completion, and the cache clean no longer runs
  on the abort path.

### Recovery

- Stream, tap and UAC1 teardown fail closed. Worker UIDs, callbacks, hooks,
  pipes and route ownership are never declared retired before the subsystem that
  owns them confirms it, and a residual capture-thread UID is now recovered
  rather than becoming a permanent error.
- Sony's receive worker is an acquire-time conflict only. Waiting on it to go
  idle during release would have blocked our own teardown on an unrelated
  subsystem.
- Attach, deferred attach and detach hand the session generation over under the
  existing lock.

### Resolution

- The three SceAudio RAM functions are resolved by exported NID rather than by
  fixed text offset. Private AVConfig offsets remain gated by verified module
  NIDs plus an instruction check at each hook site.

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
