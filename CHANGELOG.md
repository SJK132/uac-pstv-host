# Changelog

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
