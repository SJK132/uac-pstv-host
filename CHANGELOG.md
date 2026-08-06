# Changelog

## v0.2 - 2026-08-05

- Added one self-contained core module for USB transport, SceAudio capture,
  resampling, and mixing; no `boot_config.txt` helper is required.
- Added fixed UAC1 stereo, signed 16-bit, 48 kHz isochronous output.
- Captured game, LiveArea/system-effect, and BGM audio ports.
- Added a compact 24-tap fixed-point FIR for non-48 kHz sources while keeping
  native 48 kHz stereo bit-exact, saturating multi-source mixing, hot-plug,
  callback draining, and inactive fast paths.
- Removed release-time file logging and USB power-reset fallbacks.
- Documented the required UAC-before-StorageMgr ordering and YAMT compatibility.
- Sized capture for four simultaneous SceAudio ports, reducing static ring RAM
  while retaining one spare beyond the three-source pattern seen in testing.
- Added host-side mixer correctness, all-rate spectral/continuity coverage,
  deterministic drain/race tests, and hot-path benchmarks.
