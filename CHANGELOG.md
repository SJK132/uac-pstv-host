# Changelog

## v0.1 - 2026-08-05

- Split early USB transport and late SceAudio capture into separate kernel
  modules so a connected UAC device can enumerate during boot.
- Added fixed UAC1 stereo, signed 16-bit, 48 kHz isochronous output.
- Captured game, LiveArea/system-effect, and BGM audio ports.
- Added 44.1-to-48 kHz conversion, saturating multi-source mixing, hot-plug,
  callback draining, and inactive fast paths.
- Removed release-time file logging and USB power-reset fallbacks.
- Added host-side mixer correctness and deterministic drain/race tests.
