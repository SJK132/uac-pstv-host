# UAC PSTV

[![License: 0BSD](https://img.shields.io/badge/license-0BSD-blue.svg)](LICENSE)

USB Audio Class 1.0 output for PlayStation TV.

UAC PSTV redirects game, LiveArea, system-effect, and background-music PCM to a
compatible USB audio output. The USB transport is intentionally fixed at
48 kHz, stereo, signed 16-bit PCM for predictable operation on the PSTV host
controller.

The plugin uses the normal taiHEN `*KERNEL` configuration and does not modify
`boot_config.txt`, so the usual hold-L recovery path remains available.

## At a glance

| | Support |
|---|---|
| Console | PlayStation TV only |
| USB protocol | UAC1 output |
| Output format | 48,000 Hz, stereo, signed 16-bit PCM |
| Vita audio | Games, LiveArea/system effects, and BGM ports |
| Hot-plug | Yes |
| Device present during boot | Yes with the tested plugin ordering |
| HDMI | Left untouched; it continues to operate |
| UAC2 / direct RME ADI-2 USB | Not supported yet |
| Debug logging | Compiled out by default |

Tested hardware is an Amazon-branded, Realtek-based USB-to-TOSLINK adapter
(`0bda:4e27`) feeding an RME ADI-2 DAC through optical S/PDIF. Directly
connecting the ADI-2 by USB is not supported because that path requires UAC2.
Other UAC1 devices are accepted only if they expose a matching 48 kHz,
two-channel, 16-bit isochronous OUT alternate setting.

## Installation

The release contains one self-contained module: `uac_pstv.skprx`.

> [!WARNING]
> **StorageMgr users:** `uac_pstv.skprx` must appear before
> `storagemgr.skprx` under `*KERNEL`. Otherwise, the USB audio device may not
> be detected during boot.

1. Copy `dist/uac_pstv.skprx` to `ur0:tai/`.
2. Add it once under `*KERNEL` in `ur0:tai/config.txt`.
3. Remove obsolete `uac_pstv_boot.skprx`, `uac_pstv_audio.skprx`, and
   `vita_uac_host.skprx` entries. In particular, remove any old UAC helper line
   from `ur0:tai/boot_config.txt`.
4. Reboot with the adapter connected. Hot-plugging after boot is also supported.

```text
*KERNEL
ur0:tai/uac_pstv.skprx
ur0:tai/storagemgr.skprx
```

## Logging

> [!NOTE]
> **No log file from the included binary is expected.** Logging is a
> compile-time build option and is disabled in every release artifact. It
> cannot be enabled through `config.txt` on the PSTV.

For diagnostics, build with logging enabled:

```bash
UAC_PSTV_ENABLE_LOGGING=ON bash ./build.sh
```

Or from Windows PowerShell:

```powershell
.\build.ps1 -Logging
```

The diagnostic module is written to `dist/debug/`. Messages use `[uac-pstv]`.
Reinstall the normal file from `dist/` after diagnosis.

## How it works

The module registers its UAC1 driver, fixes a matching endpoint at 48 kHz, and
submits one 192-byte isochronous OUT packet per millisecond. Native 48 kHz
stereo takes a bit-exact direct-copy path.

Other supported Vita source rates use a compact 24-tap fixed-point polyphase
FIR. This resampling is necessary for 44.1 kHz BGM; the USB link itself always
remains at 48 kHz. The audio path performs no allocation or floating-point
work. With no supported adapter attached, no USB stream runs and captured PCM
returns before copying or resampling.

The USB copy is taken before Sony's final hardware output stage. System master
volume or post-processing may therefore differ from HDMI; use the DAC's volume
control when needed.

## Building

The quickest WSL2 build is:

```bash
bash ./build.sh
```

This runs host-side mixer and race tests, builds `uac_pstv.skprx` with VitaSDK,
verifies that logging is absent from a release build, and copies the artifact
to `dist/`. The build tree defaults to `/tmp/uac-pstv-build` to avoid DrvFS
permission problems.

The PowerShell wrapper performs the same build:

```powershell
.\build.ps1
```

For complete setup instructions, see [Building on Windows with WSL2](docs/BUILDING.md).

## Project status and limitations

- Current target: PSTV, UAC1, 48 kHz, stereo, 16-bit PCM.
- UAC2 and direct USB output to devices such as the RME ADI-2 are future work.
- 8, 11.025, 12, 16, 22.05, 24, 32, and 44.1 kHz Vita sources are resampled
  internally; the USB link remains fixed at 48 kHz.
- USB hubs and multi-device combinations have not been validated as release
  configurations, apart from basic HID coexistence observed during development.
- This is kernel software. Keep backups and test new builds with a recovery
  path available.

When reporting an issue, include the PSTV firmware, adapter VID:PID, full UAC
descriptors if available, storage-plugin configuration and ordering, and a
diagnostic log.

## Credits

- **OpenAI Codex** — driver architecture, implementation, optimization,
  host-side tests, build tooling, and documentation.
- **Anthropic Claude** — independent code review and a mixer/drain-race revision
  incorporated during stabilization.
- [psvita-usb-audio-midi](https://github.com/intermynd-instruments/psvita-usb-audio-midi)
  — audio-processing reference work.
- [TVIKEY](https://github.com/isage/tvikey) and
  [vita-packages-extra](https://github.com/isage/vita-packages-extra) — Vita USB
  host examples and packaging support.
- [vita-udcd-uvc](https://github.com/xerpi/vita-udcd-uvc) and
  [Vita-USB-Stream](https://github.com/BenMitnicK/Vita-USB-Stream) — prior Vita
  USB streaming work that informed the investigation.
- The VitaSDK, taiHEN, HENkaku, Ensō, and StorageMgr contributors.

## License

UAC PSTV is released under the [BSD Zero Clause License](LICENSE) (`0BSD`). It
permits use, copying, modification, and distribution for any purpose, with or
without a fee and without an attribution condition. The license retains the
standard warranty and liability disclaimer.
