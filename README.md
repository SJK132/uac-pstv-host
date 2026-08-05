# UAC PSTV

[![License: 0BSD](https://img.shields.io/badge/license-0BSD-blue.svg)](LICENSE)

USB Audio Class 1.0 output for PlayStation TV.

UAC PSTV redirects game, LiveArea, system-effect, and background-music PCM to a
compatible USB audio output. The transport is intentionally narrow: stereo,
16-bit PCM at 48 kHz over UAC1. Keeping that contract fixed made the driver
small, predictable, and stable on the PSTV's USB host controller.

> [!WARNING]
> `uac_pstv_boot.skprx` is an early-boot kernel module. Back up
> `ur0:tai/boot_config.txt` and make sure you have a recovery method before
> editing it. A typo or bad boot-module order can prevent a normal boot.

## At a glance

| | Support |
|---|---|
| Release | v0.1 |
| Console | PlayStation TV only |
| USB protocol | UAC1 output |
| Output format | 48,000 Hz, stereo, signed 16-bit PCM |
| Vita audio | Games, LiveArea/system effects, and BGM ports |
| Hot-plug | Yes |
| Device present during boot | Yes, with the required early-boot order |
| HDMI | Left untouched; it continues to operate |
| UAC2 / direct RME ADI-2 USB | Not supported yet |
| Debug logging | Compiled out by default |

Tested hardware is an Amazon-branded, Realtek-based USB-to-TOSLINK adapter
(`0bda:4e27`) feeding an RME ADI-2 DAC through optical S/PDIF. Directly
connecting the ADI-2 by USB is not supported because that path requires UAC2.
Other UAC1 devices are accepted only if they expose a matching 48 kHz,
two-channel, 16-bit isochronous OUT alternate setting.

## Installation

The release contains two modules because they must load at different stages:

| File | Configuration | Purpose |
|---|---|---|
| `uac_pstv_boot.skprx` | Ensō `ur0:tai/boot_config.txt` | Registers the UAC1 USB driver before enumeration and owns the isochronous stream |
| `uac_pstv_audio.skprx` | taiHEN `ur0:tai/config.txt` under `*KERNEL` | Captures and mixes SceAudio output after taiHEN is available |

Do not place both modules in `config.txt`, and do not place the audio module in
`boot_config.txt`.

### 1. Copy the modules

Copy these files from [`dist`](dist/) to `ur0:tai/`:

```text
uac_pstv_boot.skprx
uac_pstv_audio.skprx
```

Release hashes are recorded in [`dist/SHA256SUMS.txt`](dist/SHA256SUMS.txt).

Remove or comment out any older `vita_uac_host.skprx` entry. Loading the old
driver alongside UAC PSTV can duplicate SceAudio hooks or make two USB drivers
compete for the same device.

### 2. Add the early USB module

Open `ur0:tai/boot_config.txt`. Find Sony's existing USB block and insert the
UAC line **immediately after `usbd.skprx` and before both `udcd.skprx` and
`usbserv.skprx`**:

```text
load os0:kd/usbd.skprx
- load ur0:tai/uac_pstv_boot.skprx
load os0:kd/udcd.skprx
load os0:kd/usbserv.skprx
```

The whitespace between `load` and the path may be spaces or a tab. Preserve all
other model- and firmware-specific lines in the file. In particular:

- Do not move or duplicate `usbd.skprx`.
- Do not put UAC PSTV before `usbd.skprx`; it imports `SceUsbdForDriver`.
- Do not put it after `udcd.skprx` or `usbserv.skprx`; the USB device may already
  have been enumerated by then.
- If YAMT already has an early `- load ur0:tai/yamt.skprx` line, leave that line
  where YAMT installed it. Only place the UAC line relative to Sony's USB block
  as shown above.

### 3. Add the audio module

Open `ur0:tai/config.txt` and add the following line once under `*KERNEL`:

```text
*KERNEL
ur0:tai/uac_pstv_audio.skprx
```

It may share the existing `*KERNEL` section with other plugins. The recommended
order is storage/system plugins first, UAC PSTV next, and any other plugin that
hooks SceAudio after it. Multiple SceAudio-hooking plugins are not guaranteed to
cooperate, so start with those disabled when diagnosing missing or duplicated
audio.

### 4. Reboot and test

Connect the UAC1 adapter before powering on, then reboot the PSTV. Audio should
begin when a supported device is attached and remain idle when no device is
present. Hot-plug is supported after boot.

> [!NOTE]
> **No log file from the included binaries is expected.** Logging is a
> compile-time build option and is disabled in every release artifact. It
> cannot be enabled through `config.txt` on the PSTV.

For diagnostics, build logging-enabled modules from WSL2:

```bash
UAC_PSTV_ENABLE_LOGGING=ON bash ./build.sh
```

Or from Windows PowerShell:

```powershell
.\build.ps1 -Logging
```

The diagnostic modules are written to `dist/debug/`. Copy both of those
modules to `ur0:tai/`, reboot, and then read `ur0:data/uac_pstv.log`. Messages
use the `[uac-pstv-boot]` and `[uac-pstv-audio]` prefixes. Reinstall the normal
modules from `dist/` after diagnosis.

## Plugin compatibility

The PSTV has one physical USB host port, so a USB audio adapter and a USB
storage device cannot be plugged into that port simultaneously. StorageMgr can
remain installed: its mass-storage driver does not claim a UAC audio device.

| Plugin or feature | Status | Notes |
|---|---|---|
| StorageMgr | Compatible | It can remain installed with the tested `INT=imc0`, `GCD=ux0`, `UMA=uma0` layout. A connected UAC device is not claimed as USB mass storage. |
| YAMT Lite | Compatible | Recommended when using SD2Vita, internal storage, or a memory card with UAC PSTV. |
| YAMT Full | Conditional | Keep **Enable experimental USB patches** and **Force legacy USB/PSVSD mode** disabled. Do not configure the PSTV USB port as active mass storage while using the audio adapter. |

The tested StorageMgr mapping is:

```text
INT=imc0
GCD=ux0
UMA=uma0
```

## How it works

The early module registers a narrow `SceUsbd` class driver, selects the matching
UAC1 alternate interface, fixes the endpoint at 48 kHz, and continuously submits
192-byte isochronous OUT packets. The late module hooks SceAudio port
open/config/output/release calls, converts active sources to 48 kHz stereo, and
feeds the transport through a small callback bridge.

The bridge is designed so the audio module can unload safely: it disables new
callbacks and drains any callback already in progress before its code is
released. With no supported USB device attached, there is no isochronous stream
and PCM pushes return before copying or resampling audio.

The USB copy is taken before Sony's final hardware output stage. As a result,
system master-volume or post-processing behavior may not exactly match HDMI;
use the DAC's volume control when needed.

## Building

The quickest build from WSL2 is:

```bash
bash ./build.sh
```

This runs the host-side mixer and drain/race tests, builds both modules with
VitaSDK, and copies the results to `dist/`. The build directory defaults to
`/tmp/uac-pstv-build` to avoid DrvFS permission problems.

From Windows PowerShell, the wrapper performs the same build through WSL2:

```powershell
.\build.ps1
```

For the complete toolchain setup, see [Building on Windows with WSL2](docs/BUILDING.md).

To include file logging in a diagnostic build:

```bash
UAC_PSTV_ENABLE_LOGGING=ON bash ./build.sh
```

Release builds leave logging off and do not link the debug or file-I/O logger
imports. Diagnostic artifacts are placed in `dist/debug/` so they cannot
overwrite the release binaries. Do not report timing or performance results
from a logging-enabled build as release behavior.

## Project status and limitations

- Current target: PSTV, UAC1, 48 kHz, stereo, 16-bit PCM.
- UAC2 and direct USB output to devices such as the RME ADI-2 are future work.
- 44.1 kHz Vita sources are resampled internally; the USB link remains fixed at
  48 kHz.
- USB hubs and multi-device combinations have not been validated as supported
  release configurations, apart from basic HID coexistence observed during
  development.
- This is kernel software. Keep backups and test new builds with a recovery path
  available.

When reporting a device issue, include the PSTV firmware, adapter VID:PID, full
UAC descriptors if available, storage-plugin configuration, both plugin-order
snippets, and a diagnostic log.

## Credits

- **Tian** — project direction, hardware testing, listening tests, measurements,
  and compatibility validation on PSTV and RME hardware.
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
- The VitaSDK, taiHEN, HENkaku, Ensō, YAMT, and StorageMgr contributors.

## License

UAC PSTV is released under the [BSD Zero Clause License](LICENSE) (`0BSD`). It
permits use, copying, modification, and distribution for any purpose, with or
without a fee and without an attribution condition. The license retains the
standard warranty and liability disclaimer.
