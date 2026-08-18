# uac-pstv

**USB audio output for the PlayStation TV.**

Plug a USB sound card, a USB DAC, or one of those cheap USB headphone dongles
into your PSTV's USB port and system audio comes out of it — games, PSX titles,
the LiveArea, everything the console plays. No app to launch and nothing to
configure: attach the device and it takes over, unplug it and audio returns to
HDMI on its own.

This is useful if your TV's audio is bad, if you want headphones without running
a cable from the TV, or if you want a real DAC in the chain.

It is **output only**. Microphones and headset input are not supported.

## Install
> [!WARNING]
> **StorageMgr users:** `uac_pstv.skprx` must appear before
> `storagemgr.skprx` under `*KERNEL`. Otherwise, the USB audio device may
> not be detected during boot.
> 
1. Copy `uac_pstv.skprx` to `ur0:tai/`.
2. Add it under the `*KERNEL` section of `ur0:tai/config.txt`
   ```
   *KERNEL
   ur0:tai/uac_pstv.skprx
   ur0:tai/storagemgr.skprx
   ```
3. Reboot.

To uninstall, remove that line and reboot.


## Will my device work?

Probably, if it's a simple one. The transport is fixed, so a device has to
advertise all of:

- 48 kHz, stereo, 16-bit PCM (USB Audio Class **1**)
- an isochronous **OUT** endpoint, adaptive or synchronous
- full speed with `bInterval=1`, or high speed with `bInterval=4` — 1 ms packets
- a max packet size of at least 192 bytes

In practice most inexpensive USB-C/USB-A headphone dongles and basic USB sound
cards are UAC1 and work. Devices that are **UAC2 only** will not — that includes
a lot of higher-end audiophile DACs, though many of them also expose a UAC1 mode.

Anything unsuitable is declined during probe and the console carries on
normally. If a device doesn't work and you want to know why, run the logging
build (below) and look for a `reject ep` line — it names the exact reason.

## Supported firmware

Verified profiles are 3.60, 3.61, 3.63, 3.65, 3.67, 3.68, 3.69, 3.70, 3.71, 3.72
and 3.73. PlayStation TV only; the plugin checks and unloads itself on a handheld
Vita. Unverified firmware (currently only 3.74) is rejected at USB attach without
installing a hook. It needs a real `SceAVConfig` module sample before it can be
added safely.


## How it works

The console has no public API for "give me the audio that's about to be played,"
so the plugin borrows Sony's own path for it.

`SceAVConfig` already knows how to render system audio into a RAM buffer instead
of sending it to HDMI — that's the route it uses to feed Bluetooth audio. The
plugin claims that route for itself, then runs a worker that rotates seven
144-frame slices through Sony's `SceAudio` RAM-output function, so Sony's own
engine writes the staging buffer directly rather than a private copy of it. A
seqlock guard word per slice marks which ones are complete.

On the USB side, a UAC1 driver enumerates the device, walks its descriptors to
find a stream interface it can actually drive, selects the alternate setting and
sets the sample rate. A feeder thread cuts those 144-frame slices into
48-frame (192-byte) packets and rotates four fixed isochronous contexts: three
owned by USBD and one READY or being prepared. The completion callback submits
the oldest READY context before waking the feeder, keeping the periodic USB
schedule ahead without building a deep audio queue.

Two taiHEN hooks make Sony cooperate while USB owns the route: one redirects the
DataSend start into the plugin's capture worker, and one absorbs the
physical-stop wrapper locally so tearing down the USB stream doesn't wait on a
Bluetooth completion that can never arrive. Both hooks are removed after a
successful teardown; a failed teardown deliberately keeps the module resident
with its ownership state intact. Bluetooth handoff is not implemented in v1:
an already-active BT worker prevents USB from taking the route, and USB should
be unplugged before starting a new Bluetooth-audio session.

A few details that matter if you go reading the source:

- **Latency** is about 9 ms: 3 ms to fill a capture slice, about 3 ms because
  the consumer trails the producer by one slice, plus 3 ms submitted to USB.
- **Staging is latest-wins, not a queue.** The capture worker isn't
  rate-locked to 48 kHz — it publishes whatever `ram_submit` returns, whenever it
  returns — so a FIFO underneath it thrashes. The seqlock lets the reader snap to
  the freshest block instead.
- **One thread owns a session end to end** — opening the pipes, the setup
  control transfers, taking the route, and the whole teardown. USB callbacks
  only post to it and return, because releasing the route can take seconds and
  blocking a detach callback is what makes rapid replug drop events.
- **Alternate setting 0** is selected on a clean teardown, so an external DAC is
  told the stream ended and drops back to its internal clock instead of staying
  locked to USB.
- **USB host mode is asserted at start and again on resume.** The PSTV is
  already in host mode normally — that's what the port is for — so this is a
  re-assert rather than a mode change, and it mainly matters coming back from
  sleep.

<details>
<summary>Module NIDs and how the range was established</summary>

`SceAVConfig`'s text segment is **byte-identical** on every firmware below —
across all of them the module differs only in its own NID and one digit of an
internal version string. The private offsets this plugin uses are therefore
fixed rather than per-firmware, and one binary covers the whole range.

| Firmware | SceAVConfig module NID |
|---|---|
| 3.60 | `0x222DDEB1` |
| 3.61 | `0xCC9A71FB` |
| 3.63 | `0x83636271` |
| 3.65 | `0x55A6E312` |
| 3.67 | `0x1A5B797C` |
| 3.68 | `0xA1F08F46` |
| 3.69 | `0x4069C16D` |
| 3.70 | `0x3F226D11` |
| 3.71 | `0x5B294543` |
| 3.72 | `0x0790F1A9` |
| 3.73 | `0x136D0561` |

Verified by comparing decrypted modules: the two hooked function prologues, the
`movw`/`movt` pair that yields the private data base, and the data segment size
are the same in all eleven. Retail and devkit ship the same module — a retail
3.65 console reports `0x55A6E312`, matching the 3.65 devkit dump.

The three `SceAudio` RAM-output functions are pinned by text offset on the same
evidence, and checked against their first instruction before use. SceAudio is
byte-identical across the same eleven firmware.

The NID list is enforced, not just documentation: a module NID outside it is
refused and no hooks are installed. Firmware outside this verified list loses
USB audio, but nothing else — the plugin and USB driver still load and the rest
of the system is untouched.

Two gates run, in order:

1. **Module NID** — identifies the exact build, and must be one of the eleven.
2. **Small sanity check** — each hook entry must still contain the recorded
   instruction and the `movw`/`movt` pair must decode for the expected register.
   The three SceAudio entry points must each begin with their recorded
   instruction.

The second gate is deliberately thin, because the NID has already pinned the
firmware. It checks the two hook entries and the instruction pair used to derive
the data base; it is not a general module-integrity scanner.

Logging builds report an unverified NID and the validation boundary that failed.

</details>

## What it touches

Kernel plugins deserve suspicion, so here is exactly what this one does, and
when.

**With no USB audio device attached, it does no audio-routing work.** It keeps
only its USB driver, lifecycle thread and system-event handler registered.
`SceAVConfig` is never read and no hooks are installed. Attach a device that
isn't UAC1 and it is declined during probe without touching Sony audio state.

The two taiHEN hooks go in **only after** a UAC1 device has been probed,
claimed, configured, had its streaming interface selected and its sample rate
accepted. They come back out after teardown completes successfully; a failed
teardown keeps them installed so the resident module can retry safely.

**Nothing persists.** The release build writes nothing to storage — no registry
keys, no flash, no files. Everything it does lives in RAM: two hooks, and writes
to AVConfig's data segment. A reboot is a complete reset, so there is no state
that can be left in a bad shape.

**`module_start` does not resolve or dereference private Sony addresses.** A
wrong profile cannot affect boot: resolution and hook installation happen only
after a supported UAC1 device has configured successfully.

If something does go wrong, remove the line from `config.txt` and reboot.

One known limitation, because a safety section that only brags is worth less:
if releasing the audio route ever times out, AVConfig can be left mid-transition
and system audio may stay silent until you reboot. Nothing is written anywhere,
so a power cycle always restores it — but that is the one case that needs one.

## Source layout

- `src/main.c` — module, system-event, and USB-driver lifetime.
- `src/uac1.c` — UAC1 descriptor parsing and the three USB driver callbacks.
- `src/session.c` — the session thread: device setup, route handover, teardown.
- `src/stream.c` — 7×144-frame slice staging, 48-frame packetizer, and the fixed
  four-context scheduler with three USB requests in flight.
- `src/audio_tap.c` — AVConfig route ownership, the slice-rotating capture worker,
  the DataSend-start hook, and virtual physical-stop acknowledgment.
- `src/resolver.c` — taiHEN module lookup, firmware gate, and Sony function
  resolution.
- `src/log.c` — debug-only logging; absent from release builds.

The plugin imports no firmware-specific `SceModulemgr` functions. It finds
`SceAVConfig` and `SceAudio` through taiHEN and installs its two offset hooks
only after a UAC1 device has configured successfully.

## Build

Requires [VitaSDK](https://vitasdk.org/). From PowerShell:

```powershell
.\build.ps1
```

Or from WSL2 / Linux:

```bash
bash ./build.sh
```

The release build is written to `dist/uac_pstv.skprx`, compiled with logging
disabled and `-O2` for Cortex-A9. For a build that writes
`ur0:data/uac_pstv.log`, configure with `-DUAC_PSTV_ENABLE_LOGGING=ON`.

## Testing

After a full reboot:

1. Normal system audio, then PSX audio.
2. Unplug and replug the USB device several times in quick succession.
3. Sleep with the device connected, then wake. The device should be told the
   stream ended so an external DAC returns to its internal clock.
4. Unload the module while the USB device is detached.

## Credits

* OpenAI Codex — driver architecture, implementation, optimization, host-side
  tests, build tooling, and documentation.
* Anthropic Claude — the v1.0 rework around AVConfig's RAM-output route and its
  sleep/wake recovery fixes; v1.0.1's latency reduction, feeder scheduling, and
  a seqlock memory-ordering fix in the PCM handoff; and v1.1's disassembly-level
  review of the 1 ms path, covering completion ordering, DMA page containment,
  and the reservation-granule grouping of shared state.
* [psvita-usb-audio-midi](https://github.com/intermynd-instruments/psvita-usb-audio-midi)
  — audio-processing reference work.
* [TVIKEY](https://github.com/isage/tvikey) and
  [vita-packages-extra](https://github.com/isage/vita-packages-extra) — Vita
  USB host examples and packaging support.
* [vita-udcd-uvc](https://github.com/xerpi/vita-udcd-uvc) and
  [Vita-USB-Stream](https://github.com/BenMitnicK/Vita-USB-Stream) — prior
  Vita USB streaming work that informed the investigation.
* The VitaSDK, taiHEN, HENkaku, Ensō, and StorageMgr contributors.

## License

MIT — see `LICENSE`.
