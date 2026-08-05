# UAC PSTV v0.1 installation

> **Warning:** `uac_pstv_boot.skprx` is an early-boot kernel module. Back up
> `ur0:tai/boot_config.txt` and keep a recovery method available.

1. Copy `uac_pstv_boot.skprx` and `uac_pstv_audio.skprx` to `ur0:tai/`.
2. In `ur0:tai/boot_config.txt`, insert the boot module immediately after
   Sony's `usbd.skprx` and before `udcd.skprx` and `usbserv.skprx`:

   ```text
   load os0:kd/usbd.skprx
   - load ur0:tai/uac_pstv_boot.skprx
   load os0:kd/udcd.skprx
   load os0:kd/usbserv.skprx
   ```

3. Add the audio module once under `*KERNEL` in `ur0:tai/config.txt`:

   ```text
   *KERNEL
   ur0:tai/uac_pstv_audio.skprx
   ```

4. Remove or comment out any old `vita_uac_host.skprx` entry, then reboot.

This release is PSTV-only and accepts UAC1 stereo, signed 16-bit, 48 kHz OUT
interfaces. It does not support UAC2. Do not enable USB mass-storage or USB
device mode on the same PSTV USB port. StorageMgr is compatible and can remain
installed; the physical port simply cannot hold a USB storage device and the
audio adapter simultaneously. See the repository README for the YAMT and
StorageMgr compatibility table and recovery notes.

## Logging

The included release modules do **not** create `ur0:data/uac_pstv.log` because
logging is compiled out. To create diagnostic modules, build with
`UAC_PSTV_ENABLE_LOGGING=ON` in WSL2 or run `.\build.ps1 -Logging` from
PowerShell. Install both modules from `dist/debug/`; after reboot they write to
`ur0:data/uac_pstv.log`. Restore the normal modules from `dist/` afterward.
