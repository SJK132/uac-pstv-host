# UAC PSTV installation

The release contains one module and does not modify `boot_config.txt`.

1. Copy `uac_pstv.skprx` to `ur0:tai/`.
2. Add it once under `*KERNEL` in `ur0:tai/config.txt`.
3. Remove obsolete `uac_pstv_boot.skprx`, `uac_pstv_audio.skprx`, and
   `vita_uac_host.skprx` entries, including any old helper line in
   `ur0:tai/boot_config.txt`.
4. Reboot with the adapter connected. Hot-plugging is also supported.

## StorageMgr

StorageMgr is conditionally supported. UAC PSTV must appear above it:

```text
*KERNEL
ur0:tai/uac_pstv.skprx
ur0:tai/storagemgr.skprx
```

Tested `storage_config.txt`:

```text
INT=imc0
GCD=ux0
UMA=uma0
```

## YAMT

YAMT's tested delayed USB startup works with the normal UAC plugin; no UAC
boot helper is required. Leave YAMT's own boot entry unchanged. Do not enable
experimental or forced-legacy USB mass-storage modes while using USB audio.

This release is PSTV-only and accepts UAC1 stereo, signed 16-bit, 48 kHz OUT
interfaces. It does not support UAC2. Do not use USB audio and USB mass storage
on the same physical PSTV port simultaneously.

## Logging

The release module does **not** create `ur0:data/uac_pstv.log`; logging is
compiled out. Diagnostic builds are written to `dist/debug/`. Restore the
normal module from `dist/` afterward.
