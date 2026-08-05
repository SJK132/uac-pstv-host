# Contributing

UAC PSTV is deliberately narrow kernel code. Changes should preserve the fixed
48 kHz, stereo, 16-bit transport unless a separate, measured extension is being
proposed.

Before opening a pull request:

```bash
make -C tests clean test race
bash ./build.sh
```

Keep hot paths allocation-free, avoid logging in release builds, and document
any new kernel import or hook. Changes to USB request lifetime, callbacks, or
module stop paths should include a deterministic host test when possible.

Device reports should include:

- PSTV firmware and Ensō/YAMT version
- USB VID:PID and full UAC descriptors
- Storage and USB-related plugins
- Relevant `boot_config.txt` and `config.txt` ordering
- Whether cold boot, reboot, and hot-plug differ
- `ur0:data/uac_pstv.log` from a logging-enabled build

