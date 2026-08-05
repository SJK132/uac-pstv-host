# Building UAC PSTV on Windows with WSL2

Prebuilt modules are available in `dist/`. These instructions are for people
who want to reproduce or modify the build.

## 1. Install WSL2

From an Administrator PowerShell window:

```powershell
wsl --install -d Ubuntu
```

Restart Windows if prompted, launch Ubuntu, and create the Linux account.

## 2. Install the build prerequisites

Inside Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git wget patch p7zip-full tar libarchive-tools
```

`libarchive-tools` provides `bsdtar`, which Vita package builds use to extract
source archives.

## 3. Install VitaSDK

```bash
git clone https://github.com/vitasdk/vdpm.git ~/vdpm
cd ~/vdpm
./bootstrap-vitasdk.sh

echo 'export VITASDK=/usr/local/vitasdk' >> ~/.bashrc
echo 'export PATH=$VITASDK/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

./install-all.sh
```

Verify the installation:

```bash
arm-vita-eabi-gcc --version
vita-make-fself --help
```

## 4. Install the Vita USB host package

UAC PSTV uses the additional `vitausb` host headers and stubs:

```bash
git clone https://github.com/isage/vita-packages-extra.git ~/vita-packages-extra
cd ~/vita-packages-extra/vitausb
vita-makepkg
vdpm vitausb-*.tar.xz
```

## 5. Build and test

From the repository directory mounted in WSL:

```bash
bash ./build.sh
```

The script runs the native correctness and deterministic drain/race tests,
builds the two kernel modules, verifies that release VELFs contain no logging
symbols/imports/strings, and copies them to `dist/`. Logging-enabled builds go
to `dist/debug/` instead. Its CMake build tree
defaults to `/tmp/uac-pstv-build`; keeping generated VitaSDK files off `/mnt/c`
avoids DrvFS permission limitations.

Useful overrides:

```bash
JOBS=4 bash ./build.sh
BUILD_DIR="$HOME/uac-pstv-build" bash ./build.sh
UAC_PSTV_ENABLE_LOGGING=ON bash ./build.sh
```

The PowerShell wrapper is equivalent:

```powershell
.\build.ps1
.\build.ps1 -Logging
```

## Manual commands

```bash
export VITASDK=/usr/local/vitasdk
export PATH="$VITASDK/bin:$PATH"

make -C tests clean test race
cmake -S . -B /tmp/uac-pstv-build \
  -DCMAKE_TOOLCHAIN_FILE="$VITASDK/share/vita.toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAC_PSTV_ENABLE_LOGGING=OFF
cmake --build /tmp/uac-pstv-build -j2
```

Expected artifacts:

```text
/tmp/uac-pstv-build/uac_pstv_boot.skprx
/tmp/uac-pstv-build/uac_pstv_audio.skprx
```

## Release checks

Before publishing a binary release:

1. Run both host tests.
2. Build with logging explicitly set to `OFF`.
3. Confirm both `.skprx` files are present in `dist/`.
4. Update `dist/SHA256SUMS.txt`.
5. Test cold boot, reboot, hot-plug, game audio, LiveArea effects, and BGM on a
   PSTV with a recovery method available.
