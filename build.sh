#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${BUILD_DIR:-/tmp/uac-pstv-build}"
jobs="${JOBS:-2}"
logging="${UAC_PSTV_ENABLE_LOGGING:-OFF}"

: "${VITASDK:=/usr/local/vitasdk}"
export VITASDK
export PATH="$VITASDK/bin:$PATH"

if [[ ! -f "$VITASDK/share/vita.toolchain.cmake" ]]; then
  echo "VitaSDK was not found at $VITASDK" >&2
  echo "Install VitaSDK or set VITASDK to its absolute path." >&2
  exit 1
fi

cleanup() {
  make -C "$project_dir/tests" clean >/dev/null 2>&1 || true
}
trap cleanup EXIT

make -C "$project_dir/tests" clean test race

cmake -S "$project_dir" -B "$build_dir" \
  -DCMAKE_TOOLCHAIN_FILE="$VITASDK/share/vita.toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUAC_PSTV_ENABLE_LOGGING="$logging"
cmake --build "$build_dir" -j"$jobs"

if [[ "$logging" == "OFF" ]]; then
  image="$build_dir/uac_pstv_kernel.velf"
  if arm-vita-eabi-readelf -Ws "$image" |
      grep -Eq 'uac_log|SceDebugForDriver|SceIofilemgrForDriver'; then
    echo "Release verification failed: logging symbol or import in $image" >&2
    exit 1
  fi
  if strings "$image" |
      grep -Eq 'uac_pstv\.log|\[uac-pstv'; then
    echo "Release verification failed: logging string in $image" >&2
    exit 1
  fi
  output_dir="$project_dir/dist"
else
  output_dir="$project_dir/dist/debug"
fi

mkdir -p "$output_dir"
cp "$build_dir/uac_pstv.skprx" "$output_dir/"
rm -f "$output_dir/uac_pstv_audio.skprx" \
  "$output_dir/uac_pstv_boot.skprx"

if [[ "$logging" == "OFF" ]]; then
  (
    cd "$output_dir"
    sha256sum uac_pstv.skprx > SHA256SUMS.txt
  )
fi

echo "Built artifacts in $output_dir"
