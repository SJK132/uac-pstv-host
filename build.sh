#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${BUILD_DIR:-/tmp/uac-pstv-build}"

: "${VITASDK:=/usr/local/vitasdk}"
export VITASDK
export PATH="$VITASDK/bin:$PATH"

rm -rf "$build_dir"
cmake -S "$project_dir" -B "$build_dir" \
  -DCMAKE_TOOLCHAIN_FILE="$VITASDK/share/vita.toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j2

mkdir -p "$project_dir/dist"
cp "$build_dir/uac_pstv.skprx" "$project_dir/dist/uac_pstv.skprx"
(
  cd "$project_dir/dist"
  sha256sum uac_pstv.skprx > SHA256SUMS.txt
)
