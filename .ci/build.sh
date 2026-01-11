#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-build}"
configuration="${2:-Release}"
extra_args=()
if [[ -n "${EXTRA_CMAKE_ARGS:-}" ]]; then
  # shellcheck disable=SC2206
  extra_args=(${EXTRA_CMAKE_ARGS})
fi

cmake -S "$repo_root" -B "$repo_root/$build_dir" -DSTUDIOMDL_REQUIRE_32BIT=OFF -DCMAKE_BUILD_TYPE="$configuration" "${extra_args[@]}"
cmake --build "$repo_root/$build_dir" --config "$configuration" --parallel
