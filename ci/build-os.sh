#!/usr/bin/env bash
set -euo pipefail
# All three commands must share one container and one OS-specific workspace.
graph=$1
recipe=$2
os=$3
rebuild=()
[[ "$4" != true ]] || rebuild+=(--rebuild-all)
export CCACHE_DIR="$PWD/ccache"
mkdir -p "$CCACHE_DIR"
build_packages --graph "$graph" --workspace workspace --recipe "$recipe" --ros-distro ros1 \
  --build-report "build-report-$os-ros1.yaml" "${rebuild[@]}"
build_packages --graph "$graph" --workspace workspace --recipe "$recipe" --ros-distro ros2 \
  --build-report "build-report-$os-ros2.yaml" "${rebuild[@]}"
build_bundles --graph "$graph" --workspace workspace --recipe "$recipe" "${rebuild[@]}"
