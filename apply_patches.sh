#!/usr/bin/env bash
set -euo pipefail

# Apply all patches in this repo to the current git repository.
# Usage: run this script from inside the target ocs2 git repo.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
patch_dir="$script_dir/patches"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: not inside a git repository. Please cd into the target ocs2 repo." >&2
  exit 2
fi

if [[ ! -d "$patch_dir" ]]; then
  echo "ERROR: patch directory not found: $patch_dir" >&2
  exit 2
fi

# Ensure clean working tree for git am
if [[ -n "$(git status --porcelain)" ]]; then
  echo "ERROR: working tree is not clean. Please commit/stash changes before applying patches." >&2
  git status --porcelain >&2
  exit 2
fi

echo "Applying patches from: $patch_dir"

# Apply in order
for p in "$patch_dir"/*.patch; do
  echo "- git am $p"
  git am "$p"
done

repo_root="$(git rev-parse --show-toplevel)"
mm_root="$repo_root/ocs2_robotic_examples/ocs2_mobile_manipulator_ros"
dst_scripts="$mm_root/scripts"
dst_lib="$mm_root/lib"

src_toolkit="$script_dir/XRoboToolkit-PC-Service-Pybind"
src_lib="$script_dir/lib"

if [[ ! -d "$mm_root" ]]; then
  echo "ERROR: expected target package directory not found: $mm_root" >&2
  exit 2
fi
if [[ ! -d "$src_toolkit" ]]; then
  echo "ERROR: source directory not found: $src_toolkit" >&2
  exit 2
fi
if [[ ! -d "$src_lib" ]]; then
  echo "ERROR: source directory not found: $src_lib" >&2
  exit 2
fi

mkdir -p "$dst_scripts" "$dst_lib"

echo "Copying XRoboToolkit-PC-Service-Pybind into: $dst_scripts"
if command -v rsync >/dev/null 2>&1; then
  rsync -a "$src_toolkit/" "$dst_scripts/$(basename "$src_toolkit")/"
else
  rm -rf "$dst_scripts/$(basename "$src_toolkit")"
  cp -a "$src_toolkit" "$dst_scripts/"
fi

echo "Copying lib contents into: $dst_lib"
if command -v rsync >/dev/null 2>&1; then
  rsync -a "$src_lib/" "$dst_lib/"
else
  cp -a "$src_lib/." "$dst_lib/"
fi

echo "Done."
