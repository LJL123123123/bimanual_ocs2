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

echo "Done."
