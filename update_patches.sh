#!/usr/bin/env bash
set -euo pipefail

# Regenerate patches/ from a local ocs2 git repo.
# This script is meant to be run from inside this patch repo.

usage() {
  cat <<'EOF'
Usage:
  ./update_patches.sh [--ocs2 <path-to-ocs2-repo>] [--base <base-commit>]

Behavior:
  - Validates the ocs2 repo exists and is clean (no uncommitted changes)
  - Validates <base-commit> is an ancestor of ocs2/HEAD
  - Deletes existing patches/*.patch
  - Recreates patches using: git format-patch --binary <base>..HEAD

Defaults:
  --ocs2 defaults to "../src/ocs2" relative to this patch repo.
  --base defaults to the commit hash stored in ./.base_commit

Notes:
  - If you have new local changes, commit them in the ocs2 repo first.
  - This script does not git-commit changes in the patch repo.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
patch_dir="$script_dir/patches"
base_file="$script_dir/.base_commit"

default_ocs2="$script_dir/../ocs2"
ocs2_repo="$default_ocs2"
base_commit=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ocs2)
      ocs2_repo="${2:-}"; shift 2;;
    --base)
      base_commit="${2:-}"; shift 2;;
    -h|--help)
      usage; exit 0;;
    *)
      echo "ERROR: unknown arg: $1" >&2
      usage
      exit 2;;
  esac
done

if [[ -z "$base_commit" ]]; then
  if [[ -f "$base_file" ]]; then
    base_commit="$(tr -d ' \t\r\n' < "$base_file")"
  fi
fi

if [[ -z "$base_commit" ]]; then
  echo "ERROR: base commit not provided and $base_file missing/empty." >&2
  echo "       Provide --base <sha> or write the sha into $base_file" >&2
  exit 2
fi

if [[ ! -d "$ocs2_repo/.git" ]]; then
  echo "ERROR: ocs2 repo not found: $ocs2_repo" >&2
  echo "       Provide --ocs2 <path-to-ocs2-repo>" >&2
  exit 2
fi

mkdir -p "$patch_dir"

# Validate base commit exists and is a commit
if ! git -C "$ocs2_repo" cat-file -e "$base_commit^{commit}" 2>/dev/null; then
  echo "ERROR: base commit not found in ocs2 repo: $base_commit" >&2
  exit 2
fi

# Ensure clean working tree in ocs2 repo so patches match commits
if [[ -n "$(git -C "$ocs2_repo" status --porcelain)" ]]; then
  echo "ERROR: ocs2 working tree is not clean: $ocs2_repo" >&2
  echo "       Commit/stash your changes first, then re-run." >&2
  git -C "$ocs2_repo" status --porcelain >&2
  exit 2
fi

head_commit="$(git -C "$ocs2_repo" rev-parse HEAD)"

if ! git -C "$ocs2_repo" merge-base --is-ancestor "$base_commit" "$head_commit"; then
  echo "ERROR: base commit is not an ancestor of HEAD." >&2
  echo "       base=$base_commit" >&2
  echo "       head=$head_commit" >&2
  echo "       Make sure you generated changes on top of the documented base." >&2
  exit 2
fi

# Remove old patches
rm -f "$patch_dir"/*.patch

echo "Generating patches into: $patch_dir"
echo "- ocs2 repo: $ocs2_repo"
echo "- base:     $base_commit"
echo "- head:     $head_commit"

# Generate new patches
# --binary: include binary blobs when needed
# --full-index: stable, complete blob ids
# --no-stat: cleaner patch files
# -o: output directory
# We keep the default numbering (0001-...) so apply_patches.sh can glob in order.
git -C "$ocs2_repo" format-patch \
  --binary \
  --full-index \
  --no-stat \
  -o "$patch_dir" \
  "$base_commit".."$head_commit" >/dev/null

patch_count="$(ls -1 "$patch_dir"/*.patch 2>/dev/null | wc -l)"
if [[ "$patch_count" -eq 0 ]]; then
  echo "ERROR: no patches generated. Is HEAD equal to base, or did you squash changes into the base commit?" >&2
  exit 2
fi

echo "Done. Generated $patch_count patch(es):"
ls -1 "$patch_dir"/*.patch

cat <<EOF

Next:
  1) Review patches under: $patch_dir
  2) Commit & push changes in this patch repo:
     git add patches .base_commit update_patches.sh README.md
     git commit -m "Update patches"
     git push
EOF
