#!/usr/bin/env bash
# test-clip.sh — test any UI branch's clip rendering in isolation via a git worktree.
# The current working tree is never modified. Clip tools from clip-tools are injected.
#
# Usage:
#   tools/clip/test-clip.sh <ui-branch> [route] [-s start -e end] [--stacked] [--no-metadata]
#   tools/clip/test-clip.sh <ui-branch> --best-rf [--rf-window N]
#
# If no route/window args are given, uses the demo route with --best-rf.
# Output: /tmp/clip_test_<branch>_<pid>.mp4, opened automatically.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <ui-branch> [route] [-s start -e end] [--stacked] [--no-metadata]"
  echo "       $0 <ui-branch> --best-rf [--rf-window N]"
  exit 1
fi

BRANCH="$1"
shift

# Sanitize branch name for use in filename (replace / and other chars with -)
BRANCH_SAFE="${BRANCH//\//-}"
OUT="/tmp/clip_test_${BRANCH_SAFE}_$$.mp4"
WORKTREE="/tmp/clip_worktree_${BRANCH_SAFE}_$$"

# Default: demo route with --best-rf when no additional args given
if [[ $# -eq 0 ]]; then
  set -- --best-rf
fi

# Cleanup on exit
cleanup() {
  if [[ -d "$WORKTREE" ]]; then
    echo "Cleaning up worktree $WORKTREE..."
    git -C "$REPO_ROOT" worktree remove --force "$WORKTREE" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# Create worktree from the UI branch
echo "Creating worktree for branch '$BRANCH' at $WORKTREE..."
git -C "$REPO_ROOT" worktree add "$WORKTREE" "$BRANCH"

# Inject clip tools from clip-tools branch (our modified versions)
echo "Injecting clip tools..."
cp "$SCRIPT_DIR/run.py" "$WORKTREE/tools/clip/run.py"
cp "$SCRIPT_DIR/find_rocket_fuel.py" "$WORKTREE/tools/clip/find_rocket_fuel.py"

# Run clip tool: use main repo's venv + PYTHONPATH pointing to the worktree.
# PYTHONPATH=$WORKTREE shadows openpilot.* imports with the UI branch's files.
# Broken submodule symlinks in the worktree (msgq, cereal, etc.) are skipped by
# Python's isdir check, so they fall through to the main repo where .so files
# are compiled. No submodule init or uv sync needed in the worktree.
echo "Rendering clip to $OUT..."
PYTHONPATH="$WORKTREE" "$REPO_ROOT/.venv/bin/python3" "$WORKTREE/tools/clip/run.py" "$@" -o "$OUT"

echo "Done: $OUT"
open "$OUT" 2>/dev/null || echo "(open not available — file at $OUT)"
