# Clip Tools — Development History & Usage Guide

These tools live on the `clip-tools` branch only. They are not part of any UI PR.

---

## Branch Workflow

```
master
  ├── accel-bar-ui    ← UI branch (PR-able, clean)
  └── clip-tools      ← clip tool branch (local dev, based on master)
```

`clip-tools` is rebased on master to stay current:
```bash
git checkout clip-tools && git rebase master && git push --force-with-lease origin clip-tools
```

`test-clip.sh` merges clip tools + any UI branch at test time via git worktrees — the branches
are never combined in git.

---

## History of Changes

### Initial state
The clip tool (`run.py`) could render onroad UI frames from a route, but had no RocketFuel
support. The RocketFuel bar silently skipped rendering every frame because only `TorqueBar`
was set in params — `RocketFuel` was not set.

### Fix: set both params
`clip()` now sets both params before rendering:
```python
ui_state.params.put_bool("TorqueBar", True)
ui_state.params.put_bool("RocketFuel", True)
```
Without `RocketFuel`, the guard at the top of `RocketFuel.render()` exits early every frame.

### Fix: pre-warm filter state
RocketFuel uses two `FirstOrderFilter` instances (rc=0.1, dt=1/20, alpha≈1/3):
- `_accel_filter`: tracks `clip(aEgo / MAX_ACCEL, -1, 1)`
- `_alpha_filter`: tracks engagement (1.0 when engaged, 0.0 when not)

Without pre-warming, both filters start at 0.0 and take several seconds to reach the correct
value, causing an incorrect bar animation at the start of every clip. The fix replays all
`carState` chunks before the clip window through `_accel_filter` to warm it up.

### Added `--stacked`
Renders both c4 (MICI, narrow) and c3x (big, wide) sizes and stacks them vertically using
ffmpeg's `vstack` filter. The `--no-metadata` flag is opt-in, not forced in stacked mode.

### Added `find_rocket_fuel.py`
Simulates both filters over the full route to score windows by `min(pos_variation, neg_variation)`.
This rewards clips where both the green (accel) and red (braking) bars are actively moving —
a window that only brakes would score low because pos_variation would be near zero.

The script prints non-overlapping top-N windows with clip commands for each.

### Added `--best-rf` / `--rf-window`
Instead of downloading the full route again for pre-warm, `--best-rf` uses
`find_rocket_fuel.py` to find the top window and injects the filter state directly:
```python
rf._accel_filter.x, rf._alpha_filter.x = rf_state
```
This gives accurate filter state at the clip start without a separate pre-warm pass.

### RocketFuel direction
Green fills upward (positive acceleration), red fills downward (braking):
- `fg_y = cy - fg_h if accel >= 0 else cy`

`MAX_ACCEL=4.0` means typical driving (1–2 m/s²) fills only 10–25% of bar height.

### Added `test-clip.sh`
Worktree-based isolation: creates a fresh checkout of any UI branch, injects the clip tools
from `clip-tools`, renders the clip, and opens it. The current working tree is never touched.

### Fix: ipc_pyx.so error in worktree (`test-clip.sh`)
Git worktrees check out files but do not initialize submodules, so `msgq_repo/` in the
worktree is an empty directory. The `msgq -> msgq_repo/msgq` symlink is broken, and
`from msgq.visionipc import VisionIpcServer` fails with a missing `ipc_pyx.so` error when
`uv run` uses the worktree's Python environment.

Fix: instead of `cd $WORKTREE && uv run python3 ...`, use the main repo's pre-built venv
with the worktree prepended to `PYTHONPATH`:
```bash
PYTHONPATH="$WORKTREE" "$REPO_ROOT/.venv/bin/python3" "$WORKTREE/tools/clip/run.py" "$@" -o "$OUT"
```
- `PYTHONPATH=$WORKTREE` is checked first → `openpilot.*` imports find the UI branch's files ✓
- Broken symlinks in the worktree (msgq, cereal, opendbc, …) cause Python's `isdir` to return
  False → Python skips them and falls through to the next sys.path entry ✓
- The main repo's `.pth` (loaded by `$REPO_ROOT/.venv`) adds `REPO_ROOT` → `msgq` and other
  submodule symlinks resolve correctly with compiled `.so` files ✓

---

## Machine Setup (one-time per machine)

After cloning and running `uv sync`, compile the msgq Cython extensions:
```bash
scons -j$(sysctl -n hw.ncpu)   # full build (includes msgq)
# or just msgq:
cd msgq_repo && scons -j$(sysctl -n hw.ncpu) && cd ..
```
The compiled files land at `msgq_repo/msgq/ipc_pyx.so` and
`msgq_repo/msgq/visionipc/visionipc_pyx.so`. This step is **not** handled by `uv sync`.

`test-clip.sh` uses the main repo's compiled venv (via `PYTHONPATH`), so you only need to
build once on each machine — not per worktree.

---

## Usage

### `run.py`

```bash
# Demo route, default window
uv run python3 tools/clip/run.py --demo

# Explicit window
uv run python3 tools/clip/run.py <route> -s <start> -e <end> -o /tmp/clip.mp4

# Best RocketFuel window (auto-finds and clips)
uv run python3 tools/clip/run.py <route> --best-rf -o /tmp/clip.mp4

# Stacked (c4 bottom, c3x top)
uv run python3 tools/clip/run.py <route> -s 604 -e 619 --stacked -o /tmp/clip.mp4

# All flags
uv run python3 tools/clip/run.py <route> -s <start> -e <end> \
  -o <output.mp4>       # output path (default: output.mp4)
  -f <MB>               # target file size in MB (default: 9.0)
  -x <speed>            # speed multiplier (default: 1)
  -t "title text"       # title overlay
  --demo                # use demo route (a2a0ccea32023010/2023-07-27--13-01-19)
  --big                 # c3x size (2160x1080); mutually exclusive with --stacked
  --stacked             # render c4 + c3x, stack vertically
  --qcam                # use qcamera instead of fcamera
  --no-metadata         # disable metadata overlay
  --no-time-overlay     # disable time overlay
  --best-rf             # auto-find best RocketFuel window
  --rf-window N         # window size in seconds for --best-rf (default: 15)
```

### `find_rocket_fuel.py`

```bash
# Demo route
uv run python3 tools/clip/find_rocket_fuel.py --demo

# Specific route, custom window and top-N
uv run python3 tools/clip/find_rocket_fuel.py <route> --window 20 --top 10
```

Prints a table of top non-overlapping windows with scores and ready-to-run clip commands.

Demo route best window: 604–619s (score 112.1, balanced pos/neg).

### `test-clip.sh`

```bash
# Default: demo route with --best-rf on the given UI branch
tools/clip/test-clip.sh accel-bar-ui

# Explicit window
tools/clip/test-clip.sh accel-bar-ui a2a0ccea32023010/2023-07-27--13-01-19 -s 604 -e 619 --no-metadata

# Stacked
tools/clip/test-clip.sh accel-bar-ui --best-rf --stacked

# Another branch
tools/clip/test-clip.sh some-other-ui-branch --best-rf
```

Output is saved to `/tmp/clip_test_<branch>_<pid>.mp4` and opened automatically.
