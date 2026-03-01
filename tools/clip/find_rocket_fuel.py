#!/usr/bin/env python3
"""
Find route windows with the most RocketFuel bar motion.

Simulates the two FirstOrderFilter instances used in RocketFuel.render() and
computes total variation of the bar fill across sliding windows.
Top non-overlapping windows are printed sorted by motion score.

Usage:
  uv run python3 tools/clip/find_rocket_fuel.py <route> [--window 15] [--top 5]
  uv run python3 tools/clip/find_rocket_fuel.py --demo
"""
import sys
import logging
import itertools
import multiprocessing
from argparse import ArgumentParser
from concurrent.futures import ThreadPoolExecutor, as_completed

from openpilot.tools.lib.route import Route
from openpilot.tools.lib.filereader import FileReader
from openpilot.tools.lib.logreader import _LogFileReader  # noqa: PLC2701
from openpilot.selfdrive.test.process_replay.migration import migrate_all

FRAMERATE = 20
MAX_ACCEL = 4.0
FILTER_RC = 0.1
FILTER_DT = 1.0 / FRAMERATE
FILTER_ALPHA = FILTER_DT / (FILTER_RC + FILTER_DT)  # ~1/3, matches FirstOrderFilter(rc=0.1, dt=1/20)

DEMO_ROUTE = "a2a0ccea32023010/2023-07-27--13-01-19"

logger = logging.getLogger("find_rocket_fuel")


def parse_args():
  parser = ArgumentParser(description="Find route windows with the most RocketFuel bar motion")
  parser.add_argument("route", nargs="?", help="Route ID")
  parser.add_argument("-d", "--data-dir", help="Local directory with route data")
  parser.add_argument("-w", "--window", type=int, default=15, help="Window size in seconds (default: 15)")
  parser.add_argument("-n", "--top", type=int, default=5, help="Number of top windows to show (default: 5)")
  parser.add_argument("--demo", action="store_true", help="Use demo route")
  args = parser.parse_args()
  if args.demo:
    args.route = args.route or DEMO_ROUTE
  elif not args.route:
    parser.error("route is required (or use --demo)")
  return args


def _download_segment(path: str) -> bytes:
  with FileReader(path) as f:
    return bytes(f.read())


def _parse_segment(raw_data: bytes) -> list[dict]:
  from openpilot.tools.lib.logreader import _LogFileReader
  messages = migrate_all(list(_LogFileReader("", dat=raw_data, sort_by_time=True)))
  if not messages:
    return []

  dt_ns = 1e9 / FRAMERATE
  chunks: list[dict] = []
  current: dict = {}
  next_time = messages[0].logMonoTime + dt_ns
  for msg in messages:
    if msg.logMonoTime >= next_time:
      chunks.append(current)
      current = {}
      next_time += dt_ns * ((msg.logMonoTime - next_time) // dt_ns + 1)
    current[msg.which()] = msg
  if current:
    chunks.append(current)
  return chunks


def load_all_chunks(log_paths: list[str]) -> list[dict]:
  num_workers = min(16, len(log_paths), (multiprocessing.cpu_count() or 1))
  logger.info(f"Downloading {len(log_paths)} segments with {num_workers} workers...")
  with ThreadPoolExecutor(max_workers=num_workers) as pool:
    futures = {pool.submit(_download_segment, p): i for i, p in enumerate(log_paths)}
    raw_data = {futures[f]: f.result() for f in as_completed(futures)}
  logger.info("Parsing segments...")
  with multiprocessing.Pool(num_workers) as pool:
    return list(itertools.chain.from_iterable(pool.map(_parse_segment, [raw_data[i] for i in range(len(log_paths))])))


def extract_timeseries(chunks: list[dict]) -> tuple[list[float], list[bool]]:
  """Extract per-frame aEgo and engaged status from message chunks."""
  last_a_ego = 0.0
  last_engaged = False
  a_ego_series: list[float] = []
  engaged_series: list[bool] = []
  for chunk in chunks:
    if "carState" in chunk:
      last_a_ego = float(chunk["carState"].as_builder().carState.aEgo)
    if "selfdriveState" in chunk:
      last_engaged = bool(chunk["selfdriveState"].as_builder().selfdriveState.enabled)
    a_ego_series.append(last_a_ego)
    engaged_series.append(last_engaged)
  return a_ego_series, engaged_series


def simulate_filters(a_ego_series: list[float], engaged_series: list[bool]) -> list[tuple[float, float]]:
  """Simulate the two FirstOrderFilter instances and return (accel_x, alpha_x) per frame.

  accel_x is signed: positive = acceleration (green bar), negative = braking (red bar).
  """
  accel_x = 0.0
  alpha_x = 0.0
  series: list[tuple[float, float]] = []
  for a_ego, engaged in zip(a_ego_series, engaged_series):
    accel_norm = max(-1.0, min(1.0, a_ego / MAX_ACCEL))
    accel_x = (1.0 - FILTER_ALPHA) * accel_x + FILTER_ALPHA * accel_norm
    alpha_x = (1.0 - FILTER_ALPHA) * alpha_x + FILTER_ALPHA * float(engaged)
    series.append((accel_x, alpha_x))
  return series


def find_top_windows(series: list[tuple[float, float]], window_s: int, top_n: int) -> list[dict]:
  """Find top non-overlapping windows by min(pos_variation, neg_variation).

  Scoring rewards windows where both the green (accel) and red (braking) bars move,
  ensuring the clip demonstrates full bar usage in both directions.
  """
  window_frames = window_s * FRAMERATE
  n = len(series)

  if n < window_frames:
    logger.warning(f"Route has only {n / FRAMERATE:.1f}s of data, less than window size {window_s}s")
    return []

  # Precompute signed bar fills: positive = accel (green), negative = braking (red)
  pos_h = [max(0.0, accel_x) * alpha_x for accel_x, alpha_x in series]
  neg_h = [max(0.0, -accel_x) * alpha_x for accel_x, alpha_x in series]

  candidates = []
  for start in range(0, n - window_frames + 1):
    end = start + window_frames
    pos_variation = sum(abs(pos_h[i] - pos_h[i - 1]) for i in range(start + 1, end))
    neg_variation = sum(abs(neg_h[i] - neg_h[i - 1]) for i in range(start + 1, end))
    score = min(pos_variation, neg_variation)
    engaged_frames = sum(1 for accel_x, alpha_x in series[start:end] if alpha_x > 0.01)
    pos_full_pct = 100.0 * max(pos_h[start:end])
    neg_full_pct = 100.0 * max(neg_h[start:end])
    candidates.append({
      "start_frame": start,
      "end_frame": end,
      "start_s": start / FRAMERATE,
      "end_s": end / FRAMERATE,
      "score": score,
      "pos_motion": pos_variation,
      "neg_motion": neg_variation,
      "pos_full_pct": pos_full_pct,
      "neg_full_pct": neg_full_pct,
      "engaged_pct": 100.0 * engaged_frames / window_frames,
    })

  candidates.sort(key=lambda x: x["score"], reverse=True)

  # Greedily pick non-overlapping windows
  selected = []
  used_frames: set[int] = set()
  for c in candidates:
    frames = set(range(c["start_frame"], c["end_frame"]))
    if not frames & used_frames:
      selected.append(c)
      used_frames |= frames
      if len(selected) >= top_n:
        break

  selected.sort(key=lambda x: x["start_s"])
  return selected


def main():
  logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s\t%(message)s")
  args = parse_args()

  route = Route(args.route, data_dir=args.data_dir)
  log_paths = [p for p in route.log_paths() if p]
  if not log_paths:
    logger.error("No log paths found for route")
    sys.exit(1)

  chunks = load_all_chunks(log_paths)
  logger.info(f"Loaded {len(chunks)} frames ({len(chunks) / FRAMERATE:.1f}s)")

  a_ego_series, engaged_series = extract_timeseries(chunks)
  bar_series = simulate_filters(a_ego_series, engaged_series)

  top_windows = find_top_windows(bar_series, args.window, args.top)
  if not top_windows:
    print("No windows found.")
    return

  print(f"\nTop {len(top_windows)} windows by RocketFuel bar motion (window={args.window}s):")
  print(f"  Score = min(pos, neg) — rewards windows with both green (accel) and red (braking) bar activity")
  print(f"  Full% = peak bar fill as % of max (100% = aEgo sustained at ±{MAX_ACCEL} m/s² while engaged)")
  print(f"{'Rank':<6} {'Start':>8} {'End':>8} {'Score':>8} {'Pos(+)':>8} {'Neg(-)':>8} {'Pos Full%':>10} {'Neg Full%':>10} {'Engaged%':>10}  Clip command")
  print("-" * 130)
  for rank, w in enumerate(top_windows, 1):
    start_s = int(w['start_s'])
    end_s = int(w['end_s'])
    clip_cmd = f"uv run python3 tools/clip/run.py {args.route} -s {start_s} -e {end_s} -o /tmp/clip.mp4"
    print(f"{rank:<6} {start_s:>7}s {end_s:>7}s {w['score']:>8.1f} {w['pos_motion']:>8.1f} {w['neg_motion']:>8.1f} {w['pos_full_pct']:>9.1f}% {w['neg_full_pct']:>9.1f}% {w['engaged_pct']:>9.1f}%  {clip_cmd}")

  print()


if __name__ == "__main__":
  main()
