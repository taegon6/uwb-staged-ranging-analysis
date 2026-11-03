"""Position update rate and lateral RMSE during the approach (Table 3).

Update rate
    Number of valid poses divided by the elapsed time of each mode segment.
    A mode segment is split where consecutive valid poses are more than 1 s
    apart.

Lateral RMSE
    Each camera reference sample is paired with the nearest valid UWB pose of
    the same ranging mode within 50 ms. The lateral deviation is the
    UWB-estimated RX-coil X minus the camera board-center X. Because the
    camera trajectory is a relative trajectory, the mean deviation of each
    experiment and mode is removed, and the RMSE of the remaining deviation
    is reported (sample normalization, n - 1).
"""
from __future__ import annotations

import bisect
import math
from pathlib import Path

import numpy as np

from .common import camera_file, is_true, num, read_csv, run_dir

MODES = ("SS", "DS", "PHASE")
MAX_PAIR_DT_S = 0.05
SEGMENT_GAP_S = 1.0


def load_valid_poses(data_root: Path, run: str) -> list:
    """Return sorted (time_s, mode, rx_coil_x_manuscript_m) for valid poses."""
    tables = run_dir(data_root, run) / "tables"
    modes = {r["frame_id"]: r["mode"].upper() for r in read_csv(tables / "frames.csv")}
    poses = []
    for row in read_csv(tables / "poses.csv"):
        if not is_true(row["valid"]):
            continue
        mode = modes.get(row["frame_id"])
        t = num(row["host_pose_wall_ns"]) / 1e9
        x = num(row["rx_coil_x_m"])
        if mode in MODES and math.isfinite(t) and math.isfinite(x):
            poses.append((t, mode, -x))  # solver X -> manuscript X
    poses.sort()
    return poses


def update_rate(poses: list, mode: str) -> float:
    times = [p[0] for p in poses if p[1] == mode]
    if len(times) < 2:
        return float("nan")
    segments = []
    start = prev = times[0]
    count = 1
    for t in times[1:]:
        if t - prev > SEGMENT_GAP_S:
            segments.append((start, prev, count))
            start, count = t, 0
        prev = t
        count += 1
    segments.append((start, prev, count))
    return sum(s[2] for s in segments) / sum(s[1] - s[0] for s in segments)


def lateral_rmse_cm(poses: list, camera_rows: list, mode: str) -> tuple:
    """Return (mean-removed lateral RMSE [cm], number of pairs)."""
    uwb = [p for p in poses if p[1] == mode]
    times = [p[0] for p in uwb]
    deviations = []
    for row in camera_rows:
        if row["mode"].upper() != mode:
            continue
        ct = num(row["uwb_aligned_timestamp_ns"]) / 1e9
        cx = num(row["x_cam_left_gp_m"])
        if not (math.isfinite(ct) and math.isfinite(cx)):
            continue
        k = bisect.bisect_left(times, ct)
        candidates = [j for j in (k - 1, k) if 0 <= j < len(uwb)]
        if not candidates:
            continue
        j = min(candidates, key=lambda i: abs(times[i] - ct))
        if abs(times[j] - ct) < MAX_PAIR_DT_S:
            deviations.append(uwb[j][2] - cx)
    if len(deviations) < 2:
        return float("nan"), len(deviations)
    return float(np.std(deviations, ddof=1) * 100), len(deviations)


def evaluate_run(data_root: Path, run: str) -> dict:
    poses = load_valid_poses(data_root, run)
    camera_rows = read_csv(camera_file(data_root, run))
    result = {"rate_hz": {m: update_rate(poses, m) for m in MODES}, "lateral_rmse_cm": {}, "pairs": {}}
    for mode in ("SS", "DS"):
        rmse, pairs = lateral_rmse_cm(poses, camera_rows, mode)
        result["lateral_rmse_cm"][mode] = rmse
        result["pairs"][mode] = pairs
    return result

