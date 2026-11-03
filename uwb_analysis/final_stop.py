"""Final stationary tag-midpoint estimation (Table 3, last row).

1. Take the last high-precision DS-TWR segment of each run (split at host-time
   gaps larger than 1 s) and keep its last 3 s.
2. Use the per-link median of the phase-corrected ranges in that window.
3. Subtract constant per-link range offsets (antenna delay). For the
   evaluated run, the offsets are the mean difference between the measured
   ranges and the ranges computed from the manually measured stop position
   (heading 0 deg) of the other stationary runs.
4. Convert the slant ranges to horizontal ranges with the anchor-tag height
   difference and solve the 2A2T pose (tag midpoint and heading) with
   Gauss-Newton iterations.
5. The error is the 2-D distance between the estimated tag midpoint and the
   manually measured tag midpoint.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np

from .common import (ANCHORS, HEIGHT_DIFFERENCE, LINKS, STATIONARY_RUNS, TAG_MIDPOINT_TARGET, TAGS,
                     is_true, read_csv, rotation, run_dir)

WINDOW_S = 3.0
SEGMENT_GAP_NS = 1e9


def model(q: np.ndarray) -> np.ndarray:
    rot = rotation(q[2] if len(q) == 3 else 0.0)
    return np.array([np.linalg.norm(q[:2] + rot @ TAGS[link[:2]] - ANCHORS[link[3:]]) for link in LINKS])


def solve_pose(ranges: np.ndarray, fixed_yaw: bool = False) -> tuple:
    """Gauss-Newton solution of the 2A2T pose from four horizontal ranges."""
    starts = [0.0] if fixed_yaw else np.deg2rad([-20.0, 0.0, 20.0])
    solutions = []
    for angle in starts:
        q = np.array([0.0, -0.8]) if fixed_yaw else np.array([0.0, -0.8, angle])
        for _ in range(100):
            pred = model(q)
            jac = np.zeros((4, len(q)))
            for j in range(len(q)):
                delta = np.zeros(len(q))
                delta[j] = 1e-6
                jac[:, j] = (model(q + delta) - pred) / 1e-6
            step = np.linalg.lstsq(jac, ranges - pred, rcond=None)[0]
            cost = np.linalg.norm(ranges - pred)
            alpha = 1.0
            while alpha > 1 / 1024 and np.linalg.norm(ranges - model(q + alpha * step)) > cost:
                alpha /= 2
            q = q + alpha * step
            if np.linalg.norm(alpha * step) < 1e-10:
                break
        yaw = float(q[2]) if len(q) == 3 else 0.0
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        midpoint = q[:2]
        # keep the solution on the vehicle side of the anchor baseline
        if all((midpoint + rotation(yaw) @ b)[1] > -3.0 for b in TAGS.values()):
            solutions.append((float(np.linalg.norm(ranges - model(q))), midpoint, yaw))
    if not solutions:
        raise ValueError("no solution on the vehicle side of the anchor baseline")
    residual, midpoint, yaw = min(solutions, key=lambda s: s[0])
    return midpoint, yaw, residual / 2


def slant_ranges(midpoint: np.ndarray, yaw: float = 0.0) -> np.ndarray:
    return np.array([
        math.sqrt(np.linalg.norm(midpoint + rotation(yaw) @ TAGS[link[:2]] - ANCHORS[link[3:]]) ** 2
                  + HEIGHT_DIFFERENCE ** 2)
        for link in LINKS
    ])


def horizontal(ranges: np.ndarray) -> np.ndarray:
    squared = ranges ** 2 - HEIGHT_DIFFERENCE ** 2
    if np.any(squared < 0):
        raise ValueError("range shorter than the height difference")
    return np.sqrt(squared)


def manual_tag_midpoint(data_root: Path, run: str) -> np.ndarray:
    """Manually measured tag midpoint in the solver frame [m]."""
    row = read_csv(run_dir(data_root, run) / "tables" / "manual_measurements.csv")[0]
    values = re.findall(r"-?\d+(?:\.\d+)?", row["reported_value"])
    displacement = np.array([float(v) for v in values[:2]]) / 100
    displacement[0] *= -1  # manual X is positive to the left; solver X is positive to the right
    return TAG_MIDPOINT_TARGET + displacement


def final_window_ranges(data_root: Path, run: str) -> tuple:
    """Per-link median phase-corrected range in the last 3 s of the final segment."""
    rows = [r for r in read_csv(run_dir(data_root, run) / "tables" / "links.csv")
            if r["protocol"] == "PHASE" and is_true(r["phase_valid"]) and r["success"] == "1"]
    rows.sort(key=lambda r: int(r["host_wall_ns"]))
    start = len(rows) - 1
    while start > 0 and int(rows[start]["host_wall_ns"]) - int(rows[start - 1]["host_wall_ns"]) <= SEGMENT_GAP_NS:
        start -= 1
    end_ns = int(rows[-1]["host_wall_ns"])
    tail = [r for r in rows[start:] if int(r["host_wall_ns"]) >= end_ns - WINDOW_S * 1e9]
    medians, counts = [], []
    for link in LINKS:
        values = [float(r["phase_corrected_range_m"]) for r in tail if r["link_id"] == link]
        if not values:
            raise ValueError(f"no final-stop ranges for {run}/{link}")
        medians.append(float(np.median(values)))
        counts.append(len(values))
    return np.array(medians), counts


def evaluate(data_root: Path, runs: list, calibrated: bool = True, fixed_yaw: bool = False) -> list:
    measured = {r: final_window_ranges(data_root, r)[0] for r in STATIONARY_RUNS}
    reference = {r: manual_tag_midpoint(data_root, r) for r in STATIONARY_RUNS}
    results = []
    for run in runs:
        others = [r for r in STATIONARY_RUNS if r != run]
        if calibrated:
            offsets = np.mean([measured[r] - slant_ranges(reference[r]) for r in others], axis=0)
        else:
            offsets = np.zeros(len(LINKS))
        midpoint, yaw, residual = solve_pose(horizontal(measured[run] - offsets), fixed_yaw)
        results.append({
            "run": run,
            "tag_midpoint_error_cm": float(np.linalg.norm(midpoint - reference[run]) * 100),
            "estimated_tag_midpoint_solver_m": midpoint.tolist(),
            "reference_tag_midpoint_solver_m": reference[run].tolist(),
            "estimated_yaw_deg": math.degrees(yaw),
            "range_residual_rms_cm": residual * 100,
            "link_offsets_cm": dict(zip(LINKS, (offsets * 100).tolist())),
            "calibration_runs": others if calibrated else [],
        })
    return results

