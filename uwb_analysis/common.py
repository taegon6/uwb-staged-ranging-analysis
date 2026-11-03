"""Shared helpers: dataset layout, CSV reading, and experiment geometry.

Coordinates
-----------
The logger and solver use a frame that is mirrored in X with respect to the
manuscript (manuscript X = -solver X). The camera reference trajectories and
the manual final-stop measurements already use the manuscript X direction
(positive to the physical left).
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

# Paper experiment number -> run folder name in the dataset (uwb_runs/<name>)
EXPERIMENTS = {
    1: "Exp2_214725_RUN72_VEHICLE_DRIVE_AUTO_R_FINAL_STOP_GT",
    2: "Exp3_215106_RUN72_VEHICLE_DRIVE_AUTO_R_NEXT",
    3: "Exp5_221028_RUN72_VEHICLE_DRIVE_AUTO_R_RUN5",
}

# Runs that end in a stationary high-precision DS-TWR segment. The per-link
# range offsets for an evaluated experiment are obtained from the other runs
# in this list (the evaluated run is always excluded).
STATIONARY_RUNS = [
    "Exp2_214725_RUN72_VEHICLE_DRIVE_AUTO_R_FINAL_STOP_GT",
    "Exp3_215106_RUN72_VEHICLE_DRIVE_AUTO_R_NEXT",
    "Exp4_215932_RUN72_VEHICLE_DRIVE_AUTO_R_PHASE_AGAIN",
    "Exp5_221028_RUN72_VEHICLE_DRIVE_AUTO_R_RUN5",
    "Exp7_221709_RUN72_VEHICLE_DRIVE_AUTO_R_RUN7",
    "Exp8_222010_RUN72_VEHICLE_DRIVE_AUTO_R_RUN8",
]

LINKS = ["T1_A1", "T1_A2", "T2_A1", "T2_A2"]

# Geometry in the solver frame [m]
ANCHORS = {"A1": np.array([-1.0, -3.0]), "A2": np.array([1.0, -3.0])}
TAGS = {"T1": np.array([-0.4, 0.0]), "T2": np.array([0.4, 0.0])}  # relative to the tag midpoint
RX_COIL_OFFSET = np.array([0.0, 2.2])  # RX-coil center relative to the tag midpoint
HEIGHT_DIFFERENCE = 1.11 - 0.81  # anchor height - tag height [m]
TAG_MIDPOINT_TARGET = -RX_COIL_OFFSET  # tag midpoint when the RX coil is centered on the TX coil


def read_csv(path: Path) -> list:
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def num(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def is_true(value) -> bool:
    return str(value).strip().lower() in ("true", "1")


def run_dir(data_root: Path, run: str) -> Path:
    return Path(data_root) / "uwb_runs" / run


def camera_file(data_root: Path, run: str) -> Path:
    return Path(data_root) / "camera_reference" / f"{run}_camera_board_center.csv"


def rotation(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])

