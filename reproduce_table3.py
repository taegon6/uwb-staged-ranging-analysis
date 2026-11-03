"""Reproduce Table 3 of the manuscript from the minimal dataset.

Usage:
    python reproduce_table3.py --data /path/to/minimal_dataset
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from uwb_analysis.common import EXPERIMENTS
from uwb_analysis.driving_metrics import evaluate_run
from uwb_analysis.final_stop import evaluate as evaluate_final_stop


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", required=True, type=Path,
                        help="minimal dataset folder (contains uwb_runs/ and camera_reference/)")
    parser.add_argument("--json", type=Path, help="optional path to save the detailed results")
    args = parser.parse_args()

    runs = [EXPERIMENTS[k] for k in sorted(EXPERIMENTS)]
    driving = [evaluate_run(args.data, run) for run in runs]
    final_stop = evaluate_final_stop(args.data, runs)

    rows = [
        ("SS-TWR position update rate [Hz]", [d["rate_hz"]["SS"] for d in driving]),
        ("SS-TWR lateral RMSE, mean deviation removed [cm]", [d["lateral_rmse_cm"]["SS"] for d in driving]),
        ("DS-TWR position update rate [Hz]", [d["rate_hz"]["DS"] for d in driving]),
        ("DS-TWR lateral RMSE, mean deviation removed [cm]", [d["lateral_rmse_cm"]["DS"] for d in driving]),
        ("High-precision DS-TWR position update rate [Hz]", [d["rate_hz"]["PHASE"] for d in driving]),
        ("High-precision DS-TWR final 2-D tag-midpoint error [cm]", [f["tag_midpoint_error_cm"] for f in final_stop]),
    ]
    print(f"{'Metric':58s}{'Exp. 1':>9s}{'Exp. 2':>9s}{'Exp. 3':>9s}{'Mean':>9s}")
    for name, values in rows:
        print(f"{name:58s}" + "".join(f"{v:9.2f}" for v in values) + f"{np.mean(values):9.2f}")

    if args.json:
        args.json.write_text(json.dumps({"driving": dict(zip(runs, driving)), "final_stop": final_stop}, indent=2),
                             encoding="utf-8")


if __name__ == "__main__":
    main()

