# Staged UWB ranging for EV wireless charging alignment: analysis code

This repository contains the analysis code for the manuscript

> T.-g. Lee, D.-w. Seo, and B.-J. Jang, "Distance-Based Switching of UWB Ranging for Electric Vehicle Wireless Charging Alignment," submitted to *Energies* (MDPI), 2026.

The code computes the values reported in Table 3 of the manuscript from the experimental UWB logs and camera reference trajectories:

- position update rate of each ranging mode (SS-TWR, DS-TWR, and high-precision DS-TWR),
- lateral RMSE of SS-TWR and DS-TWR during the approach, and
- final two-dimensional tag-midpoint estimation error at the stop with high-precision DS-TWR.

## Data

The experimental data are not included in this repository. They are available on request from the corresponding author (Byung-Jun Jang, bjjang@kookmin.ac.kr). The expected folder layout is:

```
minimal_dataset/
  uwb_runs/<run>/tables/frames.csv, links.csv, poses.csv, manual_measurements.csv
  uwb_runs/<run>/config/geometry.json, capture_config.json, firmware_info.json
  camera_reference/<run>_camera_board_center.csv
```

Experiments 1, 2, and 3 in the manuscript correspond to the runs `Exp2_...`, `Exp3_...`, and `Exp5_...`. The runs `Exp4_...`, `Exp7_...`, and `Exp8_...` also end at a stationary position and are used only to obtain the per-link range offsets.

## Usage

Python 3.8 or later and NumPy are required.

```
pip install -r requirements.txt
python reproduce_table3.py --data /path/to/minimal_dataset
```

Add `--json results.json` to save the detailed results, including the number of paired samples, the estimated positions, and the per-link offsets.

## Method summary

**Update rate** (`uwb_analysis/driving_metrics.py`): the number of valid poses divided by the elapsed time of each mode segment. A segment is split where consecutive valid poses are more than 1 s apart. Depending on how the segment boundaries are defined, the values can differ from those in the manuscript by up to about 1%.

**Lateral RMSE** (`uwb_analysis/driving_metrics.py`): each camera sample is paired with the nearest valid UWB pose of the same ranging mode within 50 ms. The camera trajectory of the ChArUco board center is a relative trajectory, so the mean deviation of each experiment and mode is removed, and the RMSE of the remaining lateral deviation is reported (n - 1 normalization).

**Final stationary error** (`uwb_analysis/final_stop.py`): the per-link median of the phase-corrected ranges in the last 3 s of the final high-precision DS-TWR segment is corrected by constant per-link range offsets, converted to horizontal ranges, and used to solve the 2A2T pose with Gauss-Newton iterations. For each evaluated experiment, the offsets are obtained from the other stationary runs at their manually measured positions. The error is the 2-D distance between the estimated and manually measured tag midpoints.

## Coordinates

The logger and solver frame is mirrored in X with respect to the manuscript (manuscript X = -solver X). The camera reference and the manual measurements use the manuscript X direction (positive to the physical left).

## License

MIT License. See `LICENSE`.

