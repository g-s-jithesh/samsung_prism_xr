# Path Finder — Samsung PRISM XR Hackathon 2026 (Track 03)

Estimate a 6-DoF trajectory (position + orientation) from **degraded IMU data only**, then
dead-reckon **5 s** after the IMU fails completely.

**Final submission: 22 Nov 2026, 11:59 PM IST** (target ~8 PM).

## Hard constraints
| Constraint | Budget | Score weight |
|---|---|---|
| Peak RAM | ≤ 8 MB | 20% |
| Runtime | ≤ 20 ms compute per 1 s of IMU data | 20% |
| Trajectory (evo ATE/RPE) | waypoints 30%, blind 5 s 20%, dropouts 10% | 60% |

Single CPU thread, fully offline.

## Tech stack
| Part | Choice | Why |
|---|---|---|
| Submission | **C++17**, `<cstdio>` + `<cmath>` only | CPython alone is ~8–10 MB RSS; numpy adds ~15–20 MB |
| I/O | `fgets` + `strtod`, one row at a time | O(1) memory regardless of sequence length |
| State | fixed-size (q, v, p, gyro/accel bias, last sample) | no heap growth |
| Algorithm sandbox | Python + numpy in `ref/` | fast iteration; not submitted |
| Evaluation | `evo` (ATE/RPE), `/usr/bin/time -v` (peak RSS) | matches organizer scoring |
| Packaging | Docker (gcc) | reproducible build |

Rules for `src/main.cpp`: no iostream, no STL containers, no exceptions, no threads.

## Build & run
```bash
g++ -O2 -std=c++17 -o pathfinder src/main.cpp
./pathfinder data/V1_01_easy/mav0/imu0/data.csv out/V1_01_easy.txt
/usr/bin/time -v ./pathfinder <in.csv> <out.txt>   # "Maximum resident set size" = peak RAM
```

## Evaluate
```bash
pip install numpy evo
python tools/gt_to_tum.py data/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv out/gt_V1_01_easy.txt
evo_ape tum out/gt_V1_01_easy.txt out/V1_01_easy.txt -va --align   # ATE (also run without --align)
evo_rpe tum out/gt_V1_01_easy.txt out/V1_01_easy.txt -va --align   # RPE
```
Record every experiment as a row in `RESULTS.md` (ATE/RPE, runtime, peak RAM).

## Formats
- **Input** (EuRoC `imu0/data.csv`): `timestamp_ns, wx, wy, wz, ax, ay, az` — rad/s, m/s² (includes gravity).
- **Output** (TUM, `estimated_trajectory.txt`): `timestamp_s tx ty tz qx qy qz qw` —
  exactly 8 columns, **one pose per input row** (+ blind window), unit quaternion, monotonic time.

## Data (local only, never commit)
Put sequences under `data/` (gitignored):
- EuRoC Vicon Room 1: `V1_01_easy`, `V1_02_medium`, `V1_03_difficult` — GT `mav0/state_groundtruth_estimate0/data.csv` (q is w-first)
- TUM-VI `room1`–`room3` (512_16 EuRoC export) — GT `mav0/mocap0/data.csv`

## Layout
```
src/main.cpp      # submitted estimator
src/README.md     # build/run notes for the bundle
ref/              # Python reference filter (dev only)
tools/            # gt_to_tum.py, degrade.py, validate.py, evaluate.sh
RESULTS.md        # experiment log, feeds the deck
Dockerfile
```

## Team & work split
See [issues](https://github.com/g-s-jithesh/samsung_prism_xr/issues).

| Who | Owns |
|---|---|
| g-s-jithesh | C++ estimator (#1), packaging (#5), organizer questions (#6) |
| pavbhaji7 | Reference filter (#2), dropouts + blind prediction (#4) |
| Harsha-1709 | Eval pipeline + validator (#3), deck + demo video (#7) |

Workflow: branch per issue → PR → one teammate reviews → merge. Never leave `main` broken.
