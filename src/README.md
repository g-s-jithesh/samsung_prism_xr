# Path Finder — build & run

IMU-only 6-DoF trajectory estimator. Single C++17 file, no dependencies beyond the C/C++ standard library.

## Build
```bash
g++ -O2 -std=c++17 -static -o pathfinder src/main.cpp
```
Or with Docker (from the repo root):
```bash
docker build -t pathfinder .
```

## Run
```bash
./pathfinder <imu_csv> <out_txt>
# Docker:
docker run --rm -v "$PWD/data:/data" pathfinder /data/imu0/data.csv /data/estimated_trajectory.txt
```
- Input: EuRoC `imu0/data.csv` — `timestamp_ns,wx,wy,wz,ax,ay,az` (rad/s, m/s²).
- Output: TUM format `timestamp_s tx ty tz qx qy qz qw`, one pose per input row.
- Rows with blank/NaN values still get a pose (propagated from the last valid state).

## Measure
```bash
/usr/bin/time -v ./pathfinder <imu_csv> <out_txt>   # "Maximum resident set size" = peak RAM
```

## Method
1. **Init (first 1 s):** gyro bias = mean rate; roll/pitch from mean accel (gravity); yaw = 0.
2. **Orientation:** RK2 quaternion integration of bias-corrected gyro, Mahony correction of
   roll/pitch (and slow gyro-bias tracking) when |a| ≈ g.
3. **Position:** rotate accel to world, subtract g, high-pass (1 s running mean) to remove
   tilt/bias leak, integrate with velocity damping (τ = 1 s; ZUPT when stationary) and a
   position leak toward the origin (τ = 3 s).
4. **Dropouts / invalid rows:** hold last angular rate, constant-velocity propagation.

Memory is O(1): rows are streamed with `fgets`/`strtod`; only a fixed 1024-sample init buffer is kept.
