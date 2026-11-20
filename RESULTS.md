# Results log

One row per experiment. ATE = RMSE position error (m) after SE(3) alignment (no scale).
"Stationary" = always output the origin; any estimator must beat it.
Runtime = best of 5 wall-clock runs incl. process start, per 1 s of 200 Hz IMU data (budget 20 ms).
Peak RAM: Windows peak working set (includes shared CRT DLL pages) / private bytes. Budget 8 MB.
Linux `/usr/bin/time -v` numbers to be added via `tools/evaluate.sh` (#3).

## Clean data (no degradation)

| Date | Commit | Change | V1_01 | V1_02 | V1_03 | room1 | room2 | room3 | ms / 1 s data | Peak RAM (WS / private) |
|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-20 | — | Stationary baseline | 1.853 | 1.778 | 1.574 | 1.013 | 1.244 | 1.283 | — | — |
| 2026-11-20 | 2535e91 | Orientation only (Mahony), position fixed | 1.853 | 1.778 | 1.574 | 1.013 | 1.244 | 1.283 | — | — |
| 2026-11-20 | 103aa6a | + naive double integration (no damping) | 25.5 | 40.6 | 88.5 | 1971 | 707 | 803 | — | — |
| 2026-11-20 | 103aa6a | + accel high-pass 1 s, vel tau 1 s, pos leak 3 s, ZUPT | 1.828 | 1.668 | 1.557 | 0.945 | 0.982 | 1.258 | 1.7–2.0 | 4.30 / 0.66 MB |

Mean tilt (roll/pitch) error at 103aa6a: 3.4, 1.4, 1.2, 5.6, 2.3, 4.0 deg.

## Degraded data
_Pending `tools/degrade.py` (#3)._
