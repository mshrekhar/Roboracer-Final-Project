# ESE6150 Final Project
# End-to-End Autonomous Racing Pipeline for F1TENTH

A fully autonomous racing pipeline for 1/10-scale F1TENTH vehicles that requires no prior map or human-assisted initialization. The system performs a single exploratory lap to build a map, automatically extracts a racing centerline, and transitions to high-speed Pure Pursuit control, all triggered from a single script.

---

## Team

| Name | GitHub |
|------|--------|
| Milan Manoj | [@milanmnj](https://github.com/milanmnj) |
| Manasi Shrekhar | [@mshrek](https://github.com/mshrek) |
| Eric Ouyang | [@eouyang](https://github.com/eouyang) |

---

## Demo

| Track | Video | Foxglove |
|-------|-------|----------|
| Levine | [Video](https://youtu.be/39KLCvDCAwY) | [Foxglove](https://youtu.be/JQoCNW_JeGI) |
| Towne | [Video & Foxglove](https://youtu.be/P-oC10r4B0A) | |
| Skirkanich | [Video & Foxglove](https://youtu.be/1X9nICsn6gU) | |

---

## Overview

Most F1TENTH implementations require a pre-built map or a manual initialization lap — meaning they fail entirely on any track they haven't seen before. We built a complete end-to-end pipeline that solves all three phases from scratch with no human input:

1. **Mapping** — the car drives itself around the track once using a reactive wall-following controller while building a SLAM map in real time.
2. **Planning** — the finished map is automatically processed to extract a smooth centerline and generate optimized waypoints while the particle filter initializes.
3. **Racing** — the car switches to Pure Pursuit and completes subsequent laps at competitive speed, with an adaptive speed controller that automatically finds the maximum sustainable speed.

Validated on three different tracks without any track-specific tuning.

---

## System Architecture

```
Phase 1 — Exploration
  Wall-following (PID) + RTAB-Map SLAM → occupancy grid map

Phase 2 — Transition (~30 s)
  Centerline extraction + raceline optimization → waypoints
  Particle filter initialization

Phase 3 — Racing
  Pure Pursuit + adaptive speed control → high-speed lap
```

---

## Technical Approach

### Wall Following
A right-wall PID controller drives the exploratory lap. Two LiDAR beams estimate the wall angle and project the car's lateral distance ahead by a 3 m lookahead. A PID controller drives this to a 0.8 m setpoint, with EMA smoothing on the steering output and speed scheduling based on steering magnitude (capped at 1.0 m/s during mapping). Gap following was tried first but required per-track tuning and produced too much oscillation for reliable SLAM.

### SLAM
RTAB-Map fuses the SICK 2D LiDAR, Intel RealSense D435i stereo/RGB (for visual loop closure), and D435i IMU (fused with wheel odometry via EKF). The visual bag-of-words loop closure corrects drift at the end of the exploratory lap, producing a metrically consistent occupancy grid at 0.05 m/cell resolution. The map is saved automatically as a `.pgm`/`.yaml` pair at lap completion — no human trigger required.

### Centerline Extraction & Raceline Optimization
1. Threshold free space from the occupancy grid
2. Compute the Euclidean distance transform
3. Extract skeleton from ridge points near the corridor center
4. Build a graph, prune dead-end branches, detect the closed loop
5. Fit a periodic cubic B-spline (smoothing factor 2000), sample uniform waypoints
6. Run a shortest-path QP (OSQP via cvxpy) to laterally shift waypoints within the drivable corridor, with a 0.5 m wall margin
7. Convert to world coordinates using map metadata

### Particle Filter
Localizes the car against the saved static map for all laps after the first. Initialized programmatically at the map origin (0, 0, 0) — the same location where the exploratory lap started — by publishing `/initialpose` 30 times over 3 s. Uses LiDAR beam model for likelihood weighting and EKF odometry for particle propagation.

### Pure Pursuit + Adaptive Speed
Geometric path-tracking controller that steers toward a lookahead point on the waypoint set. Speed starts at 2.0 m/s and increments by 0.4 m/s each lap if average cross-track error stays below 0.21 m. When CTE exceeds the threshold, the system locks the last good speed and runs a confirmation lap. No manual speed tuning required.

---

## Installation

```bash
cd ~/f1tenth_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Update the file paths in `final.sh` and the Pure Pursuit node to match your workspace.

**Dependencies:** `rtabmap_ros`, `particle_filter`, `pure_pursuit_reactive`, `sensors_bringup`, `f1tenth_stack`, `cvxpy` (with OSQP), `numpy`, `scipy`, `Pillow`

---

## Quick Start

```bash
bash final.sh
```

The car will wall-follow for one lap while building a SLAM map, pause ~30 s for centerline extraction and particle filter initialization, then switch to Pure Pursuit racing.

To stop everything cleanly:
```bash
bash final.sh kill
```

---

## Results

| Track | SLAM Lap (s) | Transition (s) | Locked Speed (m/s) |
|-------|-------------|----------------|-------------------|
| Levine | ~40 | ~30 | 4.0 |
| Towne | ~40 | ~30 | 3.0 |
| Skirkanich | ~40 | ~30 | 3.2 |

All runs are uncut — videos show the full pipeline from wall-following through SLAM transition to Pure Pursuit racing at locked speed.

---

## Challenges

**Sharp corners** — Wall-following clips inside walls on tight 90° turns, occasionally introducing SLAM drift. Speed was reduced during mapping as mitigation.

**Particle filter init** — Occasionally fails to converge if the car drifts from its start position at the end of the exploratory lap. Solved by publishing the initial pose repeatedly and ensuring the lap gate brings the car back close to origin.

**Skeleton noise** — Distance-transform skeleton contained spurious branches from map artifacts. Fixed by iterative dead-end pruning (removing degree-1 nodes until none remain).

**Node management** — Cleanly killing all nodes between phases and verifying teardown before starting new ones was critical to avoid topic conflicts.

---

## Future Work

- Curvature-aware speed reduction during wall following for tighter corners
- Replace Pure Pursuit with MPPI for dynamics-aware tracking at higher speeds
- Eliminate redundant hardware restarts to reduce transition time below 15 s
- Extend to head-to-head racing with opponent detection and overtaking
