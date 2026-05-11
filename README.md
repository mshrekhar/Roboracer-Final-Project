[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/JarnGLDa)
# ESE6150 Final Project 
# End-to-End Autonomous Racing Pipeline for F1TENTH

A fully autonomous racing pipeline for 1/10-scale F1TENTH vehicles that requires no prior map or human-assisted initialization. The system performs a single exploratory lap to build a map, automatically extracts a racing centerline, and transitions to high-speed Pure Pursuit control, all triggered from a single script.

---

## Table of Contents

- [Team](#team)
- [Demo](#demo)
- [Overview](#overview)
- [Technical Approach](#technical-approach)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Results](#results)
- [Challenges](#challenges)
- [Future Work](#future-work)

---

## Team

| Name | GitHub |
|------|--------|
| Milan Manoj | [@milanmnj](https://github.com/milanmnj) |
| Eric Ouyang | [@eouyang](https://github.com/eouyang) |
| Manasi Shrekhar | [@mshrek](https://github.com/mshrek) |
| Jackson Wang | [@yxjacksn](https://github.com/yxjacksn) |

---

## Demo

| Track | Video | Foxglove |
|-------|-------|----------|
| Track 1 | [Video](https://drive.google.com/file/d/1HYN3x-Qh0wS01kLwBNuDhgdLquM_PSRi/view?usp=sharing) | [Foxglove](https://www.youtubeeducation.com/watch?v=6sNKPYoXPBI) |
| Track 2 | [Video](https://drive.google.com/file/d/1Dkfw_AU1Y1vDPZ08qFdchJVDIyK-ZZMW/view?usp=sharing) | [Foxglove](https://drive.google.com/file/d/1A2atkTd6qpkxnQ-qBKz6E_uek8P20I6E/view?usp=sharing) |
| Track 3 | [Video](https://drive.google.com/file/d/12nTyHf5h7Cq68dngfshzSM0Sx3hR79Vg/view?usp=sharing) | [Foxglove](https://drive.google.com/file/d/1ygJcwItUR9RrkG2Ec6uon8oMbUUrycoB/view?usp=sharing) |

---

## Overview

### Problem

Autonomous racing on an unknown track is hard. Most F1TENTH implementations sidestep the hardest part: they either use a pre-built map or require a human to manually drive the car first to collect one. This means the system cannot operate without prior human effort and will fail entirely on any track it has not seen before.

### Why It Matters

Removing the dependency on prior maps is important well beyond racing. Any robot deployed in an unstructured or previously unseen environment — a collapsed building, a new warehouse floor, an unmapped road — faces the same problem. A system that can map, plan, and act entirely from scratch is fundamentally more general and more robust.

### Our Contribution

We built a complete end-to-end pipeline that solves all three phases of the problem in sequence with no human input:

1. **Mapping** — the car drives itself around the track once using a reactive wall-following controller while building a SLAM map in real time.
2. **Planning** — the finished map is automatically processed to extract a smooth centerline and generate optimized waypoints, while the particle filter initializes for localization.
3. **Racing** — the car switches to Pure Pursuit control and completes subsequent laps at competitive speed.

The entire pipeline is launched with a single shell script and was validated on three different tracks without any track-specific tuning.

---

## Technical Approach

The pipeline is built around a clean separation of concerns: map first, plan once, then race. This avoids the problem of needing a map to plan and needing to drive to build a map, by using a reactive controller (wall following) that requires no map at all for the first lap.

The five main components are:

- **Wall Following** — a right-wall PID controller that safely drives the car around any closed track using only LiDAR, with no prior knowledge of the layout. Used exclusively during lap 1 to collect sensor data for SLAM.
- **SLAM** — CHANGE
- **Centerline Extraction** — a geometry pipeline that processes the occupancy grid using a Euclidean distance transform and skeletonization to find the track centerline, prunes noise, fits a periodic B-spline, and outputs evenly spaced waypoints with headings.
- **Particle Filter** — CHANGE
- **Pure Pursuit** — a geometric controller that tracks the extracted waypoints by computing the steering angle to a lookahead point on the path, with speed scheduled by curvature.

These components are chained together automatically by `final.sh`, with a ~20 s downtime between the mapping and racing phases for planning and localization to initialize.

---

## System Architecture

```
Phase 1 — Exploration
  Wall-following (PID) + SLAM → occupancy grid map

Phase 2 — Planning (~20 s downtime)
  Centerline extraction → B-spline waypoints
  Particle filter initialization

Phase 3 — Racing
  Pure Pursuit → high-speed waypoint tracking
```

### Wall Following

A right-wall PID controller is used during the exploratory lap. Two LiDAR beams — one pointing directly right and one angled 50° forward — estimate the wall angle and project the car's distance to the wall ahead by a lookahead distance. A PID controller drives this projected distance to a setpoint, with EMA smoothing on the steering output and speed scheduling based on steering magnitude. We initially tried gap following but it required per-track tuning and produced too much oscillation for reliable SLAM; right-wall PID generalized immediately.

### SLAM

CHANGE

### Centerline Extraction

The saved occupancy grid is processed to extract a smooth closed-loop racing trajectory:
1. Threshold free space from the occupancy grid
2. Compute the Euclidean distance transform
3. Threshold a skeleton from ridge points near the corridor center
4. Build a graph, prune dead-end branches, detect the closed loop
6. Convert from pixel to world coordinates using map yaml

### Particle Filter

Localizes the car against the saved map for all laps after the first. Initialized during the downtime window using the SLAM pose at loop closure as a prior.

### Pure Pursuit

A geometric path-tracking controller that selects a lookahead waypoint and computes the required steering angle from the pure pursuit curvature equation. Speed is scheduled based on path curvature.

---

## Installation

### Setup

Clone the repository into the `src` folder of your ROS 2 workspace.
Then build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start

Run the full pipeline with the provided shell script:

```
./final.sh
```

The car will:
1. Wall-follow for one lap while building a SLAM map
2. Pause ~20 s for centerline extraction and particle filter initialization
3. Switch to Pure Pursuit and begin racing

---

## Results

The pipeline was validated on three tracks without any track-specific tuning.

| Track | Wall-Follow Lap Time | Pure Pursuit Lap Time |
|-------|---------------------|-----------------------|
| Track 1 | 43 s | 13 s |
| Track 2 | 47 s | 13 s |
| Track 3 | 40 s | 15 s |

Pure Pursuit was able to significantly reduce lap times compared to the initial SLAM exploration lap, bringing performance much closer to competitive racing times on the Levine track. Across all three tracks, the full pipeline successfully completed autonomous mapping, centerline extraction, localization, and high-speed racing without any track-specific tuning or manual initialization. The wall-following controller produced stable trajectories that enabled reliable SLAM maps, while the spline-based centerline extraction generated smooth waypoint paths for racing. Curvature-based speed scheduling improved stability in tighter turns and reduced overshoot at higher speeds. Visualizations of the occupancy grids, extracted centerlines, and Foxglove trajectory tracking showed consistent localization and smooth Pure Pursuit behavior across different track layouts, demonstrating that the pipeline generalizes well to unseen closed-loop environments.

---

## Challenges

**Sharp 90-degree corners** — The wall-following controller can clip the inside wall on tight turns, occasionally introducing drift in the SLAM map. Speed was reduced during mapping as a mitigation; a geometry-aware turn strategy is a planned improvement.

**Particle filter initialization** — The filter occasionally fails to converge within the 20 s window when the start pose estimate is poor. Using the SLAM loop-closure pose as a strong prior improved reliability.

**Skeleton noise** — The distance-transform skeleton contained spurious branches from map artifacts. Iterative dead-end pruning (removing degree-1 nodes until none remain) resolved this cleanly.

**Node management** — Integrating all the nodes proved to be difficult. Killing nodes cleanly and double checking before starting new ones that might conflict resolved this.

---

## Future Work

- Add curvature-aware speed reduction during wall following to handle 90-degree corners more reliably
- Replace Pure Pursuit with an MPPI controller for tighter, dynamics-aware tracking at higher speeds
- Reduce or eliminate the fixed 20 s downtime from mostly particle filter
- Extend the pipeline to head-to-head racing with car detection and overtaking
