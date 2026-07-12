# Lattice Motion Planning — A*, RRT & PRM with Time-Optimal Trajectory Generation

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-rclpy-22314E?logo=ros&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-array%20ops-013243?logo=numpy&logoColor=white)
![SciPy](https://img.shields.io/badge/SciPy-KDTree-8CAAE6?logo=scipy&logoColor=white)
![Matplotlib](https://img.shields.io/badge/Matplotlib-visualization-11557C?logo=plotly&logoColor=white)

A from-scratch motion-planning stack for a differential-drive mobile robot (TurtleBot3 target): a **kinematically-constrained state-lattice planner** solved with **A\***, benchmarked against two sampling-based planners (**RRT**, **PRM**), all feeding into a **time-optimal trapezoidal trajectory generator** that turns a discrete waypoint path into a time-parameterized `(x, y, θ, v, a, ω)` profile.

Originally built as an academic RMPC (motion planning & control) coursework deliverable — cleaned up here as a standalone reference implementation of three classic planning paradigms on the same map.

<p align="center">
  <img src="RMPC_Assignment2_ENTREGA/report_figures/fig7_trajectory_velocity_colored.png" width="70%" alt="Generated trajectory colored by velocity">
</p>

## Table of Contents

- [Overview](#overview)
- [Demo](#demo)
- [Architecture](#architecture)
- [Algorithms](#algorithms)
  - [State-Lattice A*](#1-state-lattice-a)
  - [RRT](#2-rrt-rapidly-exploring-random-tree)
  - [PRM](#3-prm-probabilistic-roadmap)
  - [Planner Comparison](#planner-comparison)
- [Trajectory Generation](#trajectory-generation)
- [Tech Stack](#tech-stack)
- [Getting Started](#getting-started)
- [Configuration Reference](#configuration-reference)
- [Design Notes & Known Limitations](#design-notes--known-limitations)
- [Documentation](#documentation)
- [References](#references)

## Overview

Given a 100×100 occupancy grid seeded with five rectangular obstacles, the same start/goal pair is solved three different ways:

1. A **state-lattice graph** (4-connected headings, straight + quarter-circle arc motion primitives) searched with **A\*** — the only planner that respects heading and guarantees an optimal path.
2. A continuous-space **RRT** that incrementally grows a random tree until it reaches the goal region.
3. A continuous-space **PRM** that builds a reusable roadmap via k-nearest-neighbor connections (`scipy.spatial.KDTree`) and queries it with Dijkstra.

The winning lattice path is then densified and handed to a **trajectory generator** that assigns gear (forward/reverse), cumulative arc-length ("station"), a forward/backward-pass trapezoidal velocity profile bounded by `v_max`/`a_max`, and continuous (unwrapped) heading — producing a fully time-stamped trajectory ready to drive a real differential-drive base.

## Demo

| Obstacle map & goal | Three planners, same problem |
|:---:|:---:|
| ![obstacle map](RMPC_Assignment2_ENTREGA/report_figures/fig1_obstacle_map.png) | ![comparison](RMPC_Assignment2_ENTREGA/report_figures/fig9_comparison.png) |

| Final trajectory on the map | Velocity / acceleration / ω profiles |
|:---:|:---:|
| ![trajectory on map](RMPC_Assignment2_ENTREGA/report_figures/fig6_trajectory_on_map.png) | ![velocity profile](RMPC_Assignment2_ENTREGA/report_figures/fig5_velocity_profile.png) |

## Architecture

```
RMPC_Assignment2_ENTREGA/
├── PathPlanner/
│   ├── main.py                          # Orchestrator: builds map, runs all 3 planners, plots
│   ├── path_planner/
│   │   ├── utils.py                     # Graph structure + ObstaclesGrid collision checker
│   │   ├── lattice_planner.py           # LatticeGraph (motion primitives) + Astar solver
│   │   ├── rrt_planner.py               # RRTPlanner
│   │   └── prm_planner.py               # PRMPlanner (KDTree-backed)
│   └── trajectory_generator/
│       └── traj_generation.py           # Resampling, trapezoidal velocity profile, interpolation
└── report_figures/                      # fig1–fig9, referenced above
```

**Pipeline** (`main.py`):

```
10×10 lattice (400 vertices × 4 headings)
        │  obstacle-aware edge invalidation
        ▼
   A* on lattice ──────► path (row, col, θ)
        │
        ▼
 path_interpolation()   densify straight/arc segments
        │
        ▼
 resample_path()        gears → stations → trapezoidal v(t) → x,y,θ,v,a,ω
        │
        ▼
   matplotlib visualization (+ optional YAML export for a Nav2 launch file)

           (independently, for comparison only)
   RRT.plan()  ──┐
   PRM.plan()  ──┴──► plotted alongside the lattice path
```

## Algorithms

### 1. State-Lattice A*

`path_planner/lattice_planner.py`

- **Vertex** = `(row, col, heading)` with `heading ∈ {0°, 90°, 180°, 270°}` → a 10×10×4 = 400-vertex graph.
- **Motion primitives**: straight edges (cost `1`) connect same-heading neighbors; quarter-circle **arc primitives** (cost `π`) connect adjacent headings, precomputed once as sampled `(x, y)` point sets and reused for every cell via translation.
- **Collision checking**: every edge — straight or arc — is rasterized into grid points and checked against the boolean obstacle map; blocked edges get weight `∞`.
- **Search**: classic A* over the adjacency matrix with a priority queue, `g`-cost accumulation, and parent-pointer path reconstruction.
- **Heuristic**: Euclidean distance on `(row, col)`, ignoring heading.
  - *Admissible* — straight-line distance is always ≤ any real path cost, since every edge costs ≥ 1.
  - *Consistent* — Euclidean distance satisfies the triangle inequality, so A* never needs to reopen a closed node.

### 2. RRT (Rapidly-exploring Random Tree)

`path_planner/rrt_planner.py`

Standard single-query RRT: uniform random sampling over the map, nearest-node lookup by linear scan, `step_size`-bounded steering toward the sample, segment collision-checking by discretized ray marching, and goal-region termination (`max_iter=500`, `step_size=5`).

### 3. PRM (Probabilistic Roadmap)

`path_planner/prm_planner.py`

Two-phase planner: **(a)** sample `num_samples=200` collision-free points and connect each to its `k_neighbors=10` nearest neighbors (`scipy.spatial.KDTree`) if the connecting segment is collision-free, building a reusable roadmap graph; **(b)** query the roadmap with Dijkstra (A* with `h=0`) from start to goal. Because construction and querying are decoupled, the same roadmap can answer multiple start/goal queries without rebuilding.

### Planner Comparison

| Property | A* Lattice | RRT | PRM |
|---|---|---|---|
| Space | Discrete (lattice) | Continuous | Continuous |
| Optimality | Yes (admissible heuristic) | No | No (approximate) |
| Completeness | Complete | Probabilistically complete | Probabilistically complete |
| Multi-query | Yes (same graph) | No (new tree per query) | Yes (roadmap is reusable) |
| Respects heading | Yes (4 headings) | No (x, y only) | No (x, y only) |

Example run (seeded by this repo's obstacle map): A* returns the optimal 9-vertex lattice path; RRT explores a 331-node tree; PRM builds a 202-node / 1,114-edge roadmap. See [Design Notes](#design-notes--known-limitations) for why the RRT/PRM panels in the comparison figure look almost trivial next to the lattice path.

## Trajectory Generation

`trajectory_generator/traj_generation.py`

1. **`path_interpolation`** — densifies the coarse lattice path: 10 samples per straight segment, ~15 per arc segment (using the same precomputed arc primitives as the planner), so the trajectory generator has a smooth, dense polyline to work with.
2. **Stations & gears** — cumulative arc-length (`station[i] = station[i-1] + ‖Δp‖`) plus a forward/reverse gear flag from the dot product between the displacement vector and the heading vector `(-sin θ, cos θ)` (row increases downward, hence the sign flip).
3. **Trapezoidal velocity profile** — for each forward/reverse segment independently: a forward pass computes the max velocity reachable from rest (`v² = v₀² + 2·a_max·Δs`, capped at `v_max`), a backward pass computes the max velocity for decelerating to rest at the segment end, and the profile takes the pointwise minimum of both — the standard trapezoidal (bang-bang acceleration) time-optimal velocity profile.
4. **Uniform-time resampling** — the variable-spaced time profile is resampled onto a fixed `time_step = 0.1 s` grid (at least `min_nfe = 20` points), with `x`, `y`, and unwrapped `θ` linearly interpolated.
5. **Differentiation** — `v`, `ω`, and `a` are recovered by finite-differencing the resampled `x, y, θ` against `dt`.

<p align="center">
  <img src="RMPC_Assignment2_ENTREGA/report_figures/fig8_orientation_profile.png" width="70%" alt="Orientation unwrapped over time">
</p>

## Tech Stack

| Concern | Library |
|---|---|
| Numerics / grid ops | NumPy |
| Nearest-neighbor search (PRM) | SciPy (`scipy.spatial.KDTree`) |
| Visualization | Matplotlib |
| Trajectory export | PyYAML |
| Robot integration target | ROS 2 (`rclpy`, `geometry_msgs`, `visualization_msgs`), TurtleBot3 `navigation2` stack |

## Getting Started

### Prerequisites

```bash
pip install numpy scipy matplotlib pyyaml
```

`main.py` also imports `rclpy`, `visualization_msgs.msg.Marker`, and `geometry_msgs.msg.Point` — these come from a sourced **ROS 2** installation. Note they're currently unused in `main.py`'s body (no node is spun up); they're leftover scaffolding for wiring this planner into a live ROS 2 node. If you just want to run the planning/visualization demo without a ROS 2 environment, comment out those three import lines.

### Run the demo

```bash
cd RMPC_Assignment2_ENTREGA/PathPlanner
python3 main.py
```

This builds the map, runs all three planners, generates the trajectory, and opens a Matplotlib window with the obstacle map, path, and velocity curve. Run from inside `PathPlanner/` (or add it to `PYTHONPATH`) — `main.py` imports `path_planner.*` and `trajectory_generator.*` as top-level packages.

### Exporting a trajectory for Nav2

`path_planner/utils.py::write_result_to_yaml` writes the resampled trajectory to `install/turtlebot3_navigation2/share/turtlebot3_navigation2/launch/<filename>.yaml`, relative to the package location — call it from `main.py` (or your own script) after `resample_path()` if you're feeding this into a TurtleBot3 `navigation2` launch pipeline.

## Configuration Reference

**Map / lattice** (`main.py`)

| Parameter | Value | Meaning |
|---|---|---|
| `n_rows`, `n_cols` | 10, 10 | Lattice grid dimensions |
| `lattice_cell_size` | 10 | World units per lattice cell |
| `s_3d` / `g_3d` | `(1, 8, 90)` / `(8, 2, 270)` | Start / goal as `(row, col, heading°)` |

**RRT** (`RRTPlanner.__init__`)

| Parameter | Default | Meaning |
|---|---|---|
| `max_iter` | 500 | Max sampling iterations |
| `step_size` | 5 | Steering distance & goal tolerance |

**PRM** (`PRMPlanner.__init__`)

| Parameter | Default | Meaning |
|---|---|---|
| `num_samples` | 200 | Random free-space samples |
| `k_neighbors` | 10 | Neighbors considered per node |
| `step_size` | 5 | Collision-check ray-march resolution |

**Trajectory generator** (`TrajGenerator.__init__`)

| Parameter | Default | Meaning |
|---|---|---|
| `time_step` | 0.1 s | Output sample period |
| `max_acceleration` | 1 m/s² | Trapezoidal profile accel bound |
| `max_velocity` | 1 m/s | Trapezoidal profile velocity cap |
| `omega_max` | 2.8 rad/s | Declared angular-velocity bound (see note below) |
| `min_nfe` | 20 | Minimum output trajectory points |

## Design Notes & Known Limitations

Found while reading the implementation closely enough to write this README — worth knowing before extending the code:

- **RRT/PRM solve an easier problem than A\*.** `main.py` passes the *same* `(1, 8)` / `(8, 2)` tuples to both the lattice planner (where they're grid-cell indices, later scaled by `lattice_cell_size=10`) and to `RRTPlanner`/`PRMPlanner` (where they're used directly as raw `(x, y)` coordinates in the full 100×100 map). The practical effect: A* solves the intended long diagonal traverse through all five obstacles, while RRT and PRM solve a trivial ~10-unit hop near the top-left corner. That's why their panels in `fig9_comparison.png` show a big exploration tree/roadmap but a barely-visible straight path — to compare all three planners on equal footing, scale the RRT/PRM start/goal by `lattice_cell_size` first.
- **`omega_max` is declared but never enforced.** `TrajGenerator.omega_max = 2.8 rad/s` is set in `__init__` but `resample_path()` never clamps against it — `ω` is purely the finite-difference of interpolated heading. `fig5_velocity_profile.png` shows angular-velocity spikes above 6 rad/s at the heading-transition instants (the arc waypoints around t≈5s, 17s, and 39s), more than double the declared bound.
- **`Astar.get_neighbor` is O(V) per expansion** (`graph_vert_list.index(u)` + a full row scan of the adjacency matrix), fine at 400 vertices, would need an adjacency-list rewrite to scale.
- The Spanish/English viva-prep guides (`STUDY_GUIDE_VIVA.md`, `GUIA_ESTUDIO_VIVA.md`, `DEFENSE_GUIDE.md`, `VIVA_QA.md`) go considerably deeper into the math (admissibility/consistency proofs, why `-sin(θ)` for the row heading component, etc.) than this README — check those for the full derivations.

## Documentation

- [`STUDY_GUIDE_VIVA.md`](STUDY_GUIDE_VIVA.md) / [`.pdf`](STUDY_GUIDE_VIVA.pdf) — full line-by-line walkthrough with likely viva questions and answers (English).
- [`GUIA_ESTUDIO_VIVA.md`](GUIA_ESTUDIO_VIVA.md) / [`.pdf`](GUIA_ESTUDIO_VIVA.pdf) — same, in Spanish.
- [`DEFENSE_GUIDE.md`](DEFENSE_GUIDE.md) — condensed defense/oral-exam prep notes.
- [`VIVA_QA.md`](VIVA_QA.md) / [`.pdf`](VIVA_QA.pdf) — Q&A format.

## References

- Kavraki, L. E., Švestka, P., Latombe, J.-C., & Overmars, M. H. (1996). *Probabilistic roadmaps for path planning in high-dimensional configuration spaces*. IEEE Transactions on Robotics and Automation, 12(4), 566–580.
- LaValle, S. M. (1998). *Rapidly-exploring random trees: A new tool for path planning*. Technical Report TR 98-11, Iowa State University.
- Pivtoraiko, M., & Kelly, A. (2008). *Differentially constrained motion replanning using state lattices with graduated fidelity*. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A formal basis for the heuristic determination of minimum cost paths*. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107. (A*)
