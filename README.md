# LiDAR Rover Simulation with Autonomous Navigation and Frontier Exploration

A real-time 3D LiDAR rover simulation written in C with OpenGL/GLUT rendering. The rover autonomously explores the environment using frontier-based waypoint generation on a projected 2D occupancy map, while building a live occupancy grid from raycasted LiDAR scans.

<img width="753" height="757" alt="Screenshot 2026-04-03 at 6 06 45 PM" src="https://github.com/user-attachments/assets/04cbdf64-4cd9-49a4-b01b-2dbf83e0e28a" />
<img width="735" height="662" alt="Screenshot 2026-04-03 at 6 11 26 PM" src="https://github.com/user-attachments/assets/2dd5b375-8b8f-4755-92b3-4b3437a9fef2" />
<img width="554" height="539" alt="Screenshot 2026-04-03 at 6 12 05 PM" src="https://github.com/user-attachments/assets/bbc0a642-fe72-4b86-a2e7-d5f590da5e96" />

---
 
## Features

- LiDAR raycasting — rotating sensor sweeps the scene using Möller–Trumbore ray-triangle intersection, producing a live point cloud with height-based colour gradient
- 3D occupancy mapping — log-odds voxel grid updated via 3D DDA traversal; tracks free, occupied, and unknown cells
- 2D projected map — column-summed occupancy map at rover height for navigation and frontier visualisation
- Autonomous waypoint generation — frontier planner selects reachable frontier targets and generates waypoint paths automatically
- Clearance-aware path planning — projected pathing avoids cells that are too close to obstacles for the rover to fit through
- Waypoint skipping — rover can advance past already-reached intermediate waypoints, including the final waypoint
- Frontier detection — identifies boundaries between free and unknown space for exploration targeting
- MPPI controller — Model Predictive Path Integral controller with parallel rollout evaluation for autonomous navigation
- Rover physics — differential drive model with acceleration, friction, angular dynamics, and triangle mesh collision
- Multi-process pipeline — ray casting, occupancy updates, frontier analysis, and MPPI rollouts each run in separate forked processes communicating via Unix pipes
- Manual and autonomous modes — switch between WASD manual control and autonomous waypoint following at runtime
- Point cloud toggle — runtime keyboard toggle for showing or hiding the live LiDAR point cloud
- EKF sensor fusion — in progress; predict step implemented, correct step pending scan matching
 
---
 
## Controls
 
| Key | Action |
|-----|--------|
| `W/S` | Throttle forward / backward |
| `A/D` | Steer left / right |
| `C` | Toggle manual / auto mode |
| `P` | Pause / unpause |
| `F` | Toggle 2D frontier visualisation |
| `G` | Toggle 3D occupancy map (pauses sim) |
| `T` | Toggle scene wireframe |
| `V` | Toggle point cloud visibility |
| `+/-` | Zoom in / out |
| Right drag | Orbit camera |
| Middle drag | Pan camera |
| `ESC` | Quit |
 
---
 
## Architecture
 
```
main.c
  │
  ├── rendering/
  │   ├── scene.c            Triangle mesh builder (quads, boxes, tessellated walls)
  │   ├── renderer.c         OpenGL draw calls: wireframe, point cloud, occupancy voxels, sensor arrow
  │   └── camera.c           Orbit/pan camera, keyboard and mouse callbacks
  │
  ├── rover/
  │   ├── rover_physics.c    Differential drive: acceleration, friction, collision response
  │   ├── rover_controller.c MPPI controller, path following, waypoint management, odometry
  │   └── ekf_fusion.c       Extended Kalman Filter — predict step done, correct step in progress
  |   ├── frontier_exploration/
  │   ├── frontier_planner.c   2D frontier selection, reachability scoring, waypoint generation
  │   └── frontier_planner.h   Planner entry point used by frontier_analyzer
  │
  ├── lidar/
  │   ├── lidar_sensor.c     Sensor state, elevation ring initialisation
  │   ├── sensor_control.c   Scan scheduling, throttle/steer inputs, rover_control tick
  │   └── raycaster.c        Möller–Trumbore triangle intersection, closest-hit ray cast
  │
  ├── scene/
  │   ├── point_cloud.c      Dynamic array of hit points with age, distance, intensity
  │   ├── occupancy_map.c    Log-odds voxel grid, 3D DDA ray cast, frontier detection
  │   ├── frontier_projection.c  Column summaries and 2D projected map updates
  │   └── scene_collision.c  Point-to-triangle distance, stepped collision sweep
  │
  ├── piping/
  │   ├── coordinator.c      Scan coordinator: shards ray work across workers, gathers results
  │   ├── worker.c           Ray worker loop + MPPI rollout worker loop
  │   ├── occupancy_updater.c  Applies point batches to occupancy map in a dedicated process
  │   ├── frontier_analyzer.c  Detects frontier voxels and emits waypoints
  │   └── method_dispatcher.h  Entry points for all forked process loops
  │
  └── core/
      ├── math_utils.c       wrap_angle, matrix helpers
      ├── vec3.h             Vector3 operations
      ├── noise.c            Gaussian noise (Box-Muller)
      └── physics_constants.h  MAX_SPEED, ACCELERATION, DEG_TO_RAD, etc.
```
 
---
 
## Multi-Process Pipeline
 
The simulation forks four long-running child processes at startup and communicates with them via Unix pipes:
 
```
main process
  │
  ├─► scan coordinator       receives ScanRequest, shards rays across NUM_WORKERS workers
  │       └─► ray workers    each cast a subset of elevation rings, return RayResultBatch
  │
  ├─► occupancy updater      receives point batches, runs DDA ray cast, updates voxel grid (shared mmap)
  │
  ├─► frontier analyzer      receives voxel updates, identifies frontier cells, emits waypoints
  │
  └─► rollout coordinator    receives RolloutRequest (MPPI), shards samples across NUM_WORKERS workers
          └─► rollout workers each simulate a subset of trajectories, return costs
```
 
The occupancy grid is allocated with `mmap(MAP_SHARED | MAP_ANONYMOUS)` so the main process can read it for rendering without copying.
 
---
 
## MPPI Controller
 
The rover uses a Model Predictive Path Integral controller for autonomous navigation.
 
**References:**
- [MPPI — Williams et al.](https://acdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdeltacdelta.github.io/mppi-generic-website/docs/mppi.html)
- [MPPI explained — dilithjay.com](https://dilithjay.com/blog/mppi)
 
At each frame, `MPPI_SAMPLES` randomised control sequences are rolled out over `MPPI_HORIZON` steps. Each rollout is scored using a cost function that penalises:
 
| Term | Weight |
|------|--------|
| Cross-track error from path | 3.5 |
| Heading error vs path tangent | 8.0 |
| Steering rate (jerk) | 2.0 |
| Speed deviation from reference | 6.0 |
| Throttle effort | 0.15 |
| Terminal cross-track error | 6.0 |
| Terminal heading error | 2.0 |
 
Weights are then computed as `exp(-(cost - min_cost) / λ)` and the optimal control sequence is updated as a weighted sum of all noise perturbations. The nominal sequence is warm-started from the previous frame.
 
Rollout evaluation is offloaded to the rollout coordinator process for parallelism, with a synchronous fallback in the main process if the pipe is unavailable.



## MPPI Parameter Tuning
 
A grid search was conducted over MPPI planner parameters to identify the configuration that minimises completion time while maintaining reliability (zero DNFs). Due to the large search space, sweeps were conducted sequentially. The exact numbers from some earlier runs were lost; the most recent weight tuning results are shown below.
 

<img width="651" height="410" alt="Screenshot 2026-06-24 at 3 33 17 AM" src="https://github.com/user-attachments/assets/0429713c-ae58-4a69-bdad-9fc98bf54e06" />

### Search Space
 
| Group | Parameter | Values Tested |
|---|---|---|
| **Weights** | `w_heading` | 4.0, 8.0, 12.0 |
| **Weights** | `w_speed` | 2.0, 6.0, 10.0 |
| **Architecture** | `mppi_samples` | 128, 256 |
| **Architecture** | `mppi_horizon` | 60, 80 |
| **Exploration** | `sigma_steer` | 0.8, 1.2, 1.5 |
| **Exploration** | `mppi_lambda` | 1.0, 2.5, 4.0, 7.0, 10.0 |
 
### Key Findings
 
- `w_heading=8.0` was the most reliable row in the weight sweep. Lower values (`4.0`) were sluggish; higher values (`12.0`) over-constrained the planner, causing up to 67% DNF rates.
- `w_speed=6.0` balanced aggressiveness and stability. `w_speed=10.0` was fast in successful runs but unreliable.
- Larger sample counts (`256`) and longer horizons (`80`) consistently outperformed their smaller counterparts, at the cost of compute.
- Low `mppi_lambda` (`1.0`) created less smoothing over the sample distribution and performed best, suggesting the planner benefits from sharper cost discrimination.
- `sigma_steer=1.2` struck the best balance between exploration and control smoothness.

> **Note:** These parameters were tuned to optimise tracking performance
> in simulation and do not account for real-time computational constraints. In particular,
> `mppi_samples=256` and `mppi_horizon=80` were selected as the practical upper limit for
> a consumer GPU without causing simulation frame rate degradation. On embedded or
> CPU-only hardware, lower values may be necessary.

### Optimal MPPI Configuration
 
| Parameter | Value |
|---|---|
| `mppi_lambda` | 1.0 |
| `mppi_samples` | 256 |
| `mppi_horizon` | 80 |
| `w_heading` | 8.0 |
| `w_speed` | 6.0 |
| `sigma_steer` | 1.2 |

---
 
## Occupancy Mapping
 
The 3D occupancy map uses a log-odds representation over a voxel grid:
 
- **Hit** — `+6.0` log-odds at the ray terminal voxel
- **Miss** — `-0.15` log-odds (scaled by distance²) along the ray path
- **Thresholds** — cells below `FREE_THRESHOLD` are free, above `OCCUPIED_THRESHOLD` are occupied
 
Ray traversal uses the **3D Digital Differential Analyzer (DDA)** algorithm:
- [Amanatides & Woo, 1987](http://www.cse.yorku.ca/~amana/research/grid.pdf)
 
Floor hits are ignored below a configurable cell threshold to avoid the ground plane being marked occupied.
 
The occupancy grid is shared memory (`mmap`) between the main process and worker processes, allowing the renderer to visualise it live without copying.
 
---
 
## LiDAR Sensor
 
The simulated sensor rotates continuously in azimuth (`theta += 0.02 rad/frame`) and fires `NUM_RINGS` rays spread across a `[-30°, +89°]` elevation range each step.
 
Ray-triangle intersection uses the **Möller–Trumbore algorithm** for each triangle in the scene, finding the closest hit. Gaussian noise is added to hit distances to simulate real sensor noise.
 
In the current implementation, ray casting is distributed across worker processes.
 
---
 
## EKF Sensor Fusion
 
`rover/ekf_fusion` implements an Extended Kalman Filter over the state `[x, z, θ]`.
ICP-based correction is fully integrated with the EKF predict step. The EKF consistently outperforms dead-reckoning odometry; ICP scan matching noise is the primary remaining limitation, documented in the limitations section.
 
The red line rendered between the true sensor position (green) and the odometry estimate (blue) visualises the growing pose error that EKF correction would reduce.
 
**Reference:** [Probabilistic Robotics — Thrun, Burgard, Fox](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)

# EKF & MPPI Parameter Tuning — Findings

Two parameter sweeps were conducted to tune the Extended Kalman Filter (EKF) noise covariance matrices used for localisation. The optimal MPPI planner parameters (`lambda=1.0`, `samples=256`, `horizon=80`, `w_heading=8.0`, `w_speed=6.0`, `sigma_steer=1.2`) were held constant throughout. Each configuration was repeated 3 times.

---

## Sweep 1 — Process Noise (R matrix): `speed_noise` vs `angular_noise_deg`

16 configurations were evaluated across 4 speed noise levels (`0.05, 0.10, 0.20, 0.40`) and 4 angular noise levels (`2°, 4°, 8°, 12°`), with lidar and angle noise held fixed at `0.05` and `1.5°` respectively.


<img width="821" height="532" alt="Screenshot 2026-06-24 at 3 25 45 AM" src="https://github.com/user-attachments/assets/289f7d5b-d0cf-40e8-add5-4ff46fdc5588" />
<img width="822" height="539" alt="Screenshot 2026-06-24 at 3 26 16 AM" src="https://github.com/user-attachments/assets/0ae5b788-695d-41be-bbb1-32dfe34befbb" />



**Key findings:**
- `speed_noise=0.10` with `angular_noise=8°` produced the best overall result: **RMS position error 0.183 m**, **RMS heading error 1.591°**, with near-zero run-to-run variance.
- Very low speed noise (`0.05`) consistently underperformed, likely over-trusting the odometry model.
- High speed noise (`0.40`) introduced significant instability, particularly at low angular noise
- The trend suggests the EKF benefits from moderate speed uncertainty to allow LiDAR corrections to take effect

**Best config:** `speed_noise=0.10`, `angular_noise=8°` with RMS pos err **0.183 m**, RMS heading err **1.591°**

---

## Sweep 2 — Measurement Noise (Q matrix): `lidar_noise` vs `angle_noise_deg`

9 configurations were evaluated with `speed_noise` and `angular_noise` fixed at their optimal values from Sweep 1 (`0.10` and `8°`), sweeping lidar noise (`0.05, 0.10, 0.20`) and angle noise (`1.5°, 3.0°, 6.0°`).


<img width="1865" height="1328" alt="heatmap_means" src="https://github.com/user-attachments/assets/1e6a6fca-d282-4d2b-8089-76e2b544d4d8" />
<img width="1875" height="1328" alt="heatmap_stds" src="https://github.com/user-attachments/assets/394e67eb-64e3-4460-b511-52c6dff4f25c" />



**Key findings:**
- The original baseline (`lidar_noise=0.05`, `angle_noise=1.5°`) remained the best for position accuracy: **RMS position error 0.213 m**.
- Increasing lidar noise or angle noise generally degraded performance, suggesting the sensor model was already well-calibrated.
- The notable exception: `lidar_noise=0.20`, `angle_noise=1.5°` achieved the **lowest RMS heading error (1.762°)** at the cost of slightly higher position error (0.256 m) — a viable trade-off if heading accuracy is prioritised.
- Higher angle noise (`6°`) produced the most inconsistent results, with heading error standard deviations up to **5.05°** (config 6).

Ultimately, the baseline Q config (lidar=0.05, angle=1.5°) remained optimal across both sweeps. The difference between the 0.183m figure from Sweep 1 and 0.213m here reflects run-to-run variance from the unseeded RNG rather than a change in configuration.



**Best config:** `lidar_noise=0.05`, `angle_noise=1.5°` with RMS pos err **0.213 m**, RMS heading err **4.243°**

---

## EKF vs Odometry Comparison

A trajectory-level comparison was run using the optimal EKF configuration against raw odometry (dead reckoning).

<img width="1764" height="1397" alt="ekf_vs_odom_components" src="https://github.com/user-attachments/assets/1de89857-c375-4028-8bea-de93c210041a" />
<img width="1934" height="732" alt="ekf_vs_odom_magnitude" src="https://github.com/user-attachments/assets/d133de71-1f41-4c38-9cec-1db39e5503c7" />


The EKF consistently outperforms odometry:

- **X-axis:** Odometry drifts further from ground truth in the second half of the run; the EKF correction keeps error bounded.
- **Z-axis:** Both methods accumulate error mid-run (~15–20 s), corresponding to a curved section of the track. The EKF recovers faster.
- **Heading:** Errors are similar early on but diverge after ~10 s; the EKF maintains tighter heading estimates through turns.
- The 2D position error magnitude confirms the EKF rolling RMS stays **below odometry for the majority of the run**, with the gap widening after the 15 s mark.

---

## Optimal Configuration Summary

| Parameter | Value |
|---|---|
| `speed_noise` | 0.10 |
| `angular_noise_deg` | 8.0° |
| `lidar_noise` | 0.05 |
| `angle_noise_deg` | 1.5° |
| `mppi_lambda` | 1.0 |
| `mppi_samples` | 256 |
| `mppi_horizon` | 80 |
| `w_heading` | 8.0 |
| `w_speed` | 6.0 |
| `sigma_steer` | 1.2 |

| Metric | Value |
|---|---|
| Mean RMS position error | 0.213 m |
| Mean peak position error | 0.435 m |
| Mean RMS heading error | 1.591° |
| Mean peak heading error | 4.037° |
---

## Limitations

### Reproducibility
Gaussian noise is drawn from an unseeded RNG, so runs are not reproducible across
executions. Parameter sweep results reflect averaged behaviour across 3 repeats but
exact trajectories cannot be replicated, and variance estimates may understate true
run-to-run variability.

### Parameter Sweep Coverage
Sweeps were conducted sequentially. Interaction effects between different parameters
within MPPI/EKF, as well as between them, are therefore not fully characterised. A
joint sweep over every single compute would require substantially more compute that is
not feasible for this project.

### Scan-to-Keyframe Rather Than Scan-to-Map
The ICP correction step matches the current scan against a single stored keyframe scan
rather than a persistent environment map. Corrections can only bound drift relative to
the keyframe pose as accumulated error between keyframe updates is not recovered.
As the rover moves further from the keyframe, scene overlap degrades and ICP is more
likely to converge to a wrong local minimum. A scan-to-map approach (e.g. matching
against an NDT voxel map built incrementally) would provide globally consistent
corrections regardless of distance travelled since the last keyframe.

This limitation is precisely what motivates Simultaneous Localisation and Mapping
(SLAM). Rather than correcting against a single reference scan, SLAM systems maintain
a consistent map of the environment and localise against it continuously so
corrections are globally anchored rather than relative to a recent keyframe. The
keyframe strategy implemented here is a bandaid for this underlying architectural flaw.
Arriving at this limitation through implementation rather than reading about it first gives
some intuition for why SLAM is structured the way it is.

### ICP Sensitivity to Rotation Phase
Each LiDAR revolution samples a slightly different point distribution due to continuous
sensor rotation, so consecutive scans from the same pose are not identical. This causes
ICP to find spurious small transforms even when the rover is stationary, injecting
correction noise into the EKF. The residual threshold (`error > 0.15`) and delta
magnitude gate (`|dx|, |dz| < 0.5m`, `|dθ| < 1.0 rad`) mitigate this but do not
eliminate it.

### Fixed Rejection Thresholds
Bad ICP results are rejected via fixed thresholds on residual error and transform
magnitude. These values were chosen empirically and are not derived from the ICP
residual distribution. A principled approach would characterise the relationship between
ICP residual and expected pose error, and use that model to set data-driven rejection
criteria or to weight the measurement noise covariance Q dynamically.

### H = I Approximation in EKF Correction
The EKF correction step assumes the measurement model Jacobian H = I — that is, the
sensor directly and linearly observes the full pose state [x, z, θ]. This holds when
the measurement is an absolute world-frame pose. However, the ICP measurement is a
relative delta (dx, dz, dθ) from the keyframe pose, which is reconstructed into an
absolute measurement using the stored anchor pose:

    z = [kf_x + icp.dx,  kf_z + icp.dz,  kf_heading + icp.dθ]

This reconstruction implicitly treats the anchor as exact. A more rigorous formulation 
would maintain uncertainty over the keyframe pose and propagate it through the measurement
model, producing a non-identity H and a larger effective Q at each correction step.
 
## Building
 
**Dependencies:**
- `gcc`
- OpenGL + GLUT
  - macOS: `brew install freeglut`
  - Linux: `sudo apt install freeglut3-dev`
 
```bash
make          # build
./build/lidar_sim  # run
make clean    # remove build artifacts
```
 
Builds on macOS (Apple Silicon via Homebrew) and Linux.
