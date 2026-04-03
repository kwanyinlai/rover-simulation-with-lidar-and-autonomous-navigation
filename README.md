# Lidar Rover Simulation

A real-time 2D LiDAR rover simulation written in C with OpenGL/GLUT rendering. The rover autonomously explores an environment using frontier-based exploration (CORRECTION: frontier-based exploration is not complete yet but will be implemented soon), building an occupancy map from raycasted LiDAR scans as it moves.

<img width="753" height="757" alt="Screenshot 2026-04-03 at 6 06 45 PM" src="https://github.com/user-attachments/assets/04cbdf64-4cd9-49a4-b01b-2dbf83e0e28a" />
<img width="735" height="662" alt="Screenshot 2026-04-03 at 6 11 26 PM" src="https://github.com/user-attachments/assets/2dd5b375-8b8f-4755-92b3-4b3437a9fef2" />
<img width="554" height="539" alt="Screenshot 2026-04-03 at 6 12 05 PM" src="https://github.com/user-attachments/assets/bbc0a642-fe72-4b86-a2e7-d5f590da5e96" />

---
 
## Features
 
- **LiDAR raycasting** — rotating sensor sweeps the scene using Möller–Trumbore ray-triangle intersection, producing a live point cloud with height-based colour gradient
- **3D occupancy mapping** — log-odds voxel grid updated via 3D DDA traversal; tracks free, occupied, and unknown cells
- **2D projected map** — column-summed occupancy map at rover height for 2D path planning and frontier visualisation
- **Frontier detection** — identifies boundaries between free and unknown space for exploration targeting
- **MPPI controller** — Model Predictive Path Integral controller with parallel rollout evaluation for autonomous navigation
- **Rover physics** — differential drive model with acceleration, friction, angular dynamics, and triangle mesh collision
- **Multi-process pipeline** — ray casting, occupancy updates, frontier analysis, and MPPI rollouts each run in separate forked processes communicating via Unix pipes
- **Manual and autonomous modes** — switch between WASD manual control and MPPI auto-navigation at runtime
- **EKF sensor fusion** — in progress; predict step implemented, correct step pending scan matching
 
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
| Heading error vs path tangent | 1.2 |
| Steering rate (jerk) | 2.0 |
| Speed deviation from reference | 0.3 |
| Throttle effort | 0.15 |
| Terminal cross-track error | 6.0 |
| Terminal heading error | 2.0 |
 
Weights are then computed as `exp(-(cost - min_cost) / λ)` and the optimal control sequence is updated as a weighted sum of all noise perturbations. The nominal sequence is warm-started from the previous frame.
 
Rollout evaluation is offloaded to the rollout coordinator process for parallelism, with a synchronous fallback in the main process if the pipe is unavailable.
 
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
 
## EKF Sensor Fusion (In Progress)
 
`rover/ekf_fusion` implements an Extended Kalman Filter over the state `[x, z, θ]`.
 
**Predict step** — implemented. Advances state via odometry delta and propagates uncertainty:
```
P = F·P·Fᵀ + Q
```
where `F` is the motion Jacobian accounting for heading-position coupling.
 
**Correct step** — pending. Requires a scan-matched pose estimate as input. Currently commented out in `rover_controller.c`.
 
```c
// EKF integration is temporarily disabled.
// ekf_fusion_predict_from_odometry(&g_pose_ekf, &odom_prediction);
// const SensorState *fused_state = ekf_fusion_get_state(&g_pose_ekf);
```
 
The red line rendered between the true sensor position (green) and the odometry estimate (blue) visualises the growing pose error that EKF correction would reduce.
 
**Reference:** [Probabilistic Robotics — Thrun, Burgard, Fox](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf)
 
---
 
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
