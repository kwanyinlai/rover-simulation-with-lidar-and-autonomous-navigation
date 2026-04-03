#ifndef ROVER_CONTROLLER_H
#define ROVER_CONTROLLER_H

#define ROVER_HEIGHT_CELLS 3

// odometry noise
#define SPEED_NOISE 0.0f
#define ANGULAR_NOISE 0.0f


#include "rendering/scene.h"
#include "scene/point_cloud.h"


/**
 * @brief Currently active rover waypoint path.
 */
extern Path active_path;

/**
 * @brief Initialize rover controller state and MPPI warm-start controls.
 */
void init_rover_controller(void);

/**
 * @brief Set pipe write descriptor used to request replanning.
 * @param write_fd Write descriptor for rover pose messages.
 */
void set_replan_pipe_fd(int write_fd);

/**
 * @brief Set MPPI IPC descriptors for distributed rollout evaluation.
 * @param cmd_write_fd Write descriptor for rollout requests.
 * @param result_read_fd Read descriptor for rollout cost results.
 */
void set_rollout_pipe_fds(int cmd_write_fd, int result_read_fd);

/**
 * @brief Compute MPPI rollout cost for one control/noise sequence.
 * @param scene Collision scene used during rollout simulation.
 * @param path Path used for cross-track and heading costs.
 * @param init_state Initial state at rollout step 0.
 * @param horizon Number of rollout steps.
 * @param nom_steer Nominal steer sequence.
 * @param nom_throttle Nominal throttle sequence.
 * @param steer_noise Steer perturbation sequence.
 * @param throttle_noise Throttle perturbation sequence.
 * @return Total rollout cost (lower is better).
 */
float mppi_compute_rollout_cost(const TriangleArray *scene,
                                const Path *path,
                                SimState init_state,
                                int horizon,
                                const float *nom_steer,
                                const float *nom_throttle,
                                const float *steer_noise,
                                const float *throttle_noise);

/**
 * @brief Update dead-reckoned rover pose from current controls.
 * @param dt Simulation time step in seconds.
 */
void update_odometry(float dt);

/**
 * @brief Update fused rover pose estimate with latest lidar scan evidence.
 * @param cloud Current point cloud buffer.
 * @param scan_theta Current horizontal scan angle in radians.
 */
void update_lidar_fusion(const PointCloud *cloud, float scan_theta);

/**
 * @brief Run path-following control update and emit throttle/steer commands.
 * @param dt Simulation time step in seconds.
 */
void update_path_follower(float dt);

/**
 * @brief Replace active path with a new waypoint sequence.
 * @param wps Waypoint array.
 * @param count Number of waypoints in @p wps.
 */
void set_waypoints(Waypoint *wps, int count);

/**
 * @brief Render line indicating odometry error against sensor truth.
 */
void render_pose_error(void);

/**
 * @brief Render active and upcoming waypoints.
 */
void render_waypoints(void);

/**
 * @brief Render MPPI-predicted future rover trajectory.
 */
void render_predicted_path(void);



# endif // ROVER_CONTROLLER_H