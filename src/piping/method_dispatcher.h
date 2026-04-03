
# ifndef METHOD_DISPATCHER_H
# define METHOD_DISPATCHER_H

# include "rendering/scene.h"
# include "scene/occupancy_map.h"

/**
 * @brief Run occupancy updater process loop.
 */
void run_occupancy_updater_loop(int, int, OccupancyMap* occupancy_grid_3d);

/**
 * @brief Run lidar coordinator loop that dispatches ray work and gathers results.
 */
void run_coordinator_loop(int scan_cmd_fd, int ray_batch_results_fd, 
    int ray_task_pipes[NUM_WORKERS][2], int ray_results_pipes[NUM_WORKERS][2], 
    int point_batch_fd);

/**
 * @brief Run lidar ray worker loop.
 */
void run_worker_loop(int read_fd, int write_fd, TriangleArray*);

/**
 * @brief Run MPPI coordinator loop that shards rollout work across workers.
 */
void run_mppi_coordinator_loop(int mppi_cmd_read_fd,
                               int mppi_result_write_fd,
                               int mppi_task_pipes[NUM_WORKERS][2],
                               int mppi_result_pipes[NUM_WORKERS][2]);

/**
 * @brief Run MPPI rollout worker loop.
 */
void run_mppi_worker_loop(int read_fd, int write_fd, TriangleArray *scene);

/**
 * @brief Run frontier analysis process loop.
 */
void run_frontier_analyzer_loop(int voxel_update_read_fd,
                                int frontier_write_fd, 
                                int rover_pose_read_fd, 
                                const OccupancyMap *occupancy_grid_3d,
                                OccupancyMap *occupancy_grid_2d);

#endif // METHOD_DISPATCHER_H