
# ifndef METHOD_DISPATCHER_H
# define METHOD_DISPATCHER_H

# include "rendering/scene.h"
# include "scene/occupancy_map.h"


extern int g_scan_cmd_fd;

void run_occupancy_updater_loop(int, int, OccupancyMap* occupancy_grid_3d);

void run_coordinator_loop(int scan_cmd_fd, int ray_batch_results_fd, 
    int ray_task_pipes[NUM_WORKERS][2], int ray_results_pipes[NUM_WORKERS][2], 
    int point_batch_fd);

void run_worker_loop(int read_fd, int write_fd, TriangleArray*);

void run_frontier_analyzer_loop(int read_fd, int write_fd, const OccupancyMap *occupancy_grid_3d, OccupancyMap *occupancy_grid_2d);

#endif // METHOD_DISPATCHER_H