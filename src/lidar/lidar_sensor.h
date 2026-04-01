
/**
 * @file lidar_sensor.h
 * @brief Lidar sensor simulation interface.
 *
 * Provides functions for initializing, updating, and moving a simulated lidar sensor,
 * as well as retrieving its position. Integrates with scene geometry and point cloud output.
 */

# ifndef LIDAR_SENSOR_H
# define LIDAR_SENSOR_H


# define MATH_DEG_TO_RAD (M_PI / 180.0f)
# define MAX_RANGE 30.f

#include "rendering/scene.h"
#include "scene/point_cloud.h"
#include "scene/occupancy_map.h"
#include "lidar/sensor_control.h"


extern int g_scan_cmd_fd; // file descriptor for sending scan commands to coordinator process, set in main.c after forking processes
extern int g_ray_batch_results_fd; // file descriptor for receiving ray batch results from worker processes, set in main.c after forking processes

float elevations[NUM_RINGS];

/**
 * @brief Advance the sensor simulation by one step and collect point cloud data.
 * @param scene Pointer to the scene geometry.
 * @param point_cloud Output point cloud to store detected points.
 * @param occupancy_grid_3d Pointer to the occupancy grid.
 */
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *occupancy_grid_3d);

#include "core/noise.h"


void init_sensor_rays(void);

#endif // LIDAR_SENSOR_H

