#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H


#include "rendering/scene.h"
#include "scene/point_cloud.h"
#include "scene/occupancy_map.h"
#include "lidar/sensor_control.h"

#define MAX_RANGE 30.f

/**
 * @brief Configure IPC file descriptors used by the lidar scan pipeline.
 * @param scan_cmd_fd Write descriptor for scan requests.
 * @param ray_batch_results_fd Read descriptor for ray result batches.
 */
void set_scan_pipe_fds(int scan_cmd_fd, int ray_batch_results_fd);

extern float elevations[NUM_RINGS];

/**
 * @brief Advance the sensor simulation by one step and collect point cloud data.
 * @param scene Pointer to the scene geometry.
 * @param point_cloud Output point cloud to store detected points.
 * @param occupancy_grid_3d Pointer to the occupancy grid.
 */
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *occupancy_grid_3d);

/**
 * @brief Get the current horizontal scan angle in radians.
 * @return Current scan angle.
 */
float get_scan_theta(void);

/**
 * @brief Initialize vertical ray elevation angles for one full lidar revolution.
 */
void init_sensor_rays(void);

#endif // LIDAR_SENSOR_H

