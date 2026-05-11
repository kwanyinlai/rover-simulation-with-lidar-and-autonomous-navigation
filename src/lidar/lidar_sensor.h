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
typedef struct {
	PointCloud current;
	PointCloud latest;
	PointCloud previous;
	int has_previous; // one-time flag to track if we have a valid previous sweep
	int sweep_ready; // new sweep pair ready for retrieval
} ScanState;

/**
 * @brief Advance the sensor simulation by one step and collect point cloud data.
 * @param scene Pointer to the scene geometry.
 * @param point_cloud Output point cloud to store detected points.
 * @param occupancy_grid_3d Pointer to the occupancy grid.
 */
void sensor_step(const TriangleArray *scene,
				 PointCloud *point_cloud,
				 OccupancyMap *occupancy_grid_3d,
				 ScanState *scan_state);

/**
 * @brief Get the current horizontal scan angle in radians.
 * @return Current scan angle.
 */
float get_scan_theta(void);

/**
 * @brief Initialize vertical ray elevation angles for one full lidar revolution.
 */
void init_sensor_rays(void);

/**
 * @brief Initialize scan tracking state
 */
void init_scan_state(ScanState *scan_state);

/**
 * @brief Retrieve the most recent completed sweep pair
 * @param latest Output pointer to latest completed sweep.
 * @param previous Output pointer to the previous completed sweep.
 * @return 1 if a new sweep pair is available, 0 otherwise.
 */
int poll_scan_pair(ScanState *scan_state,
				   const PointCloud **latest,
				   const PointCloud **previous);

#endif // LIDAR_SENSOR_H

