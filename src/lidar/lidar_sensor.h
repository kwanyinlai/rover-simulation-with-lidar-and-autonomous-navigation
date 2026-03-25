
/**
 * @file lidar_sensor.h
 * @brief Lidar sensor simulation interface.
 *
 * Provides functions for initializing, updating, and moving a simulated lidar sensor,
 * as well as retrieving its position. Integrates with scene geometry and point cloud output.
 */

#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H


# define NUM_RINGS 256

#include "rendering/scene.h"
#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"

/**
 * @brief Initialize the internal state of the lidar sensor.
 */
void init_sensor_state();

/**
 * @brief Advance the sensor simulation by one step and collect point cloud data.
 * @param scene Pointer to the scene geometry.
 * @param point_cloud Output point cloud to store detected points.
 * @param map Pointer to the occupancy map.
 */
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *map);

/**
 * @brief Move the sensor in the scene.
 * @param forward Amount to move forward.
 * @param side Amount to move sideways.
 */
void sensor_move(float forward, float side);

/**
 * @brief Get the current position of the sensor.
 * @param pos Output vector for the sensor position.
 */
void get_sensor_pos(Vector3 *pos);

#endif  // LIDAR_SENSOR_H