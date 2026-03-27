
/**
 * @file point_cloud.h
 * @brief Point cloud data structures and operations for lidar simulation.
 *
 * Defines the point cloud entry and container, and provides functions for managing point cloud data.
 */

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "core/vec3.h"

# include "piping/messages.h"

/**
 * @brief Single point entry in the point cloud.
 */
typedef struct {
    Vector3 position;
    float distance;
    float intensity;
    float age;
} PointCloudEntry;

/**
 * @brief Container for dynamic point cloud data.
 */
typedef struct {
    PointCloudEntry *data;
    size_t size;
    size_t capacity;
} PointCloud;

/**
 * @brief Add a new point to the cloud.
 */
void point_cloud_push_back(PointCloud *cloud, Vector3 position, float distance, float intensity);

void point_cloud_push_back_multiple(PointCloud *cloud, RayResult *rays, size_t count);

/**
 * @brief Free memory used by the point cloud.
 */
void point_cloud_free(PointCloud *cloud);

/**
 * @brief Initialize a point cloud structure.
 */
void init_point_cloud(PointCloud *cloud);

/**
 * @brief Age all points in the cloud and remove old points.
 * @param cloud The point cloud to update.
 * @param delta_time Time increment to add to each point's age.
 */
void point_cloud_age(PointCloud *cloud, float delta_time);

#endif // POINT_CLOUD_H