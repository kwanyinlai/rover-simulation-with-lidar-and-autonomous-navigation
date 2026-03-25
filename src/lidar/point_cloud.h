
/**
 * @file point_cloud.h
 * @brief Point cloud data structures and operations for lidar simulation.
 *
 * Defines the point cloud entry and container, and provides functions for managing point cloud data.
 */

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H
#include "rendering/vec3.h"

/**
 * @brief Single point entry in the point cloud.
 */
typedef struct {
    Vector3 position;   /**< 3D position of the point. */
    float distance;     /**< Distance from sensor. */
    float intensity;    /**< Surface intensity at hit. */
    float age;          /**< Age of the point (for fading/cleanup). */
} PointCloudEntry;

/**
 * @brief Container for dynamic point cloud data.
 */
typedef struct {
    PointCloudEntry *data; /**< Array of point entries. */
    size_t size;           /**< Number of points in the cloud. */
    size_t capacity;       /**< Allocated capacity. */
} PointCloud;

/**
 * @brief Add a new point to the cloud.
 */
void point_cloud_push_back(PointCloud *cloud, Vector3 position, float distance, float intensity);

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