
/**
 * @file renderer.h
 * @brief Rendering interface for lidar simulation.
 *
 * Provides functions for rendering the scene, point cloud, and sensor visualization.
 */

#ifndef RENDERER_H
#define RENDERER_H

#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"
/**
 * @brief Render the scene geometry as wireframe.
 */
void render_wire();

/**
 * @brief Render the lidar point cloud.
 * @param cloud The point cloud to render.
 * @param dt Time delta for animation/fading.
 */
void render_cloud(PointCloud *cloud, float dt);

void render_occupancy_map(const OccupancyMap *map);

/**
 * @brief Render the lidar sensor visualization.
 */
void render_sensor();


#endif // RENDERER_H