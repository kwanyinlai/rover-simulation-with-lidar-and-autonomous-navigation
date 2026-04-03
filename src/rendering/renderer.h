
/**
 * @file renderer.h
 * @brief Rendering interface for lidar simulation.
 *
 * Provides functions for rendering the scene, point cloud, and sensor visualization.
 */

#ifndef RENDERER_H
#define RENDERER_H

#include "scene/point_cloud.h"
#include "scene/occupancy_map.h"
/**
 * @brief Render the scene geometry as wireframe.
 */
void render_wire(void);

/**
 * @brief Render the lidar point cloud.
 * @param cloud The point cloud to render.
 * @param dt Time delta for animation/fading.
 */
void render_cloud(PointCloud *cloud, float dt);

/**
 * @brief Render occupancy cells in the provided occupancy map.
 * @param occupancy_grid_3d Occupancy map to visualize.
 */
void render_occupancy_map(const OccupancyMap *occupancy_grid_3d);

/**
 * @brief Render the lidar sensor visualization.
 */
void render_sensor(void);


#endif // RENDERER_H