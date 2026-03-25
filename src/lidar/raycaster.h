
/**
 * @file raycaster.h
 * @brief Raycasting interface for lidar simulation.
 *
 * Provides functions for casting rays into a scene and detecting intersections for lidar point cloud generation.
 */

#ifndef RAYCASTER_H
#define RAYCASTER_H

#include "rendering/vec3.h"
#include "rendering/scene.h"
#include "lidar/point_cloud.h"

/**
 * @brief Cast a ray into the scene and find the closest intersection.
 * @param scene Pointer to the scene geometry.
 * @param origin Ray origin.
 * @param direction Ray direction (normalized).
 * @param hit Output: intersection point if hit.
 * @param intensity Output: surface intensity at hit point.
 * @return Distance to intersection, or <0 if no hit.
 */
float cast_ray(const TriangleArray *scene, 
    const Vector3 *origin, 
    const Vector3 direction, 
    Vector3 *hit,
    float *intensity
);

#endif // RAYCASTER_H