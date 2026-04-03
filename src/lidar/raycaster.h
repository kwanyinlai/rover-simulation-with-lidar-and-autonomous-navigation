
/**
 * @file raycaster.h
 * @brief Raycasting interface for lidar simulation.
 *
 * Provides functions for casting rays into a scene and detecting intersections for lidar point cloud generation.
 */

#ifndef RAYCASTER_H
#define RAYCASTER_H

#include "core/vec3.h"
#include "rendering/scene.h"
#include "scene/point_cloud.h"

/**
 * @brief Cast a single ray into the scene and return first hit distance.
 * @param scene Scene geometry to test against.
 * @param origin Ray start point.
 * @param direction Normalized ray direction.
 * @param hit Output hit position when intersection occurs.
 * @param intensity Output simulated return intensity for the hit.
 * @return Hit distance in world units, or a non-positive value if no hit occurs.
 */
float cast_ray(const TriangleArray *scene, 
    const Vector3 *origin, 
    const Vector3 direction, 
    Vector3 *hit,
    float *intensity
);

#endif // RAYCASTER_H