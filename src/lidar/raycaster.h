
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

float cast_ray(const TriangleArray *scene, 
    const Vector3 *origin, 
    const Vector3 direction, 
    Vector3 *hit,
    float *intensity
);

#endif // RAYCASTER_H