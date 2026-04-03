#ifndef SCENE_COLLISION_H
#define SCENE_COLLISION_H

#include "core/vec3.h"
#include "rendering/scene.h"

#define ROVER_COLLISION_RADIUS 0.35f

extern TriangleArray scene;

/**
 * @brief Test whether a rover footprint intersects obstacle geometry at a position.
 * @param scene Collision scene.
 * @param x Rover x coordinate.
 * @param z Rover z coordinate.
 * @param radius Rover collision radius.
 * @return Non-zero if colliding, zero otherwise.
 */
int scene_collides_rover_at(const TriangleArray *scene,
                            float x,
                            float z,
                            float radius);

/**
 * @brief Attempt movement by delta while checking intermediate collision steps.
 * @param scene Collision scene.
 * @param x In/out rover x coordinate.
 * @param z In/out rover z coordinate.
 * @param dx Desired x displacement.
 * @param dz Desired z displacement.
 * @param radius Rover collision radius.
 * @return Non-zero when movement succeeds, zero when blocked by collision.
 */
int can_move_in_dir(const TriangleArray *scene,
                      float *x,
                      float *z,
                      float dx,
                      float dz,
                      float radius);

#endif // SCENE_COLLISION_H
