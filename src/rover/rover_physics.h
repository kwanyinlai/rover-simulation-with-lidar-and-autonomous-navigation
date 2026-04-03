#ifndef ROVER_PHYSICS_H
#define ROVER_PHYSICS_H

#include "rendering/scene.h"

/**
 * @brief Advance rover kinematics and resolve collision-constrained movement.
 * @param x In/out rover x position.
 * @param z In/out rover z position.
 * @param dir_angle In/out heading angle in radians.
 * @param speed In/out linear speed.
 * @param angular_speed In/out angular speed.
 * @param throttle Throttle input in [-1, 1].
 * @param steer Steering input in [-1, 1].
 * @param dt Time step in seconds.
 * @param scene Collision scene used for movement validation.
 * @param collision_radius Rover collision radius.
 */
void step_rover_physics(float *x,
                        float *z,
                        float *dir_angle,
                        float *speed,
                        float *angular_speed,
                        float throttle,
                        float steer,
                        float dt,
                        const TriangleArray *scene,
                        float collision_radius);

#endif // ROVER_PHYSICS_H