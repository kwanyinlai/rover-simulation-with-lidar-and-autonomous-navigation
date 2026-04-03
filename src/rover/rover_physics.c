#include "rover/rover_physics.h"

#include "scene/scene_collision.h"
#include "core/physics_constants.h"

#include <math.h>

void step_rover_physics(float *x,
                        float *z,
                        float *dir_angle,
                        float *speed,
                        float *angular_speed,
                        float throttle,
                        float steer,
                        float dt,
                        const TriangleArray *scene,
                        float collision_radius)
{
    if (throttle != 0.0f) {
        *speed += throttle * ACCELERATION * dt;
        *speed = fmaxf(-MAX_SPEED, fminf(MAX_SPEED, *speed));
    } else {
        float friction = FRICTION * dt;
        if (*speed > 0.0f) {
            *speed = fmaxf(0.0f, *speed - friction);
        } else {
            *speed = fminf(0.0f, *speed + friction);
        }
    }

    if (steer != 0.0f) {
        *angular_speed += steer * ANGULAR_ACCELERATION * dt;
        *angular_speed = fmaxf(-MAX_ANGULAR_SPEED, fminf(MAX_ANGULAR_SPEED, *angular_speed));
    } else {
        float friction = ANGULAR_FRICTION * dt;
        if (fabsf(*angular_speed) < friction) {
            *angular_speed = 0.0f;
        } else {
            *angular_speed -= friction * (*angular_speed > 0.0f ? 1.0f : -1.0f);
        }
    }

    *dir_angle += *angular_speed * dt;
    float dx = cosf(*dir_angle) * *speed * dt;
    float dz = sinf(*dir_angle) * *speed * dt;
    if (!can_move_in_dir(scene, x, z, dx, dz, collision_radius)) {
        *speed = 0.0f;
    }
}