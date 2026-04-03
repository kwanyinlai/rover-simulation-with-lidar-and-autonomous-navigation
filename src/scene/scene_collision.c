#include "scene/scene_collision.h"

#include <math.h>
#include <stddef.h>

#define STEP_LENGTH 0.02f

static float point_sq_dist_to_line_segment(float point_x,
                                           float point_z,
                                           float segment_start_x,
                                           float segment_start_z,
                                           float segment_end_x,
                                           float segment_end_z) {
    float segment_dx = segment_end_x - segment_start_x;
    float segment_dz = segment_end_z - segment_start_z;

    // calculate projection
    float t = (
        (point_x - segment_start_x) * segment_dx + 
        (point_z - segment_start_z) * segment_dz
    ) / (
        segment_dx * segment_dx + segment_dz * segment_dz
    );

    if (t < 0.0f) t = 0.0f; 
    if (t > 1.0f) t = 1.0f; // clamp to either end of segment

    float closest_x = segment_start_x + t * segment_dx;
    float closest_z = segment_start_z + t * segment_dz;

    // calculate distance
    float dx = point_x - closest_x;
    float dz = point_z - closest_z;
    return dx * dx + dz * dz;
}

static float point_sq_dist_to_triangle(const Triangle *triangle,
                                           float point_x,
                                           float point_z)
{
    float sq_dist_to_v0_v1 = point_sq_dist_to_line_segment(point_x, point_z,
                                                               triangle->v0.x, triangle->v0.z,
                                                               triangle->v1.x, triangle->v1.z);
    float sq_dist_to_v1_v2 = point_sq_dist_to_line_segment(point_x, point_z,
                                                               triangle->v1.x, triangle->v1.z,
                                                               triangle->v2.x, triangle->v2.z);
    float sq_dist_to_v2_v0 = point_sq_dist_to_line_segment(point_x, point_z,
                                                               triangle->v2.x, triangle->v2.z,
                                                               triangle->v0.x, triangle->v0.z);

    float min_sq_dist = sq_dist_to_v0_v1;
    if (sq_dist_to_v1_v2 < min_sq_dist) min_sq_dist = sq_dist_to_v1_v2;
    if (sq_dist_to_v2_v0 < min_sq_dist) min_sq_dist = sq_dist_to_v2_v0;
    return min_sq_dist;
}

int is_rover_scene_collision(const TriangleArray *scene,
                             float x,
                             float z,
                             float rover_rad)
{

    float rover_rad_sq = rover_rad * rover_rad;

    for (size_t triangle_idx = 0; triangle_idx < scene->size; triangle_idx++) {
        const Triangle *triangle = &(scene->data[triangle_idx]);
        Vector3 edge1 = vector3_subtract(triangle->v1, triangle->v0);
        Vector3 edge2 = vector3_subtract(triangle->v2, triangle->v0);
        Vector3 normal = vector3_cross(edge1, edge2);  

        
        normal = vector3_normalize(normal);
        

        // this is a naive way to ignore floor collisions, should probably use RANSAC
        // but this is sufficient for our scene and scope of project
        if (fabsf(normal.y) > 1e-6f) { 
            continue;
        }

        if (point_sq_dist_to_triangle(triangle, x, z) <= rover_rad_sq) {
            return 1;
        }
    }

    return 0;
}

int can_move_in_dir(const TriangleArray *scene,
                    float *x,
                    float *z,
                    float dx,
                    float dz,
                    float rover_rad)
{
    float move_distance = sqrtf(dx * dx + dz * dz);
    if (move_distance < 1e-6f) {
        return 1;
    }

    int steps = (int)ceilf(move_distance / STEP_LENGTH);
    if (steps < 1) {
        steps = 1;
    }

    float per_step_dx = dx / (float)steps;
    float per_step_dz = dz / (float)steps;
    float current_x = *x;
    float current_z = *z;

    for (int step_idx = 0; step_idx < steps; step_idx++) {
        float step_x = current_x + per_step_dx;
        float step_z = current_z + per_step_dz;

        if (is_rover_scene_collision(scene, step_x, step_z, rover_rad)) {
            *x = current_x;
            *z = current_z;
            return 0;
        }

        current_x = step_x;
        current_z = step_z;
    }

    *x = current_x;
    *z = current_z;
    return 1;
}
