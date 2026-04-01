
/**
 * @file scene/scene
 * @brief Scene geometry definitions and utilities.
 *
 * Provides data structures and functions for representing and manipulating 3D scene geometry.
 */

#ifndef SCENE_H
#define SCENE_H

#include "core/vec3.h"

/**
 * @brief Triangle primitive for scene geometry.
 */
typedef struct {
    Vector3 v0, v1, v2;
} Triangle;

/**
 * @brief Dynamic array of triangles representing the scene.
 */
typedef struct {
    Triangle *data;
    size_t size;
    size_t capacity;
} TriangleArray;

void triangle_array_init(TriangleArray *array);
void triangle_array_free(TriangleArray *array);
void triangle_array_push_back(TriangleArray *array, Triangle triangle);
void mesh_add_quad(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3);
void mesh_add_triangle(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2);
void mesh_add_quad_tesselated(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3, int divs);
void mesh_add_box(TriangleArray *scene, float centre_x, float centre_y, float centre_z,
                    float half_width, float half_height, float half_depth);
void build_scene(TriangleArray *scene);

#endif // SCENE_H