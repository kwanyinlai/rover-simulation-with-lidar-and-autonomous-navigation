
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

/**
 * @brief Initialize an empty triangle array.
 * @param array Target triangle array.
 */
void triangle_array_init(TriangleArray *array);

/**
 * @brief Release memory held by a triangle array.
 * @param array Triangle array to free.
 */
void triangle_array_free(TriangleArray *array);

/**
 * @brief Append a triangle to the dynamic array.
 * @param array Target triangle array.
 * @param triangle Triangle to append.
 */
void triangle_array_push_back(TriangleArray *array, Triangle triangle);

/**
 * @brief Add a quad to the scene as two triangles.
 */
void mesh_add_quad(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3);

/**
 * @brief Add a single triangle to the scene.
 */
void mesh_add_triangle(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2);

/**
 * @brief Add a tessellated quad mesh to the scene.
 * @param divs Number of subdivisions per edge.
 */
void mesh_add_quad_tesselated(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3, int divs);

/**
 * @brief Add an axis-aligned box primitive to the scene.
 */
void mesh_add_box(TriangleArray *scene, float centre_x, float centre_y, float centre_z,
                    float half_width, float half_height, float half_depth);

/**
 * @brief Build the default test scene geometry.
 * @param scene Output scene geometry.
 */
void build_scene(TriangleArray *scene);

#endif // SCENE_H