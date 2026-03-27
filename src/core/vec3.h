/**
 * @file vec3.h
 * @brief 3D vector operations and data structure.
 *
 * Provides a simple 3D vector struct and basic vector math utilities for geometry and rendering.
 */

#ifndef VEC3_H
#define VEC3_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/**
 * @brief 3D vector structure.
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3;

/**
 * @brief Add two vectors.
 */
static inline Vector3 vector3_add(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

/**
 * @brief Subtract vector b from a.
 */
static inline Vector3 vector3_subtract(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

/**
 * @brief Scale a vector by a scalar.
 */
static inline Vector3 vector3_scale(Vector3 v, float scalar) {
    Vector3 result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;
    return result;
}

static inline float vector3_dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline Vector3 vector3_cross(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

static inline float vector3_magnitude(Vector3 v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline Vector3 vector3_normalize(Vector3 v) {
    float mag = vector3_magnitude(v);
    if (mag == 0) return (Vector3){0, 0, 0};
    return vector3_scale(v, 1.0f / mag);
}

# endif // VEC3_H
