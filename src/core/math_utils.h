# ifndef MATH_UTILS_H
#define MATH_UTILS_H

/**
 * @brief Wrap angle to the range [-pi, pi].
 * @param raw_angle Input angle in radians.
 * @return Wrapped angle in radians.
 */
static inline float wrap_angle(float raw_angle) {
    while (raw_angle > (float)M_PI) raw_angle -= 2.0f * (float)M_PI;
    while (raw_angle < -(float)M_PI) raw_angle += 2.0f * (float)M_PI;
    return raw_angle;
}

#endif