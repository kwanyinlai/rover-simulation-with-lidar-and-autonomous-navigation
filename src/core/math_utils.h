# ifndef MATH_UTILS_H
# define MATH_UTILS_H

/**
 * @brief Wrap angle to the range [-pi, pi].
 * @param raw Input angle in radians.
 * @return Wrapped angle in radians.
 */
static inline float wrap_angle(float raw) {
    while (raw >  (float)M_PI) raw -= 2.0f * (float)M_PI;
    while (raw < -(float)M_PI) raw += 2.0f * (float)M_PI;
    return raw;
}

#endif