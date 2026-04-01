# ifndef MATH_UTILS_H
# define MATH_UTILS_H

static inline float wrap_angle(float raw) {
    while (raw >  (float)M_PI) raw -= 2.0f * (float)M_PI;
    while (raw < -(float)M_PI) raw += 2.0f * (float)M_PI;
    return raw;
}

#endif