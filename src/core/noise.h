#ifndef NOISE_H
#define NOISE_H

#include <math.h>

// Generate a standard normal (mean=0, stddev=1) random value using the Box-Muller transform.
// @see https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
static inline float gaussian_noise() {
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f); 
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

#endif // NOISE_H