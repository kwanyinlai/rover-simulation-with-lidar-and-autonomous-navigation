#ifndef NOISE_H
#define NOISE_H

#include <math.h>



// Standard normal (mean=0, stddev=1) random value using Box-Muller transform
static inline float gaussian_noise() {
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f); // avoid log(0)
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
}

#endif // NOISE_H