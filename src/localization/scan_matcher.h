#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include "scene/point_cloud.h"

typedef struct {
    float dx;
    float dz;
    float delta_theta;
    float error; // mean closest-point distance
    int converged; // did we converge within max iterations
} ICPResult;

ICPResult run_icp(const PointCloud *current_scan,
                  const PointCloud *reference_scan,
                  int max_iterations);

#endif // SCAN_MATCHER_H