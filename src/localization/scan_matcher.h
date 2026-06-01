#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include "scene/point_cloud.h"

typedef struct {
    float dx;
    float dz;
    float delta_theta;
    float error; // mean closest-point distance
    int converged; // did we converge within max iterations
    uint64_t match_id; // id associated with this ICP run
    int accepted; // flag for whether this match is accepted for fusion
} ICPResult;

ICPResult run_icp(const PointCloud *current_scan,
                  const PointCloud *reference_scan,
                  int max_iterations);

#endif // SCAN_MATCHER_H