#include "localization/scan_matcher.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>

#define CONVERGENCE_THRESHOLD (1e-6f)


static int nearest_neighbor(float qx, float qz, const PointCloud *reference_scan) {
    float smallest_dist_sqr = FLT_MAX;
    int smallest_dist_idx  = 0;
    for (int i = 0; i < reference_scan->size; i++) {
        float dx = reference_scan->data[i].position.x - qx;
        float dz = reference_scan->data[i].position.z - qz;
        float dist_sqr = dx * dx + dz * dz;
        if (dist_sqr < smallest_dist_sqr) { 
            smallest_dist_sqr = dist_sqr; 
            smallest_dist_idx = i; 
        }
    }
    return smallest_dist_idx;
}


// TODO: couple of things to add here:
// - ICP assume stationary, so perhaps stationary scans must be used as reference
// - in that case, we may need to store PointCloud snapshots in some rolling buffer of most recent local PCs
// - we will just implement ICP directly
// - raw point cloud is expensive, may need to store spatial hash or something for fast NN search
// - we are working with a static scene so this is effective but in real autonomous rovers scenes are generally dynamic
//.  or at laeast slow moving
// https://learnopencv.com/iterative-closest-point-icp-explained/
ICPResult run_icp(const PointCloud *current_scan,   // source: current scan
                  const PointCloud *reference_scan, // target: reference scan
                  int max_iterations) {

    ICPResult result = {0};
    int n = current_scan->size;

    float *P[2] = {malloc(n * sizeof(float)), malloc(n * sizeof(float))};
    float *Q[2] = {malloc(n * sizeof(float)), malloc(n * sizeof(float))};

    // Seed P with the original source positions.
    for (int i = 0; i < n; i++) {
        P[0][i] = current_scan->data[i].position.x;
        P[1][i] = current_scan->data[i].position.z;
    }

    // T_total = (R, t), composes incremental transformations in form rotation R and then translation vector t
    float T_total[3] = {0.0f, 0.0f, 0.0f}; // d_theta, dx, dz
    float T_total[0] = 0.0f;
    float T_total[1] = 0.0f;
    float T_total[2] = 0.0f;

    float prev_error = FLT_MAX;
    

    for (int iter = 0; iter < max_iterations; ++iter) {

        // mu = centroids, used for centering point sets
        // error_sum used for convergence check
        // Q populated with nearest neighbours each iteration
        float mu_P[2] = {0.0f, 0.0f};
        float mu_Q[2] = {0.0f, 0.0f};
        float error_sum = 0.0f;

        for (int i = 0; i < n; i++) {
            int nn = nearest_neighbor(P[0][i], P[1][i], reference_scan);
            Q[0][i] = reference_scan->data[nn].position.x;
            Q[1][i] = reference_scan->data[nn].position.z;

            float error_x = Q[0][i] - P[0][i];
            float error_z = Q[1][i] - P[1][i];
            error_sum += sqrtf(error_x * error_x + error_z * error_z);

            mu_P[0] += P[0][i];
            mu_P[1] += P[1][i];
            mu_Q[0] += Q[0][i];
            mu_Q[1] += Q[1][i];
        }

        result.error = error_sum / (float)n;
        mu_P[0] /= n;
        mu_P[1] /= n;
        mu_Q[0] /= n;
        mu_Q[1] /= n;

        // Cross-covariance matrix H between centered point sets P and Q
        // Build H = Sigma_i(p_i q_i^T)
        // Recover rotation (arctan equivalent to SVD in 2D)
        //   θ = atan2(H01 - H10, H00 + H11)
        float H[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

        for (int i = 0; i < n; i++) {
            float P_centred[2] = {P[0][i] - mu_P[0], P[1][i] - mu_P[1]};
            float Q_centred[2] = {Q[0][i] - mu_Q[0], Q[1][i] - mu_Q[1]};
            H[0][0] += P_centred[0] * Q_centred[0];
            H[0][1] += P_centred[0] * Q_centred[1];
            H[1][0] += P_centred[1] * Q_centred[0];
            H[1][1] += P_centred[1] * Q_centred[1];
        }

        float theta = atan2f(H[0][1] - H[1][0], H[0][0] + H[1][1]);
        float R[2][2] = {{ cosf(theta), -sinf(theta) },
                         { sinf(theta),  cosf(theta) }}; // build rotation matrix

        // translation vector, t
        // t = mu_Q - R * mu_P
        float t[2] = {
            mu_Q[0] - (R[0][0] * mu_P[0] + R[0][1] * mu_P[1]),
            mu_Q[1] - (R[1][0] * mu_P[0] + R[1][1] * mu_P[1])
        };

        // increental transformation
        for (int i = 0; i < n; i++) {
            float R_p_i[2] = {
                R[0][0] * P[0][i] + R[0][1] * P[1][i],
                R[1][0] * P[0][i] + R[1][1] * P[1][i]
            }; // apply rotation
            P[0][i] = R_p_i[0] + t[0];
            P[1][i] = R_p_i[1] + t[1];
        }

        // accumulate T_total = (R, t) with previous transformations
        // t is expressed in the current frame, so rotate back to original frame
        // as reference
        float R_total[2][2] = {{ cosf(T_total[0]), -sinf(T_total[0]) },
                               { sinf(T_total[0]),  cosf(T_total[0]) }};
        T_total[0] += theta;
        T_total[1] += R_total[0][0] * t[0] + R_total[0][1] * t[1];
        T_total[2] += R_total[1][0] * t[0] + R_total[1][1] * t[1];

        // convergence check
        if (fabsf(prev_error - result.error) < CONVERGENCE_THRESHOLD) {
            result.converged = 1;
            break;
        }
        prev_error = result.error;
    }

    free(P[0]);
    free(P[1]);
    free(Q[0]);
    free(Q[1]);

    result.delta_theta = T_total[0];
    result.dx = T_total[1];
    result.dz = T_total[2];
    return result;
}