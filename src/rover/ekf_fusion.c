#include "rover/ekf_fusion.h"

#include "core/math_utils.h"
#include "core/physics_constants.h"

#include <math.h>
#include <string.h>

#define EKF_DEFAULT_SPEED_NOISE 0.05f
#define EKF_DEFAULT_ANGULAR_NOISE (5.0f * MATH_DEG_TO_RAD)
#define EKF_DEFAULT_LIDAR_NOISE 0.2f


// Sensor Fusion with Extended Kalman Filter (EKF)
// https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf

// matrix operations for small fixed-sized matrices
static void matrix_add(const float a[EKF_STATE_DIM][EKF_STATE_DIM],
                       const float b[EKF_STATE_DIM][EKF_STATE_DIM],
                       float out[EKF_STATE_DIM][EKF_STATE_DIM]) {
    for (int r = 0; r < EKF_STATE_DIM; r++) {
        for (int c = 0; c < EKF_STATE_DIM; c++) {
            out[r][c] = a[r][c] + b[r][c];
        }
    }
}

static void matrix_transpose(const float in[EKF_STATE_DIM][EKF_STATE_DIM],
                             float out[EKF_STATE_DIM][EKF_STATE_DIM]) {
    for (int r = 0; r < EKF_STATE_DIM; r++) {
        for (int c = 0; c < EKF_STATE_DIM; c++) {
            out[c][r] = in[r][c];
        }
    }
}

static void matrix_mult(const float a[EKF_STATE_DIM][EKF_STATE_DIM],
                        const float b[EKF_STATE_DIM][EKF_STATE_DIM],
                        float out[EKF_STATE_DIM][EKF_STATE_DIM]) {
    for (int r = 0; r < EKF_STATE_DIM; r++) {
        for (int c = 0; c < EKF_STATE_DIM; c++) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_STATE_DIM; k++) {
                sum += a[r][k] * b[k][c];
            }
            out[r][c] = sum;
        }
    }
}

void ekf_fusion_init(KalmanFilter *ekf, const SensorState *initial_state) {
    if (!ekf || !initial_state) {
        return;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->state = *initial_state;

    // set diagonal of covariance matrix to initial values, 0 elsewhere
    ekf->P[0][0] = 0.1f * 0.1f; // initial x uncertainty
    ekf->P[1][1] = 0.1f * 0.1f; // initial z uncertainty
    ekf->P[2][2] = (5.0f * MATH_DEG_TO_RAD) * (5.0f * MATH_DEG_TO_RAD); // initial angle uncertainty

    // process noise covariance is odometry noise
    ekf->Q[0][0] = EKF_DEFAULT_SPEED_NOISE * EKF_DEFAULT_SPEED_NOISE; // x
    ekf->Q[1][1] = EKF_DEFAULT_SPEED_NOISE * EKF_DEFAULT_SPEED_NOISE; // z
    ekf->Q[2][2] = (EKF_DEFAULT_ANGULAR_NOISE * MATH_DEG_TO_RAD) * 
                   (EKF_DEFAULT_ANGULAR_NOISE * MATH_DEG_TO_RAD); // angle

    // measurement noise covariance is lidar noise
    // TODO: I don't think this is right, come back to this
    ekf->R[0][0] = EKF_DEFAULT_LIDAR_NOISE * EKF_DEFAULT_LIDAR_NOISE; // x
    ekf->R[1][1] = EKF_DEFAULT_LIDAR_NOISE * EKF_DEFAULT_LIDAR_NOISE; // z
    ekf->R[2][2] = (10.0f * MATH_DEG_TO_RAD) * (10.0f * MATH_DEG_TO_RAD); // angle
}

void ekf_fusion_predict_from_odometry(KalmanFilter *ekf, const SensorState *odom_prediction) {

    float theta = ekf->state.dir_angle;
    float dx = odom_prediction->origin.x - ekf->state.origin.x;
    float dz = odom_prediction->origin.z - ekf->state.origin.z;
    float delta_theta = wrap_angle(odom_prediction->dir_angle - theta);

    ekf->state.origin.x += dx;
    ekf->state.origin.z += dz;
    ekf->state.dir_angle = wrap_angle(theta + delta_theta);

    float d = sqrtf(dx*dx + dz*dz);

    // Jacobian of F
    float F[EKF_STATE_DIM][EKF_STATE_DIM] =  {
        {1, 0, -d * sinf(theta)},
        {0, 1, d * cosf(theta)},
        {0, 0, 1}
    };

    // P' = F * P * F.T + Q
    float F_t[EKF_STATE_DIM][EKF_STATE_DIM];
    float FP[EKF_STATE_DIM][EKF_STATE_DIM];
    float FPF_t[EKF_STATE_DIM][EKF_STATE_DIM];
    matrix_transpose(F, F_t);
    matrix_mult(F, ekf->P, FP);
    matrix_mult(FP, F_t, FPF_t);
    matrix_add(FPF_t, ekf->Q, ekf->P);
}

void ekf_fusion_correct_step(KalmanFilter *ekf, const PointCloud *cloud, float scan_theta) {
    
    // TODO: this is tough. requires localization like SLAM and I don't have time to write out a 
    // full SLAM implementation here
    (void) ekf;
    (void) cloud;
    (void) scan_theta;
}

const SensorState *ekf_fusion_get_state(const KalmanFilter *ekf) {
    return &ekf->state;
}