#include "rover/ekf_fusion.h"

#include "core/math_utils.h"
#include "core/physics_constants.h"
#include "localization/scan_matcher.h"
#include "lidar/sensor_control.h"
#include "core/io_utils.h"
#include "localization/scan_matcher.h"

#include <math.h>
#include <string.h>

#define EKF_DEFAULT_SPEED_NOISE 0.05f
#define EKF_DEFAULT_ANGULAR_NOISE (5.0f * MATH_DEG_TO_RAD)
#define EKF_DEFAULT_LIDAR_NOISE 0.2f
#define EKF_DEFAULT_ANGLE_NOISE (10.0f * MATH_DEG_TO_RAD)

static int g_synthetic_scan_cmd_fd = -1;
static int g_synthetic_scan_res_fd = -1;

void set_scan_match_pipe_fds(int cmd_fd, int res_fd) {
    g_synthetic_scan_cmd_fd = cmd_fd;
    g_synthetic_scan_res_fd = res_fd;
}

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

static void matrix_inverse(const float in[EKF_MEAS_DIM][EKF_MEAS_DIM],
                           float out[EKF_MEAS_DIM][EKF_MEAS_DIM]) 
{
    float det = in[0][0] * (in[1][1] * in[2][2] - in[1][2] * in[2][1]) -
                in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0]) +
                in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);
 
    float inv_det = 1.0f / det;
    out[0][0] = inv_det * (in[1][1] * in[2][2] - in[1][2] * in[2][1]);
    out[0][1] = inv_det * (in[0][2] * in[2][1] - in[0][1] * in[2][2]);
    out[0][2] = inv_det * (in[0][1] * in[1][2] - in[0][2] * in[1][1]);
    out[1][0] = inv_det * (in[1][2] * in[2][0] - in[1][0] * in[2][2]);
    out[1][1] = inv_det * (in[0][0] * in[2][2] - in[0][2] * in[2][0]);
    out[1][2] = inv_det * (in[0][2] * in[1][0] - in[0][0] * in[1][2]);
    out[2][0] = inv_det * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);
    out[2][1] = inv_det * (in[0][1] * in[2][0] - in[0][0] * in[2][1]);
    out[2][2] = inv_det * (in[0][0] * in[1][1] - in[0][1] * in[1][0]);
}

void ekf_fusion_init(KalmanFilter *ekf, const SensorState *initial_state) {
    if (!ekf || !initial_state) {
        return;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->state = *initial_state;

    // set diagonal of covariance matrix to initial values, 0 elsewhere
    ekf->Sigma[0][0] = 0.1f * 0.1f; // initial x uncertainty
    ekf->Sigma[1][1] = 0.1f * 0.1f; // initial z uncertainty
    ekf->Sigma[2][2] = (5.0f * MATH_DEG_TO_RAD) * (5.0f * MATH_DEG_TO_RAD); // initial angle uncertainty

    // process noise covariance is odometry noise
    ekf->R_t[0][0] = EKF_DEFAULT_SPEED_NOISE * EKF_DEFAULT_SPEED_NOISE; // x
    ekf->R_t[1][1] = EKF_DEFAULT_SPEED_NOISE * EKF_DEFAULT_SPEED_NOISE; // z
    ekf->R_t[2][2] = EKF_DEFAULT_ANGULAR_NOISE * EKF_DEFAULT_ANGULAR_NOISE; // angle

    // measurement noise covariance is lidar noise
    // TODO: I don't think this is right, come back to this
    ekf->Q_t[0][0] = EKF_DEFAULT_LIDAR_NOISE * EKF_DEFAULT_LIDAR_NOISE; // x
    ekf->Q_t[1][1] = EKF_DEFAULT_LIDAR_NOISE * EKF_DEFAULT_LIDAR_NOISE; // z
    ekf->Q_t[2][2] = EKF_DEFAULT_ANGLE_NOISE * EKF_DEFAULT_ANGLE_NOISE; // angle
}

// Generates a synthetic LiDAR scan from the EKF's estimated pose
// mimicing logic in lidar_sensor.c
static void generate_synthetic_scan(KalmanFilter *ekf, float scan_theta, PointCloud *out) {
    if (!ekf || !out) return;
    if (g_synthetic_scan_cmd_fd < 0 || g_synthetic_scan_res_fd < 0) return;

    init_point_cloud(out);

    ScanRequest req = {
        .theta = scan_theta,
        .origin = { 
            ekf->state.origin.x,
            SENSOR_HEIGHT,
            ekf->state.origin.z 
        },
        .min_elev = -30.0f * MATH_DEG_TO_RAD,
        .max_elev = 89.0f * MATH_DEG_TO_RAD,
        .num_rings = NUM_RINGS
    };

    if (write_all(g_synthetic_scan_cmd_fd, &req, sizeof(ScanRequest)) <= 0) {
        return;
    }

    RayResultBatch batch;
    for (int i = 0; i < NUM_WORKERS; i++) {
        if (read_exact(g_synthetic_scan_res_fd, &batch, sizeof(RayResultBatch)) <= 0) {
            return;
        }
        for (int j = 0; j < batch.count; j++) {
            if (batch.rays[j].distance > 0.0f) {
                point_cloud_push_back(out, 
                                     batch.rays[j].hit, 
                                     batch.rays[j].distance, 
                                     batch.rays[j].intensity);
            }
        }
    }
}

// lines 2-3 of Table 3.3 in Probabilistic Robotics (Thrun et al.) EKF algorithm
// µ¯_t = g(u_t, µ_t−1)
void ekf_fusion_predict_from_odometry(KalmanFilter *ekf, const SensorState *odom_prediction) {

    float theta_prev = ekf->state.dir_angle;
    float dx_prev = odom_prediction->origin.x - ekf->state.origin.x;
    float dz_prev = odom_prediction->origin.z - ekf->state.origin.z;
    float delta_theta = wrap_angle(odom_prediction->dir_angle - theta_prev);

    // line 2, g(u_t, µ_t−1): apply odometry control input to state estimate

    // g: R^3 x R^3 → R^3
    // x_t = x_t-1 + dx
    ekf->state.origin.x += dx_prev;
    // z_t = z_t-1 + dz
    ekf->state.origin.z += dz_prev;
    // theta = theta_t-1 + delta_theta
    ekf->state.dir_angle = wrap_angle(theta_prev + delta_theta);

    float d = sqrtf(dx_prev*dx_prev + dz_prev*dz_prev);

    // Jacobian of G_t at the current state 
    float G_t[EKF_STATE_DIM][EKF_STATE_DIM] =  {
        {1, 0, -d * sinf(theta_prev)},
        {0, 1, d * cosf(theta_prev)},
        {0, 0, 1}
    };

    // line 3: Σ̄_t = G_t * Σ_t-1 * G_t^T + R_t
    float G_t_T[EKF_STATE_DIM][EKF_STATE_DIM];
    float G_t_times_Sigma[EKF_STATE_DIM][EKF_STATE_DIM];
    float G_t_times_Sigma_times_G_t_T[EKF_STATE_DIM][EKF_STATE_DIM];
    matrix_transpose(G_t, G_t_T);
    matrix_mult(G_t, ekf->Sigma, G_t_times_Sigma);
    matrix_mult(G_t_times_Sigma, G_t_T, G_t_times_Sigma_times_G_t_T);
    matrix_add(G_t_times_Sigma_times_G_t_T, ekf->R_t, ekf->Sigma);
}


// Table 3.3 (Thrun et al.), lines 4-6
void ekf_fusion_correct_step_synthetic(KalmanFilter *ekf,
                                       const PointCloud *live_cloud,
                                       float scan_theta) {
    // µ_t = rover's guess of its own position
    // g(u_t, µ_t−1): state transition function, applies odometry control input to previous state estimate to get predicted new state
    // Sigma = rover's uncertainty about its own position
    // z_t = measurement from scan matching
    // h(µ¯_t): measurement function, maps predicted state to measurement space, in our case is identity since scan matching gives us a direct pose measurement
    // so H_t = I (this is because we are using synthetic scan matching, will change)
    // 
    // I = identity matrix 
    //
    // µ¯_t or Sigma¯_t, bar indicates predicted

    // line 4: K_t = Σ̄_t * H_t^T * (H_t * Σ̄_t * H_t^T + Q_t)^{-1}
    //             = Σ̄_t * (Σ̄_t + Q_t)^{-1}   since H_t = I
    // K_t is the Kalman Gain, how much to trust measurement vs prediction

    float Sigma_bar_plus_Q_t[EKF_STATE_DIM][EKF_STATE_DIM];
    float Sigma_bar_plus_Q_t_inv[EKF_STATE_DIM][EKF_STATE_DIM];
    float K_t[EKF_STATE_DIM][EKF_STATE_DIM];
    matrix_add(ekf->Sigma, ekf->Q_t, Sigma_bar_plus_Q_t);
    matrix_inverse(Sigma_bar_plus_Q_t, Sigma_bar_plus_Q_t_inv);
    matrix_mult(ekf->Sigma, Sigma_bar_plus_Q_t_inv, K_t);
    

    // line 5a: z_t = measurement from scan matching
    PointCloud synthetic = {0};
    generate_synthetic_scan(ekf, scan_theta, &synthetic);
    if (synthetic.size == 0) {
        point_cloud_free(&synthetic);
        return;
    }
    ICPResult icp = run_icp(&synthetic, live_cloud, 20);
    point_cloud_free(&synthetic);
    if (!icp.converged || icp.error > 0.5f) {
        return;
    }

    float z_t[EKF_STATE_DIM] = {
        ekf->state.origin.x + icp.dx,
        ekf->state.origin.z + icp.dz,
        wrap_angle(ekf->state.dir_angle + icp.dtheta)
    };

    // line 5: µt = µ¯t + Kt(zt − h(µ¯t))
    // since H_t = I then h(µ̄_t) = µ̄_t

    // z_minus_h(µ¯_t) = z_t - µ¯_t
    float z_minus_h_mu_bar[EKF_STATE_DIM] = {
        z_t[0] - ekf->state.origin.x,
        z_t[1] - ekf->state.origin.z,
        wrap_angle(z_t[2] - ekf->state.dir_angle)
    };
    ekf->state.origin.x  += K_t[0][0]*z_minus_h_mu_bar[0] + K_t[0][1]*z_minus_h_mu_bar[1] + K_t[0][2]*z_minus_h_mu_bar[2];
    ekf->state.origin.z  += K_t[1][0]*z_minus_h_mu_bar[0] + K_t[1][1]*z_minus_h_mu_bar[1] + K_t[1][2]*z_minus_h_mu_bar[2];
    ekf->state.dir_angle  = wrap_angle(
        ekf->state.dir_angle + K_t[2][0]*z_minus_h_mu_bar[0] + K_t[2][1]*z_minus_h_mu_bar[1] + K_t[2][2]*z_minus_h_mu_bar[2]
    );

    // line 6: Σ_t = (I - K_t * H_t) * Σ̄_t
    //             = (I - K_t) * Σ̄_t   since H_t = I
    float I_minus_K_t[EKF_STATE_DIM][EKF_STATE_DIM];
    for (int r = 0; r < EKF_STATE_DIM; r++) {
        for (int c = 0; c < EKF_STATE_DIM; c++) {
            I_minus_K_t[r][c] = (r == c ? 1.0f : 0.0f) - K_t[r][c];
        }
    }
    float Sigma_t[EKF_STATE_DIM][EKF_STATE_DIM];
    matrix_mult(I_minus_K_t, ekf->Sigma, Sigma_t);
    memcpy(ekf->Sigma, Sigma_t, sizeof(ekf->Sigma));
}


/*
void ekf_fusion_correct_step(KalmanFilter *ekf, const PointCloud *cloud, float scan_theta) {
    
    // TODO: this is tough. requires localization like SLAM and I don't have time to write out a 
    // full SLAM implementation here
    (void) ekf;
    (void) cloud;
    (void) scan_theta;
}
*/

const SensorState *ekf_fusion_get_state(const KalmanFilter *ekf) {
    return &ekf->state;
}