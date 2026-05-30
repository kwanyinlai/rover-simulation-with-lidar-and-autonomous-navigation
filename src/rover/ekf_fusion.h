#ifndef EKF_FUSION_H
#define EKF_FUSION_H

#include "lidar/sensor_control.h"
#include "localization/scan_matcher.h"
#include "scene/point_cloud.h"

#define EKF_STATE_DIM 3
#define EKF_MEAS_DIM 3

typedef struct {
    SensorState state;
    float Sigma[EKF_STATE_DIM][EKF_STATE_DIM]; // covariance matrix of state estimate [x, z, theta]
    float Q_t[EKF_STATE_DIM][EKF_STATE_DIM]; // covariance matrix of measurement noise, how much to trust scan matching measurement vs prediction
    float R_t[EKF_MEAS_DIM][EKF_MEAS_DIM]; // covariance matrix of process noise, how much uncertainty odometry adds per step
} KalmanFilter;

/* 
void set_scan_match_pipe_fds(int cmd_fd, int res_fd);
*/

/**
 * Initialize the EKF with a starting pose and default noise parameters.
 *
 * @param ekf               Pointer to Kalman Filter to initialize
 * @param initial_state     Starting pose estimate
 */
void ekf_fusion_init(KalmanFilter *ekf, const SensorState *initial_state);

/**
 * Predict step. Advances the pose estimate using dead-reckoning data and
 * accumulates uncertainty
 *
 * @param ekf       Pointer to Kalman Filter to update
 * @param dx        dx in world-space for this frame
 * @param dz        dz in world-space for this frame
 * @param dtheta    delta theta for this frame, wrapped to [0, 2pi)
 */
void ekf_fusion_predict_from_odometry(KalmanFilter *ekf, float dx, float dz, float dtheta);

// legacy EKF correction performed on main thread
// void ekf_fusion_correct_step(KalmanFilter *ekf,
//                              const PointCloud *current_scan,
//                              const PointCloud *reference_scan);


/**
 * Correct step. Pulls the pose estimate toward an ICP scan-match result and
 * shrinks uncertainty. How much the estimate moves depends on the Kalman Gain,
 * which balances current uncertainty (Sigma) against ICP noise (Q_t).
 *
 * Should be called ONLY when ICP has converged with acceptable error to prevent
 * bad scan matches from corrupting EKF state.
 *
 * @param ekf Pointer to Kalman Filter to update.
 * @param icp Results from ICP scan matching, used as measurement to correct EKF state.
 */
void ekf_fusion_correct_from_icp(KalmanFilter *ekf, const ICPResult *icp);

const SensorState *ekf_fusion_get_state(const KalmanFilter *ekf);

#endif // EKF_FUSION_H