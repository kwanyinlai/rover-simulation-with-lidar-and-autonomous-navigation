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

/**
 * Correct step using an ICP result anchored to a keyframe pose.
 *
 * ICP gives a displacement (dx, dz, dtheta) from the keyframe scan origin.
 * This function reconstructs the absolute world-frame measurement
 *   z = keyframe_pose + icp_delta
 * and uses it to pull the EKF estimate toward the true pose.
 *
 * Must only be called when icp->converged is true and icp->error is
 * below the acceptance threshold; ekf_fusion_correct_from_icp enforces
 * this internally but callers should gate on it too for clarity.
 *
 * @param ekf           Pointer to KalmanFilter to update.
 * @param icp           ICP result relative to the keyframe.
 * @param kf_x          World-frame x position when the keyframe was taken.
 * @param kf_z          World-frame z position when the keyframe was taken.
 * @param kf_heading    World-frame heading when the keyframe was taken.
 */
void ekf_fusion_correct_from_icp(KalmanFilter *ekf,
                                  const ICPResult *icp,
                                  float kf_x,
                                  float kf_z,
                                  float kf_heading);

/**
 * Return a pointer to the current EKF state estimate
 */
const SensorState *ekf_fusion_get_state(const KalmanFilter *ekf);

#endif // EKF_FUSION_H