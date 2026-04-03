#ifndef EKF_FUSION_H
#define EKF_FUSION_H

#include "lidar/sensor_control.h"
#include "scene/point_cloud.h"

#define EKF_STATE_DIM 3
#define EKF_MEAS_DIM 3

typedef struct {
    SensorState state;
    float P[EKF_STATE_DIM][EKF_STATE_DIM]; // covariance matrix of state estimate
    float Q[EKF_STATE_DIM][EKF_STATE_DIM]; // covariance matrix of process noise
    float R[EKF_MEAS_DIM][EKF_MEAS_DIM]; // covariance matrix of measurement noise
} KalmanFilter;

void ekf_fusion_init(KalmanFilter *ekf, const SensorState *initial_state);
void ekf_fusion_predict_from_odometry(KalmanFilter *ekf, const SensorState *odom_prediction);
void ekf_fusion_correct_step(KalmanFilter *ekf, const PointCloud *cloud, float scan_theta);
const SensorState *ekf_fusion_get_state(const KalmanFilter *ekf);

#endif // EKF_FUSION_H