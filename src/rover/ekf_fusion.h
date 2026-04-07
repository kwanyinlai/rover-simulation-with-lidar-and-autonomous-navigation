#ifndef EKF_FUSION_H
#define EKF_FUSION_H

#include "lidar/sensor_control.h"
#include "scene/point_cloud.h"

#define EKF_STATE_DIM 3
#define EKF_MEAS_DIM 3


typedef struct {
    SensorState state;
    float Sigma[EKF_STATE_DIM][EKF_STATE_DIM]; // covariance matrix of state estimate
    float Q_t[EKF_STATE_DIM][EKF_STATE_DIM]; // covariance matrix of measurement noise
    float R_t[EKF_MEAS_DIM][EKF_MEAS_DIM]; // covariance matrix of process noise
} KalmanFilter;

void set_scan_match_pipe_fds(int cmd_fd, int res_fd);
void ekf_fusion_init(KalmanFilter *ekf, const SensorState *initial_state);
void ekf_fusion_predict_from_odometry(KalmanFilter *ekf, const SensorState *odom_prediction);
void ekf_fusion_correct_step(KalmanFilter *ekf, const PointCloud *cloud, float scan_theta);
const SensorState *ekf_fusion_get_state(const KalmanFilter *ekf);

#endif // EKF_FUSION_H