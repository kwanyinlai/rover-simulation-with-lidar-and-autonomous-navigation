#ifndef METRICS_H
#define METRICS_H

#include <stdint.h>

#include "lidar/sensor_control.h"


void metrics_init(void);
void metrics_close(void);


uint64_t time_now_microsecs(void);
uint64_t next_icp_match_id(void);
uint64_t next_ekf_update_id(void);
uint64_t next_mppi_step_id(void);

/**
 * @brief Log a single ICP solver iteration.
 * @param match_id Unique ID for the ICP match
 * @param iteration Iteration number of the ICP solver (zero-indexed)
 * @param residual Alignment residual for this iteration
 * @param match_count Number of point matches considered this iteration
 *
 */
void log_icp_iteration(uint64_t match_id,
							   int iteration,
							   float residual,
							   int match_count);

/**
 * @brief Log overall ICP match result and quality metrics.
 * @param match_id Unique ID for the ICP match
 * @param timestamp_us Timestamp (microseconds) when the match completed
 * @param initial_residual Residual value before ICP
 * @param final_residual Residual value after ICP
 * @param iterations Number of ICP iterations performed until convergence
 * @param rejection_rate Fraction of point correspondences rejected by the matcher
 * @param accepted Boolean flag indicating whether the match result was accepted for use
 *
 */
void log_icp_match(uint64_t match_id,
						   uint64_t timestamp_us,
						   float initial_residual,
						   float final_residual,
						   int iterations,
						   float rejection_rate,
						   int accepted);

/**
 * @brief Log EKF update diagnostics
 * @param update_id Unique ID for this EKF update
 * @param timestamp_us Timestamp (microseconds)
 * @param match_id Associated ICP match id that produced the observation
 * @param innov_x Measurement innovation in x
 * @param innov_y Measurement innovation in y
 * @param innov_h Measurement innovation in heading
 * @param nis Normalized Innovation Squared
 * @param P_xx Diagonal posterior covariance entry for x (variance)
 * @param P_yy Diagonal posterior covariance entry for y (variance)
 * @param P_hh Diagonal posterior covariance entry for heading (variance)
 * @param est_x State estimate x
 * @param est_y State estimate y
 * @param est_h State estimate heading
 * @param true_x Ground-truth x
 * @param true_y Ground-truth y
 * @param true_h Ground-truth heading
 *
 */
void log_ekf_update(uint64_t update_id,
							uint64_t timestamp_us,
							uint64_t match_id,
							float innov_x,
							float innov_y,
							float innov_h,
							float nis,
							float P_xx,
							float P_yy,
							float P_hh,
							float est_x,
							float est_y,
							float est_h,
							float true_x,
							float true_y,
							float true_h);

/**
 * @brief Log MPPI step metrics and sample statistics
 * @param step_id Unique ID for this MPPI sample
 * @param timestamp_us Timestamp 
 * @param cost_mean Mean cost across the sampled trajectories
 * @param cost_variance Variance of the costs across samples
 * @param cost_min Minimum cost observed among samples
 * @param ess Effective sample size (ESS = 1 / sum(weight_k^2)
 * @param cross_track_error Lateral deviation from planned path
 *
 */
void log_mppi_step(uint64_t step_id,
				   uint64_t timestamp_us,
				   float cost_mean,
				   float cost_variance,
				   float cost_min,
				   float ess,
				   float cross_track_error);

/**
 * @brief Log ground-truth and estimator states and per-axis errors (speed disrrregarded from 
 * sensor state).
 * @param timestamp_us Timestamp
 * @param true_state Pointer to the ground-truth sensor state
 * @param estimate_state Pointer to the estimator's sensor state
 *
 * Logged columns: timestamp, true_x, true_y, true_heading, est_x, est_y, est_heading,
 *                error_x, error_y, error_heading where errors = estimator - ground-truth.
 */
void log_rover_ground_truth(uint64_t timestamp_us,
							const SensorState *true_state,
							const SensorState *estimate_state);

#endif // METRICS_H
