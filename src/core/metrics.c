#include "core/metrics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

// buffering config
#define METRICS_BUFSIZE (64 * 1024) // 64KB buffer for each log file
#define ICP_ITER_LOG_FREQ 1 // log every N iters

// logging
#define METRICS_RUN_ID 1
#define STR(x) #x
#define XSTR(x) STR(x)

static FILE *icp_match_log;
static FILE *icp_iters_log;
static FILE *ekf_update_log;
static FILE *mppi_log;
static FILE *rover_ground_truth_log;

// per-file mutexes
static pthread_mutex_t icp_match_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t icp_iters_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ekf_update_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mppi_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t rover_gt_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t metrics_init_lock = PTHREAD_MUTEX_INITIALIZER;

static uint64_t next_id_icp_match = 1;
static uint64_t next_id_ekf_update = 1;
static uint64_t next_id_mppi_step = 1;

static int metrics_initialized = 0; // guard to prev double initialisation

void metrics_init(void) {
    // fprintf(stderr, "metrics_init called from pid %d\n", getpid());
    pthread_mutex_lock(&metrics_init_lock);
    if (metrics_initialized) {
        pthread_mutex_unlock(&metrics_init_lock);
        return;
    }

    pthread_mutex_lock(&icp_match_lock);
    icp_match_log = fopen("logs/icp_matches_" XSTR(METRICS_RUN_ID) ".csv", "w");
    setvbuf(icp_match_log, NULL, _IOFBF, METRICS_BUFSIZE);
    fprintf(icp_match_log, "match_id,ts_microsecs,initial_residual,final_residual,iters,rejection_rate,accepted\n");
    pthread_mutex_unlock(&icp_match_lock);

    pthread_mutex_lock(&icp_iters_lock);
    icp_iters_log = fopen("logs/icp_iters_" XSTR(METRICS_RUN_ID) ".csv", "w");
    setvbuf(icp_iters_log, NULL, _IOFBF, METRICS_BUFSIZE);
    fprintf(icp_iters_log, "match_id,iter,residual,match_count\n");
    pthread_mutex_unlock(&icp_iters_lock);

    pthread_mutex_lock(&ekf_update_lock);
    ekf_update_log = fopen("logs/ekf_updates_" XSTR(METRICS_RUN_ID) ".csv", "w");
    setvbuf(ekf_update_log, NULL, _IOFBF, METRICS_BUFSIZE);
    fprintf(ekf_update_log, "update_id,ts_microsecs,match_id,innov_x,innov_y,innov_h,nis,P_xx,P_yy,P_hh,est_x,est_y,est_h,true_x,true_y,true_h\n");
    pthread_mutex_unlock(&ekf_update_lock);

    pthread_mutex_lock(&mppi_lock);
    mppi_log = fopen("logs/mppi_steps_" XSTR(METRICS_RUN_ID) ".csv", "w");
    setvbuf(mppi_log, NULL, _IOFBF, METRICS_BUFSIZE);
    fprintf(mppi_log, "step_id,ts_microsecs,cost_mean,cost_variance,cost_min,collision_rate,ess,cross_track_error,compute_time_microsecs,waypoint_id\n");
    pthread_mutex_unlock(&mppi_lock);

    pthread_mutex_lock(&rover_gt_lock);
    rover_ground_truth_log = fopen("logs/rover_ground_truth_" XSTR(METRICS_RUN_ID) ".csv", "w");
    setvbuf(rover_ground_truth_log, NULL, _IOFBF, METRICS_BUFSIZE);
    fprintf(rover_ground_truth_log, "ts_microsecs,true_x,true_y,true_heading,est_x,est_y,est_heading,error_x,error_y,error_heading\n");
    pthread_mutex_unlock(&rover_gt_lock);

    metrics_initialized = 1;
    pthread_mutex_unlock(&metrics_init_lock);
    fflush(icp_match_log);
    fflush(icp_iters_log);
    fflush(ekf_update_log);
    fflush(mppi_log);
    fflush(rover_ground_truth_log);

}

void metrics_close(void) {
    pthread_mutex_lock(&metrics_init_lock);
    if (!metrics_initialized) {
        pthread_mutex_unlock(&metrics_init_lock);
        return;
    }

    pthread_mutex_lock(&icp_match_lock);
    if (icp_match_log) { fflush(icp_match_log); fclose(icp_match_log); }
    pthread_mutex_unlock(&icp_match_lock);

    pthread_mutex_lock(&icp_iters_lock);
    if (icp_iters_log) { fflush(icp_iters_log); fclose(icp_iters_log); }
    pthread_mutex_unlock(&icp_iters_lock);

    pthread_mutex_lock(&ekf_update_lock);
    if (ekf_update_log) { fflush(ekf_update_log); fclose(ekf_update_log); }
    pthread_mutex_unlock(&ekf_update_lock);

    pthread_mutex_lock(&mppi_lock);
    if (mppi_log) { fflush(mppi_log); fclose(mppi_log); }
    pthread_mutex_unlock(&mppi_lock);

    pthread_mutex_lock(&rover_gt_lock);
    if (rover_ground_truth_log) { fflush(rover_ground_truth_log); fclose(rover_ground_truth_log); }
    pthread_mutex_unlock(&rover_gt_lock);

    metrics_initialized = 0;
    pthread_mutex_unlock(&metrics_init_lock);
}

uint64_t time_now_microsecs(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
}

uint64_t next_icp_match_id(void) {
    return (uint64_t)__sync_fetch_and_add(&next_id_icp_match, 1);
}

uint64_t next_ekf_update_id(void) {
    return (uint64_t)__sync_fetch_and_add(&next_id_ekf_update, 1);
}

uint64_t next_mppi_step_id(void) {
    return (uint64_t)__sync_fetch_and_add(&next_id_mppi_step, 1);
}


void log_icp_iteration(uint64_t match_id, int iter_no, float residual, int match_count) {
    if (iter_no % ICP_ITER_LOG_FREQ != 0) return;
    pthread_mutex_lock(&icp_iters_lock);
    fprintf(icp_iters_log, "%llu,%d,%.6f,%d\n", 
        match_id,
        iter_no,
        residual,
        match_count
    );
    pthread_mutex_unlock(&icp_iters_lock);
}


void log_icp_match(uint64_t match_id,
                           uint64_t ts_microsecs,
                           float initial_residual,
                           float final_residual,
                           int convergence_iters,
                           float rejection_rate,
                           int accepted) {
    pthread_mutex_lock(&icp_match_lock);
    fprintf(icp_match_log, "%llu,%llu,%.6f,%.6f,%d,%.6f,%d\n",
        match_id,
        ts_microsecs,
        initial_residual,
        final_residual,
        convergence_iters,
        rejection_rate,
        accepted
    );
    pthread_mutex_unlock(&icp_match_lock);
}


void log_ekf_update(uint64_t update_id,
                            uint64_t ts_microsecs,
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
                            float true_h) {
    pthread_mutex_lock(&ekf_update_lock);
    fprintf(ekf_update_log,
            "%llu,%llu,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            update_id,
            ts_microsecs,
            match_id,
            innov_x, innov_y, innov_h,
            nis,
            P_xx, P_yy, P_hh,
            est_x, est_y, est_h,
            true_x, true_y, true_h);
    pthread_mutex_unlock(&ekf_update_lock);
}


void log_mppi_step(uint64_t step_id,
                           uint64_t ts_microsecs,
                           float cost_mean,
                           float cost_variance,
                           float cost_min,
                           float collision_rate,
                           float ess,
                           float cross_track_error,
                           uint64_t compute_time_microsecs,
                           int waypoint_id) {
    pthread_mutex_lock(&mppi_lock);
    fprintf(mppi_log, "%llu,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%llu,%d\n",
        step_id,
        ts_microsecs,
        cost_mean,
        cost_variance,
        cost_min,
        collision_rate,
        ess,
        cross_track_error,
        compute_time_microsecs,
        waypoint_id
    );
    pthread_mutex_unlock(&mppi_lock);
}


void log_rover_ground_truth(uint64_t ts_microsecs,
                                    const SensorState *true_state,
                                    const SensorState *estimate_state) {
    float err_x = estimate_state->origin.x - true_state->origin.x;
    float err_y = estimate_state->origin.z - true_state->origin.z;
    float err_h = estimate_state->dir_angle - true_state->dir_angle;
    pthread_mutex_lock(&rover_gt_lock);
    fprintf(rover_ground_truth_log, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        ts_microsecs,
        true_state->origin.x,
        true_state->origin.z,
        true_state->dir_angle,
        estimate_state->origin.x,
        estimate_state->origin.z,
        estimate_state->dir_angle,
        err_x, err_y, err_h
    );
    pthread_mutex_unlock(&rover_gt_lock);
}

