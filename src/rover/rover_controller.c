#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "rover_controller.h"
#include "lidar/lidar_sensor.h"
#include "lidar/sensor_control.h"
#include "scene/scene_collision.h"
#include "scene/scene_state.h"
#include "core/physics_constants.h"
#include "core/noise.h"
#include "core/math_utils.h"
#include "core/io_utils.h"
#include "localization/scan_matcher.h"
#include "rover/ekf_fusion.h"
#include "rover/rover_physics.h"
#include "core/metrics.h"

#include <math.h>
#include <stdio.h>
#include <float.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include "piping/messages.h"

// https://acdelta_steerlab.github.io/mppi-generic-website/docs/mppi.html
// https://dilithjay.com/blog/mppi 
// MPPI Controller -- Model Predictive Path Integral

// MPPI hyperparameters

// defined partially in rover_controller.h
/*
#define MPPI_SAMPLES 32 // parallel rollouts per frame
#define MPPI_HORIZON 24 // steps per rollout
*/

#define MPPI_DT 0.05f // rollout timestep
#define MPPI_LAMBDA 2.5f // temperature (0 = greedy, inf = uniform random)

// control exploration noise, more noise = more exploration
#define MPPI_SIGMA_STEER 0.15f
#define MPPI_SIGMA_THROTTLE 0.12f

#define ICP_WORKER_MAX_ITERS 20

// reference / saturation limits
#define SPEED_REF (MAX_SPEED * 0.55f) // reference speed
#define STEER_MIN -1.0f
#define STEER_MAX 1.0f
#define THROTTLE_MIN 0.0f
#define THROTTLE_MAX 1.0f

static inline float clamp_steer(float steer) {
    return fmaxf(STEER_MIN, fminf(STEER_MAX, steer));
}

static inline float clamp_throttle(float throttle) {
    return fmaxf(THROTTLE_MIN, fminf(THROTTLE_MAX, throttle));
}

// for rendering predictions
#define PREDICTION_STEPS MPPI_HORIZON
#define PREDICTION_DT MPPI_DT

// for logging
#define POSE_DIFF_LOG_INTERVAL (2.0f)

// ====================================================================

SensorState rover_pose = {0};
Path active_path = {0};
RoverMode rover_mode = MODE_AUTO;

static int require_replan_write_fd = -1;
static int replan_request_sent = 0;
static int awaiting_replan_scan = 0;
static float replan_scan_start_theta = 0.0f;
static float replan_scan_last_theta = 0.0f;
static float replan_scan_accumulated = 0.0f;
static const float REPLAN_SCAN_TURN_RAD = 4 * M_PI;
static int rollout_cmd_write_fd = -1;
static int rollout_result_read_fd = -1;
// EKF integration is temporarily disabled.
static KalmanFilter ekf_pose;
static SensorState odom_pose = {0};
static float g_pose_diff_log_accum = 0.0f;
static SensorState prev_odom_pose = {0};
static int rollout_has_collided[MPPI_SAMPLES];


typedef struct {
    PointCloud current;
    PointCloud reference;
    int is_working;
} ICPJob;

typedef struct {
    ICPResult result;
    int is_job_complete;
} ICPResultState;

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    ICPJob job;
    ICPResultState result;
} ICPContext;

static ICPContext icp_res = {
    /* mutex = */ PTHREAD_MUTEX_INITIALIZER,
    /* cond = */ PTHREAD_COND_INITIALIZER,
    /* ICPJob job = */ {0},
    /* ICPResultState result = */ {0}
};


static void *icp_worker_main(void *arg) {
    ICPContext *context = (ICPContext *)arg;
    while (1) {
        pthread_mutex_lock(&context->mutex);
        while (!context->job.is_working) {
            pthread_cond_wait(&context->cond, &context->mutex);
        }
        ICPJob job = context->job;
        context->job.is_working = 0;
        pthread_mutex_unlock(&context->mutex);

        ICPResult result = run_icp(&job.current, &job.reference, ICP_WORKER_MAX_ITERS);

        point_cloud_free(&job.current);
        point_cloud_free(&job.reference);

        pthread_mutex_lock(&context->mutex);
        context->result.result = result;
        context->result.is_job_complete = 1;
        pthread_mutex_unlock(&context->mutex);
    }
    return NULL; // nothing to return
}


// nominal steer and throttle, warm started with previous frame'sim_state optimal
static float nom_steer[MPPI_HORIZON];
static float nom_throttle[MPPI_HORIZON];

// gaussian noise per step per rollout
static float steer_noise[MPPI_SAMPLES][MPPI_HORIZON];
static float throttle_noise[MPPI_SAMPLES][MPPI_HORIZON];
static float trajectory_cost[MPPI_SAMPLES];


// rollout cost weights
#define W_CROSS_TRACK 3.5f // penalise distancing from path centre
#define W_HEADING 1.2f // penalise heading vs path tangent
#define W_STEER_RATE 2.0f // penalise changes in steering
#define W_SPEED 0.3f // penalise changes from desired speed
#define W_THROTTLE_EFFORT 0.15f // penalise throttle usage
#define W_TERMINAL_CROSS_TRACK 6.0f // terminal cost to heavily penalise ending far from path centre
#define W_TERMINAL_HEADING 2.0f // terminal cost to penalise ending with heading error
#define W_COLLISION 25.0f // penalty when movement is blocked by collision


// waypoints
#define WAYPOINT_REACH_DIST 1.0f
#define WAYPOINT_REACH_THRESHOLD (WAYPOINT_REACH_DIST * WAYPOINT_REACH_DIST)

static void advance_waypoint_by_proximity(const Path *path,
                                          int *wp_idx,
                                          float rover_x,
                                          float rover_z,
                                          int max_idx) {
                                            
    if (!path || !wp_idx || path->count <= 0) {
        return;
    }

    int upper_bound = max_idx;
    if (upper_bound > path->count) {
        upper_bound = path->count;
    }

    while (*wp_idx < upper_bound) {
        const Waypoint *waypoint = &path->waypoints[*wp_idx];
        float dx = waypoint->x - rover_x;
        float dz = waypoint->z - rover_z;
        if (dx * dx + dz * dz < WAYPOINT_REACH_THRESHOLD) {
            (*wp_idx)++;
        } else {
            break;
        }
    }
}

static void advance_waypoint_to_farthest_reached(const Path *path,
                                                 int *wp_idx,
                                                 float rover_x,
                                                 float rover_z,
                                                 int max_idx) {

    int next_target = *wp_idx;
    for (int i = *wp_idx; i < max_idx; i++) {
        const Waypoint *waypoint = &path->waypoints[i];
        float dx = waypoint->x - rover_x;
        float dz = waypoint->z - rover_z;
        if (dx * dx + dz * dz < WAYPOINT_REACH_THRESHOLD) {
            next_target = i + 1;
        }
    }

    *wp_idx = next_target;
}

// fixed constants relative to MAX_WAYPOINTS, for sanity
#define LOOK_BACK 2
#define LOOK_AHEAD 6
// Returns cross-track error, and outputs nearest path tangent angle and index of nearest line segment in path
static float project_rover_to_path_segment(const Path *path,
                                           float rover_x,
                                           float rover_z,
                                           int waypoint_hint,
                                           float *dir_angle_out, 
                                           int *nearest_line_seg_out) {
    // waypoint hint to maintain path continuity
    float best_sqr_dist = FLT_MAX;
    float cross_track_error = 0.0f;
    *dir_angle_out = 0.0f;
    *nearest_line_seg_out = waypoint_hint;

    int start_segment = waypoint_hint - LOOK_BACK > 0 ? waypoint_hint - LOOK_BACK : 0;
    int end_segment = waypoint_hint + LOOK_AHEAD >= path->count ? path->count - 1 : waypoint_hint + LOOK_AHEAD;

    for (int i = start_segment; i < end_segment; i++) {
        float wp1_x = path->waypoints[i].x;
        float wp1_z = path->waypoints[i].z;
        float wp2_x = path->waypoints[i + 1].x;
        float wp2_z = path->waypoints[i + 1].z;

        float dx = wp2_x - wp1_x;
        float dz = wp2_z - wp1_z;
        float len_sqr = dx * dx + dz * dz;

        if (len_sqr < 1e-6f) {
            continue;
        }

        // project onto line segment
        float t = ((rover_x - wp1_x) * dx + (rover_z - wp1_z) * dz) / len_sqr;
        t = fmaxf(0.0f, fminf(1.0f, t));

        float proj_x = wp1_x + t * dx;
        float proj_z = wp1_z + t * dz;
        float x_offset = rover_x - proj_x;
        float z_offset = rover_z - proj_z;
        float error_sqr = x_offset * x_offset + z_offset * z_offset;

        if (error_sqr < best_sqr_dist) {
            best_sqr_dist = error_sqr;
            *nearest_line_seg_out = i;
            *dir_angle_out = atan2f(dz, dx);

            // cross product, -ve if rover is right of path, +ve if left
            // magnitude = distance to path
            float cross = dx * z_offset - dz * x_offset;
            cross_track_error = (cross >= 0.0f ? 1.0f : -1.0f) * sqrtf(error_sqr);
        }
    }
    return cross_track_error;
}


static void rollout_simulation(SimState *sim_state, float throttle, float steer, float dt) {
    step_rover_physics(&sim_state->x,
                       &sim_state->z,
                       &sim_state->dir_angle,
                       &sim_state->speed,
                       &sim_state->angular_speed,
                       throttle,
                       steer,
                       dt,
                       &scene,
                       ROVER_COLLISION_RADIUS);

}


static float rollout_cost(int i)
{
    int collided = 0;
    SimState init_state = {
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = odom_pose.speed,
        .angular_speed = odom_pose.angular_speed,
        .wp_idx = active_path.current
    };

    float cost = mppi_compute_rollout_cost(&scene,
                              &active_path,
                              init_state,
                              MPPI_HORIZON,
                              nom_steer,
                              nom_throttle,
                              steer_noise[i],
                              throttle_noise[i],
                              &collided);
    rollout_has_collided[i] = collided;
    return cost;
                              
}

float mppi_compute_rollout_cost(const TriangleArray *scene,
                                const Path *path,
                                SimState init_state,
                                int horizon,
                                const float *nom_steer_seq,
                                const float *nom_throttle_seq,
                                const float *steer_noise_seq,
                                const float *throttle_noise_seq,
                                int *out_collision)
{
    SimState sim_state = init_state;
    float cost = 0.0f;
    float prev_steer = nom_steer_seq[0];
    int collided = 0;

    for (int j = 0; j < horizon; j++) {
        float steer_cmd = clamp_steer(nom_steer_seq[j] + steer_noise_seq[j]);
        float throttle_cmd = clamp_throttle(nom_throttle_seq[j] + throttle_noise_seq[j]);

        advance_waypoint_by_proximity(path,
                                      &sim_state.wp_idx,
                                      sim_state.x,
                                      sim_state.z,
                                      path->count - 1);

        float path_heading;
        int nearest_segment;
        float cross_track_error = project_rover_to_path_segment(path,
                                                                sim_state.x,
                                                                sim_state.z,
                                                                sim_state.wp_idx,
                                                                &path_heading,
                                                                &nearest_segment);
        float heading_error = wrap_angle(sim_state.dir_angle - path_heading);

        float progress_reward = (float)(nearest_segment - sim_state.wp_idx);
        // allow skipping within sim state not just relative to path
        if (nearest_segment > sim_state.wp_idx) {
            sim_state.wp_idx = nearest_segment;
        }

        cost += W_CROSS_TRACK * (cross_track_error * cross_track_error)
              + W_HEADING * (heading_error * heading_error)
              + W_STEER_RATE * ((steer_cmd - prev_steer) * (steer_cmd - prev_steer))
              + W_THROTTLE_EFFORT * (throttle_cmd * throttle_cmd)
              + W_SPEED * ((sim_state.speed - SPEED_REF) * (sim_state.speed - SPEED_REF))
              - progress_reward;

        prev_steer = steer_cmd;

        float prev_x = sim_state.x;
        float prev_z = sim_state.z;
        step_rover_physics(&sim_state.x,
                           &sim_state.z,
                           &sim_state.dir_angle,
                           &sim_state.speed,
                           &sim_state.angular_speed,
                           throttle_cmd,
                           steer_cmd,
                           MPPI_DT,
                           scene,
                           ROVER_COLLISION_RADIUS);

        if (fabsf(sim_state.x - prev_x) + fabsf(sim_state.z - prev_z) < 1e-6f &&
            fabsf(throttle_cmd) > 0.05f) { // control with no movement
            cost += W_COLLISION; // heuristic weight penalisation instead of heavy computation
            collided = 1;
        }
    }

    float path_heading;
    int nearest_segment;
    float terminal_cross_track_error = project_rover_to_path_segment(path,
                                                                     sim_state.x,
                                                                     sim_state.z,
                                                                     sim_state.wp_idx,
                                                                     &path_heading,
                                                                     &nearest_segment);
    float terminal_heading_error = wrap_angle(sim_state.dir_angle - path_heading);

    cost += W_TERMINAL_CROSS_TRACK * (terminal_cross_track_error * terminal_cross_track_error)
          + W_TERMINAL_HEADING * (terminal_heading_error * terminal_heading_error);

    if (out_collision) {
        *out_collision = collided;
    }

    return cost;
}


void set_rollout_pipe_fds(int cmd_write_fd, int result_read_fd)
{
    rollout_cmd_write_fd = cmd_write_fd;
    rollout_result_read_fd = result_read_fd;
}

void set_replan_pipe_fd(int write_fd)
{
    require_replan_write_fd = write_fd;
}

void force_replan_request(void)
{
    active_path.current = active_path.count;
    replan_request_sent = 0;
    awaiting_replan_scan = 0;
    replan_scan_accumulated = 0.0f;
}

static int evaluate_rollouts_via_pipe(void)
{
    if (rollout_cmd_write_fd < 0 || rollout_result_read_fd < 0) {
        return 0;
    }

    RolloutRequest request;
    memset(&request, 0, sizeof(request));
    request.sample_count = MPPI_SAMPLES;
    request.horizon = MPPI_HORIZON;
    request.init_state = (SimState){
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = odom_pose.speed,
        .angular_speed = odom_pose.angular_speed,
        .wp_idx = active_path.current
    };
    request.path_snapshot = active_path;
    memcpy(request.nom_steer, nom_steer, sizeof(nom_steer));
    memcpy(request.nom_throttle, nom_throttle, sizeof(nom_throttle));
    memcpy(request.steer_noise, steer_noise, sizeof(steer_noise));
    memcpy(request.throttle_noise, throttle_noise, sizeof(throttle_noise));

    if (write_all(rollout_cmd_write_fd, &request, sizeof(RolloutRequest)) < 0) {
        fprintf(stderr, "rollout request write failed, falling back to synchronous MPPI this frame\n");
        return 0;
    }

    BatchedRolloutResult result;
    int result_read = read_exact(rollout_result_read_fd, &result, sizeof(BatchedRolloutResult));
    if (result_read <= 0) {
        if (result_read == 0) {
            fprintf(stderr, "rollout result pipe closed unexpectedly\n");
        }
        fprintf(stderr, "keeping previous nominal control sequence this frame\n");
        return -1;
    }

    memcpy(trajectory_cost, result.costs, sizeof(trajectory_cost));
    return 1;
}


static void mppi_update(void){
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        for (int j = 0; j < MPPI_HORIZON; j++) {
            steer_noise[i][j] = gaussian_noise() * MPPI_SIGMA_STEER;
            throttle_noise[i][j] = gaussian_noise() * MPPI_SIGMA_THROTTLE;
        }
    }

    float min_cost = FLT_MAX;

    int rollout_status = evaluate_rollouts_via_pipe();
    if (rollout_status < 0) {
        return;
    }

    // fallback do manually + for testing
    if (rollout_status == 0) {
        for (int i = 0; i < MPPI_SAMPLES; i++) {
            trajectory_cost[i] = rollout_cost(i);
        }
    }

    for (int i = 0; i < MPPI_SAMPLES; i++) {
        if (trajectory_cost[i] < min_cost) {
            min_cost = trajectory_cost[i];
        }
    }

    float weights[MPPI_SAMPLES];
    float weighted_sum = 0.0f;
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        weights[i] = expf(-(trajectory_cost[i] - min_cost) / MPPI_LAMBDA);
        weighted_sum += weights[i];
    }

    // weighted
    for (int j = 0; j < MPPI_HORIZON; j++) {
        float delta_steer = 0.0f;
        float delta_throttle = 0.0f;
        for (int i = 0; i < MPPI_SAMPLES; i++) {
            float weight = weights[i] / weighted_sum;
            delta_steer += weight * steer_noise[i][j];
            delta_throttle += weight * throttle_noise[i][j];
        }
        nom_steer[j] = clamp_steer(nom_steer[j] + delta_steer);
        nom_throttle[j] = clamp_throttle(nom_throttle[j] + delta_throttle);
    }
    uint64_t step_id = next_mppi_step_id();
    uint64_t ts = time_now_microsecs();
    float cost_mean = 0.0f;
    for (int i = 0; i < MPPI_SAMPLES; i++) cost_mean += trajectory_cost[i];
    cost_mean /= MPPI_SAMPLES;

    float cost_variance = 0.0f;
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        float d = trajectory_cost[i] - cost_mean;
        cost_variance += d * d;
    }
    cost_variance /= MPPI_SAMPLES;

    int collision_count = 0;
    for (int i = 0; i < MPPI_SAMPLES; i++) collision_count += rollout_has_collided[i];
    float collision_rate = (float)collision_count / MPPI_SAMPLES;

    float ess = 0.0f; // effective sample size = 1 / sum(w^2)
    float sum_w2 = 0.0f;
    for (int i = 0; i < MPPI_SAMPLES; i++) {
        float w = weights[i] / weighted_sum;
        sum_w2 += w * w;
    }
    ess = 1.0f / sum_w2;

    float path_heading;
    int nearest_seg;
    float cte = project_rover_to_path_segment(&active_path,
                                              rover_pose.origin.x,
                                              rover_pose.origin.z,
                                              active_path.current,
                                              &path_heading,
                                              &nearest_seg);

    log_mppi_step(step_id, ts, cost_mean, cost_variance, min_cost,
                  collision_rate, ess, cte, 0, active_path.current);

}



static void advance_mppi(void) { 
    for (int i = 0; i < MPPI_HORIZON - 1; i++) {
        nom_steer[i] = nom_steer[i + 1];
        nom_throttle[i] = nom_throttle[i + 1];
    }
    nom_steer[MPPI_HORIZON - 1] = nom_steer[MPPI_HORIZON - 2];
    nom_throttle[MPPI_HORIZON - 1] = nom_throttle[MPPI_HORIZON - 2];
}


// END MPPI CONTROLLER 

void init_rover_controller(void) {

    rover_pose.origin.x = 0.0f;
    rover_pose.origin.z = 0.0f;
    rover_pose.dir_angle = 0.0f;
    rover_pose.speed = 0.0f;
    rover_pose.angular_speed = 0.0f;
    odom_pose = rover_pose;
    prev_odom_pose = rover_pose;
    active_path.count = 0;
    active_path.current = 0;
    rover_mode = MODE_MANUAL;
    replan_request_sent = 0;

    memset(nom_steer, 0, sizeof(nom_steer));
    // seed throttle so the rover starts moving without waiting for convergence
    for (int i = 0; i < MPPI_HORIZON; i++){
        nom_throttle[i] = 0.35f;
    }

    ekf_fusion_init(&ekf_pose, &rover_pose);

    icp_res.job = (ICPJob){0};
    icp_res.result = (ICPResultState){0};
    pthread_t icp_thread;
    pthread_create(&icp_thread, NULL, icp_worker_main, &icp_res);
    pthread_detach(icp_thread);
}
void update_odometry(float dt) {
    g_pose_diff_log_accum += dt;
    if (g_pose_diff_log_accum >= POSE_DIFF_LOG_INTERVAL) {
        g_pose_diff_log_accum = 0.0f;
    }
    float throttle = get_throttle();
    float steer = get_steer();

    float throttle_cmd = throttle;
    if (throttle != 0.0f) {
        throttle_cmd += gaussian_noise() * SPEED_NOISE;
    }

    float steer_cmd = steer;
    if (steer != 0.0f) {
        steer_cmd += gaussian_noise() * ANGULAR_NOISE;
    }
    // odom_pose not touched by EKF, purely dead reckoning from last frame's pose + control input, used for odometry delta in EKF prediction step
    step_rover_physics(&odom_pose.origin.x,
                       &odom_pose.origin.z,
                       &odom_pose.dir_angle,
                       &odom_pose.speed,
                       &odom_pose.angular_speed,
                       throttle_cmd,
                       steer_cmd,
                       dt,
                       NULL,
                       ROVER_COLLISION_RADIUS);
    // ekf pred, advance uncertainty
    ekf_fusion_predict_from_odometry(&ekf_pose,
                                     odom_pose.origin.x - prev_odom_pose.origin.x,
                                     odom_pose.origin.z - prev_odom_pose.origin.z,
                                     odom_pose.dir_angle - prev_odom_pose.dir_angle);
    rover_pose = *ekf_fusion_get_state(&ekf_pose);

    prev_odom_pose = odom_pose;

    const SensorState *true_state = get_sensor_state();
    log_rover_ground_truth(time_now_microsecs(), true_state, &rover_pose);
}

void update_lidar_fusion(const PointCloud *current_scan,
                         const PointCloud *reference_scan) {

    PointCloud current_copy = {0};
    PointCloud reference_copy = {0};
    if (current_scan->size <= 0 || reference_scan->size <= 0) {
        return;
    }

    pthread_mutex_lock(&icp_res.mutex);
    int job_busy = icp_res.job.is_working;
    pthread_mutex_unlock(&icp_res.mutex);

    if (!job_busy) {

        if (point_cloud_copy(&current_copy, current_scan) &&
            point_cloud_copy(&reference_copy, reference_scan)) 
        {

            pthread_mutex_lock(&icp_res.mutex);
            icp_res.job.current = current_copy;
            icp_res.job.reference = reference_copy;
            icp_res.job.is_working = 1;
            pthread_cond_signal(&icp_res.cond);
            pthread_mutex_unlock(&icp_res.mutex);
        }
        else{
            point_cloud_free(&current_copy);
            point_cloud_free(&reference_copy);
        }
    }

    ICPResult icp = {0};
    int is_job_complete = 0;
    pthread_mutex_lock(&icp_res.mutex);
    if (icp_res.result.is_job_complete) {
        icp = icp_res.result.result;
        icp_res.result.is_job_complete = 0;
        is_job_complete = 1;
    }
    pthread_mutex_unlock(&icp_res.mutex);

    if (is_job_complete) {
        if (icp.converged) {
            ekf_fusion_correct_from_icp(&ekf_pose, &icp);
        }
    }
    rover_pose = *ekf_fusion_get_state(&ekf_pose);
}

void update_path_follower(float dt) {
    (void) dt; // dt is currently unused but might be needed in the future
    if (active_path.current >= active_path.count) {
        // wait for one full lidar revolution before requesting a new plan.
        if (!awaiting_replan_scan) {
            awaiting_replan_scan = 1;
            replan_scan_start_theta = get_scan_theta();
            replan_scan_last_theta = replan_scan_start_theta;
            replan_scan_accumulated = 0.0f;
        }

        float scan_theta = get_scan_theta();
        float scan_delta = wrap_angle(scan_theta - replan_scan_last_theta);
        replan_scan_accumulated += fabsf(scan_delta);
        replan_scan_last_theta = scan_theta;

        if (replan_scan_accumulated < REPLAN_SCAN_TURN_RAD) {
            set_throttle(0.0f);
            set_steer(0.0f);
            return;
        }

        if (!replan_request_sent && require_replan_write_fd >= 0) {
            if (write_all(require_replan_write_fd, &rover_pose, sizeof(SensorState)) < 0) {
                exit(1);
            }
            replan_request_sent = 1;
        }
        set_throttle(0.0f);
        set_steer(0.0f);
        return;
    }

    // skip waypoints based off projection
    float _;
    int nearest_segment;
    project_rover_to_path_segment(&active_path,
                    rover_pose.origin.x, rover_pose.origin.z,
                    active_path.current, &_, &nearest_segment);
    if (nearest_segment > active_path.current) {
        active_path.current = nearest_segment;
    }


    int prev_waypoint_idx = active_path.current;
    advance_waypoint_to_farthest_reached(&active_path,
                                         &active_path.current,
                                         rover_pose.origin.x,
                                         rover_pose.origin.z,
                                         active_path.count);
    if (active_path.current > prev_waypoint_idx) {
        /* waypoint reached - console print removed to avoid noisy stdout */
    }

    if (active_path.current >= active_path.count) {
        set_throttle(0.0f);
        set_steer(0.0f);
        return;
    }

    awaiting_replan_scan = 0;

    // run MPPI optimisation to get nominal control sequence
    mppi_update();

    float steer = nom_steer[0];
    float throttle = nom_throttle[0];

    const Waypoint *last = &active_path.waypoints[active_path.count - 1];
    float final_dx = last->x - rover_pose.origin.x;
    float final_dz = last->z - rover_pose.origin.z;
    float final_dist_sq = final_dx * final_dx + final_dz * final_dz;

    // slow down for final waypoint (disabled - deceleration not needed)
    /*
    if (active_path.current == active_path.count - 1 && final_dist_sq < WAYPOINT_REACH_THRESHOLD){
        throttle *= sqrtf(final_dist_sq) / 1.5f;
    }
    */

    set_steer(steer);
    set_throttle(throttle);

    advance_mppi();
}

// END OF MPPI CONTROLLER

void set_waypoints(Waypoint *points, int count)
{
    int n = count < MAX_WAYPOINTS ? count : MAX_WAYPOINTS;

    if (n > 0) {
        int write_idx = 0;

        active_path.waypoints[write_idx++] = (Waypoint){
            .x = rover_pose.origin.x,
            .z = rover_pose.origin.z
        };

        for (int i = 0; i < n && write_idx < MAX_WAYPOINTS; i++) {
            active_path.waypoints[write_idx++] = points[i];
        }

        active_path.count = write_idx;
    } 
    else {
        active_path.count = 0;
    }

    active_path.current = 0;
    replan_request_sent = 0;
    awaiting_replan_scan = 0;

    // stale controls and reseed warm start
    memset(nom_steer, 0, sizeof(nom_steer));
    for (int i = 0; i < MPPI_HORIZON; i++){
        nom_throttle[i] = 0.35f;
    }
}


// RENDERING

void render_predicted_path(void)
{
    if (active_path.current >= active_path.count) return;

    // replay nominal sequence
    SimState sim_state = {
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = rover_pose.speed,
        .angular_speed = rover_pose.angular_speed,
        .wp_idx = active_path.current
    };

    glBegin(GL_LINE_STRIP);
    glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
    glVertex3f(sim_state.x, 0.25f, sim_state.z);

    for (int i = 0; i < PREDICTION_STEPS; i++) {
        if (sim_state.wp_idx >= active_path.count) break;
        advance_waypoint_by_proximity(&active_path,
                                      &sim_state.wp_idx,
                                      sim_state.x,
                                      sim_state.z,
                                      active_path.count);

        rollout_simulation(&sim_state, nom_throttle[i], nom_steer[i], PREDICTION_DT);
        
        // fade out
        float t = (float)i / PREDICTION_STEPS;
        glColor4f(0.0f, 1.0f - 0.5f*t, 1.0f, 1.0f - 0.3f*t);
        glVertex3f(sim_state.x, 0.25f, sim_state.z);
    }
    glEnd();
}

void render_pose_error(void)
{
    float pred_x = rover_pose.origin.x, pred_z = rover_pose.origin.z;
    float odom_x = odom_pose.origin.x, odom_z = odom_pose.origin.z;

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.7f, 0.0f);
    glVertex3f(odom_x, 0.3f, odom_z);
    glVertex3f(pred_x, 0.3f, pred_z);
    glEnd();

    glLineWidth(1.0f);
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(pred_x, 0.3f, pred_z);
    glColor3f(1.0f, 0.7f, 0.0f);
    glVertex3f(odom_x, 0.3f, odom_z);
    glEnd();
    glPointSize(1.0f);
}

void render_waypoints(void)
{
    if (active_path.count == 0) return;

    glDisable(GL_DEPTH_TEST);
    glLineWidth(1.5f);
    glBegin(GL_LINE_STRIP);
    for (int i = active_path.current; i < active_path.count; i++) {
        float t = (float)(i - active_path.current) /
                  (float)(active_path.count - active_path.current);
        glColor4f(0.3f, 0.8f, 0.6f, 1.0f - 0.5f * t);
        glVertex3f(active_path.waypoints[i].x, 0.25f, active_path.waypoints[i].z);
    }
    glEnd();
    
    glLineWidth(1.0f);
    glPointSize(12.0f);
    glBegin(GL_POINTS);
    for (int i = active_path.current; i < active_path.count; i++) {
        int cur = (i == active_path.current);
        glColor3f(cur ? 1.0f : 0.1f, cur ? 0.9f : 1.0f, cur ? 0.1f : 0.2f);
        glVertex3f(active_path.waypoints[i].x, 0.25f, active_path.waypoints[i].z);
    }
    glEnd();
    glPointSize(1.0f);
    glEnable(GL_DEPTH_TEST);
}