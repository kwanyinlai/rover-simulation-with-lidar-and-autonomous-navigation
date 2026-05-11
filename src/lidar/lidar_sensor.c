#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "core/noise.h"
#include "core/physics_constants.h"
#include "core/io_utils.h"
#include "lidar/lidar_sensor.h"
#include "lidar/raycaster.h"
#include "lidar/sensor_control.h"
#include "piping/messages.h"
#include "scene/occupancy_map.h"
#include "scene/point_cloud.h"
 

#define STEP 0.02f
#define SWEEP_SLICES_PER_FRAME 8

float elevations[NUM_RINGS];


static const float min_elev_angle = -30.0f * MATH_DEG_TO_RAD;
static const float max_elev_angle = 89.0f * MATH_DEG_TO_RAD;

// Measured as angle from the X-Z plane.
static float theta;

static int g_scan_cmd_fd = -1;
static int g_ray_batch_results_fd = -1;

void set_scan_pipe_fds(int scan_cmd_fd, int ray_batch_results_fd) {
    g_scan_cmd_fd = scan_cmd_fd;
    g_ray_batch_results_fd = ray_batch_results_fd;
}

void init_scan_state(ScanState *scan_state) {

    init_point_cloud(&scan_state->current);
    init_point_cloud(&scan_state->latest);
    init_point_cloud(&scan_state->previous);
    scan_state->has_previous = 0;
    scan_state->sweep_ready = 0;
}

int poll_scan_pair(ScanState *scan_state,
                   const PointCloud **latest,
                   const PointCloud **previous) {
    if (!scan_state->sweep_ready || !scan_state->has_previous){
        return 0;
    }
    *latest = &scan_state->latest;
    *previous = &scan_state->previous;
    scan_state->sweep_ready = 0;
    // updated sweep pair has been consumed now so mark as not ready
    return 1;
}

/* LEGACY CAST RAYS
void cast_all_rays(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *occupancy_grid_3d){
    const float noise_factor = 0.01f; // realistic noise strength (2% of distance)
    for (int i = 0 ; i < NUM_RINGS ; i++){
        Vector3 hit;
        // converting theta and elevation to a normalised vector
        float x = cosf(theta) * cosf(elevations[i]); 
        // technically since should be MATH_PI / 2 - polar, but equivalent to converting 
        // from cos to sin, and vice versa.
        float y = sinf(elevations[i]);
        float z = sinf(theta) * cosf(elevations[i]);
        Vector3 dir = {x, y, z};
        float intensity;
        Vector3 origin;
        get_sensor_pos(&origin);
        float dist = cast_ray(scene, &(origin), dir, &hit, &intensity);
        if (dist > 0.0f) {
            occupancy_map_ray_cast(occupancy_grid_3d, origin, hit, 1);
        } else {
            Vector3 max_range_point = {
                origin.x + dir.x * MAX_RANGE,
                origin.y + dir.y * MAX_RANGE,
                origin.z + dir.z * MAX_RANGE
            };
            occupancy_map_ray_cast(occupancy_grid_3d, origin, max_range_point, 0);
        }
        if (dist > 0) {
            float noise = gaussian_noise();
            float noisy_dist = fmax(dist + noise * (noise_factor * dist), 0.01f); // clamp to avoid negative or zero distance
            Vector3 noisy_pos = {
                origin.x + dir.x * noisy_dist,
                origin.y + dir.y * noisy_dist,
                origin.z + dir.z * noisy_dist
            };
            point_cloud_push_back(point_cloud, noisy_pos, noisy_dist, intensity);
        }
    }
}
*/

void cast_all_rays(const TriangleArray *scene, 
                   PointCloud *point_cloud,
                   OccupancyMap *occupancy_grid_3d,
                   ScanState *scan_state) {
    
    (void) scene; 
    (void) occupancy_grid_3d;
    // unused but kept to support backwards compatabiliy with legacy raycasting with no
    // multiprocessing

    ScanRequest scan_request = {
        .theta = theta,
        .max_elev = max_elev_angle,
        .min_elev = min_elev_angle,
        .num_rings = NUM_RINGS
    };

    get_sensor_pos(&scan_request.origin);

    if (write_all(g_scan_cmd_fd, &scan_request, sizeof(ScanRequest)) <= 0) {
        exit(1);
    } 

    RayResultBatch ray_result_batch;
    for (int i = 0; i < NUM_WORKERS; i++) {
        int n = read_exact(g_ray_batch_results_fd, &ray_result_batch, sizeof(RayResultBatch));
        if (n <= 0) {
            if (n == 0) {
                fprintf(stderr, "ray batch result pipe closed unexpectedly\n");
            } else {
                perror("read ray result batch");
            }
            exit(1);
        }

        for (int j = 0; j < ray_result_batch.count; j++) {
            RayResult *r = &ray_result_batch.rays[j];
            if (r->distance > 0.0f) {
                point_cloud_push_back(point_cloud, r->hit, r->distance, r->intensity);
                if (scan_state) {
                    point_cloud_push_back(&scan_state->current, r->hit, r->distance, r->intensity);
                }
            }
        }
    }
}

void sensor_step(const TriangleArray *scene,
                 PointCloud *point_cloud, 
                 OccupancyMap *occupancy_grid_3d,
                 ScanState *scan_state) {
    for (int i = 0; i < SWEEP_SLICES_PER_FRAME; i++) {
        float prev_theta = theta;
        theta += STEP;
        if (theta >= (2.0f * M_PI)) {
            theta = fmodf(theta, 2.0f * M_PI);
        }
        cast_all_rays(scene, point_cloud, occupancy_grid_3d, scan_state);

        if (theta < prev_theta && scan_state) {
            if (scan_state->has_previous) {
                PointCloud tmp = scan_state->previous;
                scan_state->previous = scan_state->latest;
                scan_state->latest = tmp;
            } 
            else {
                scan_state->previous = scan_state->latest;
                scan_state->has_previous = 1;
            }
            PointCloud tmp = scan_state->latest;
            scan_state->latest = scan_state->current;
            scan_state->current = tmp;
            scan_state->sweep_ready = 1;
            point_cloud_clear(&scan_state->current);
        }
    }
}

float get_scan_theta(void) {
    return theta;
}

void init_sensor_rays(void) {
    for (int i = 0 ; i < NUM_RINGS ; i++){
        elevations[i] = min_elev_angle +
          i * (max_elev_angle - min_elev_angle) / NUM_RINGS;
    }
}