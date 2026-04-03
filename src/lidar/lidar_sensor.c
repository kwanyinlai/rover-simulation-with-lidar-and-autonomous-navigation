#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "core/noise.h"
#include "core/physics_constants.h"
#include "lidar/lidar_sensor.h"
#include "lidar/raycaster.h"
#include "lidar/sensor_control.h"
#include "piping/messages.h"
#include "scene/occupancy_map.h"
#include "scene/point_cloud.h"

#define STEP 0.02f

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
                   OccupancyMap *occupancy_grid_3d) {
    
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
    write(g_scan_cmd_fd, &scan_request, sizeof(ScanRequest));
    RayResultBatch ray_result_batch;
    for (int i = 0; i < NUM_WORKERS; i++) {
        int n = read(g_ray_batch_results_fd, &ray_result_batch, sizeof(RayResultBatch));
        if (n == sizeof(RayResultBatch)) {
            for (int j = 0; j < ray_result_batch.count; j++) {
                RayResult *r = &ray_result_batch.rays[j];
                if (r->distance > 0.0f) {
                    point_cloud_push_back(point_cloud, r->hit, r->distance, r->intensity);
                }
            }
        }
    }
}

void sensor_step(const TriangleArray *scene,
                 PointCloud *point_cloud, 
                 OccupancyMap *occupancy_grid_3d) {

    theta += STEP;
    cast_all_rays(scene, point_cloud, occupancy_grid_3d);
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