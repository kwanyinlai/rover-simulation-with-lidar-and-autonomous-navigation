#include "piping/method_dispatcher.h"
#include "piping/messages.h"
#include "lidar/lidar_sensor.h"
#include "lidar/raycaster.h"
#include "lidar/sensor_control.h"
#include "core/noise.h"
#include "core/io_utils.h"
#include "rover/rover_controller.h"
#include "core/noise_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

void run_worker_loop(int read_fd, int write_fd, TriangleArray *scene) {
    const float noise_factor = LIDAR_RANGE_NOISE_FACTOR;
    RayResultBatch ray_result_batch;
    RayBatch ray_batch;
    while (1) {
        int status = read_exact(read_fd, &ray_batch, sizeof(RayBatch));
        if (status == 0) {
            break;
        }
        if (status < 0) {
            exit(1);
        }
        // invariants failed
        if (ray_batch.start_ray_idx < 0 ||
            ray_batch.end_ray_idx > NUM_RINGS ||
            ray_batch.start_ray_idx > ray_batch.end_ray_idx ||
            (ray_batch.end_ray_idx - ray_batch.start_ray_idx) != ray_batch.num_rays) {
            fprintf(stderr,
                    "discarding invalid RayBatch [%d, %d)\n",
                    ray_batch.start_ray_idx,
                    ray_batch.end_ray_idx);
            continue;
        }
        ray_result_batch.count = 0;
        ray_result_batch.theta = ray_batch.theta;
        ray_result_batch.origin = ray_batch.origin;
        for (int i = ray_batch.start_ray_idx; i < ray_batch.end_ray_idx; i++) {
            Vector3 hit;
            float x = cosf(ray_batch.theta) * cosf(elevations[i]);
            float y = sinf(elevations[i]);
            float z = sinf(ray_batch.theta) * cosf(elevations[i]);
            Vector3 dir = {x, y, z};
            float intensity;
            Vector3 origin = ray_batch.origin;
            ray_result_batch.origin = ray_batch.origin;
            float dist = cast_ray(scene, &(origin), dir, &hit, &intensity);
            if (dist > 0) {
                float noise = gaussian_noise();
                float noisy_dist = fmax(dist + noise * (noise_factor * dist), 0.01f);
                Vector3 noisy_pos = {
                    origin.x + dir.x * noisy_dist,
                    origin.y + dir.y * noisy_dist,
                    origin.z + dir.z * noisy_dist
                };
                ray_result_batch.rays[ray_result_batch.count++] = (RayResult){
                    .distance = noisy_dist,
                    .intensity = intensity,
                    .hit = noisy_pos
                };
            } else {
                ray_result_batch.rays[ray_result_batch.count++] = (RayResult){
                    .distance = -1.0f,
                    .intensity = 0.0f,
                    .hit = {
                        origin.x + dir.x,
                        origin.y + dir.y,
                        origin.z + dir.z
                    }
                };
            }
        }
        if (write_all(write_fd, &ray_result_batch, sizeof(RayResultBatch)) < 0) {
            exit(1);
        }
    }
}

void run_rollout_worker_loop(int read_fd, int write_fd, TriangleArray *scene)
{
    RolloutJob job;
    while (1) {
        int status = read_exact(read_fd, &job, sizeof(RolloutJob));
        if (status == 0) {
            break;
        }
        if (status < 0) {
            exit(1);
        }
        // invariants failed
        if (job.start_sample_idx < 0 ||
            job.end_sample_idx > MPPI_SAMPLES ||
            job.start_sample_idx > job.end_sample_idx ||
            job.request.horizon <= 0 ||
            job.request.horizon > MPPI_HORIZON) {
            fprintf(stderr,
                    "discarding invalid RolloutJob [%d, %d)\n",
                    job.start_sample_idx,
                    job.end_sample_idx);
            continue;
        }

        RolloutResult result;
        memset(&result, 0, sizeof(result));
        result.start_sample_idx = job.start_sample_idx;
        result.end_sample_idx = job.end_sample_idx;

        for (int i = job.start_sample_idx; i < job.end_sample_idx; i++) {
            const RolloutRequest *request = &job.request;
            int collided = 0;
            result.costs[i] = mppi_compute_rollout_cost(scene,
                                                        &request->path_snapshot,
                                                        request->init_state,
                                                        request->horizon,
                                                        request->nom_steer,
                                                        request->nom_throttle,
                                                        request->steer_noise[i],
                                                        request->throttle_noise[i],
                                                        &collided);
            result.collisions[i] = (unsigned char)(collided ? 1 : 0);
        }

        if (write_all(write_fd, &result, sizeof(RolloutResult)) < 0) {
            exit(1);
        }
    }
}
