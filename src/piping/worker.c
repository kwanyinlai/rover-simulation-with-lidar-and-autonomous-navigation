#include "piping/method_dispatcher.h"
#include "piping/messages.h"
#include "lidar/lidar_sensor.h"
#include "lidar/raycaster.h"
#include "lidar/sensor_control.h"
#include "core/noise.h"
#include "rover/rover_controller.h"

#include <math.h>
#include <string.h>
#include <unistd.h>

void run_worker_loop(int read_fd, int write_fd, TriangleArray *scene) {
    const float noise_factor = 0.01f;
    RayResultBatch ray_result_batch;
    RayBatch ray_batch;
    while (read(read_fd, &ray_batch, sizeof(RayBatch)) > 0) {
        ray_result_batch.count = 0;
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
        write(write_fd, &ray_result_batch, sizeof(RayResultBatch));
    }
}

void run_rollout_worker_loop(int read_fd, int write_fd, TriangleArray *scene)
{
    RolloutJob job;
    while (read(read_fd, &job, sizeof(RolloutJob)) > 0) {
        RolloutResult result;
        memset(&result, 0, sizeof(result));
        result.frame_id = job.frame_id;
        result.start_sample_idx = job.start_sample_idx;
        result.end_sample_idx = job.end_sample_idx;

        for (int i = job.start_sample_idx; i < job.end_sample_idx; i++) {
            const RolloutRequest *request = &job.request;
            result.costs[i] = mppi_compute_rollout_cost(scene,
                                                        &request->path_snapshot,
                                                        request->init_state,
                                                        request->horizon,
                                                        request->nom_steer,
                                                        request->nom_throttle,
                                                        request->steer_noise[i],
                                                        request->throttle_noise[i]);
        }

        write(write_fd, &result, sizeof(RolloutResult));
    }
}