#include "piping/method_dispatcher.h"
#include "scene/occupancy_map.h"
#include "core/io_utils.h"
#include "lidar/lidar_sensor.h"
#include "piping/messages.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void run_occupancy_updater_loop(int read_fd, int write_fd, OccupancyMap *occupancy_grid_3d) {
    RayResultBatch ray_result_batch;
    while (1) {
        int status = read_exact(read_fd, &ray_result_batch, sizeof(RayResultBatch));
        if (status == 0) {
            break;
        }
        if (status < 0) {
            exit(1);
        }
        MapDelta map_delta = {.count = 0};
        for (int i = 0; i < ray_result_batch.count; i++) {
            RayResult *r = &ray_result_batch.rays[i];
            Vector3 dir = vector3_normalize(vector3_subtract(r->hit, ray_result_batch.origin));
            if (r->distance > 0.0f) {
                occupancy_map_ray_cast(occupancy_grid_3d, ray_result_batch.origin, r->hit, 1, &map_delta);
            } else {
                Vector3 max_range_point = {
                    ray_result_batch.origin.x + dir.x * MAX_RANGE,
                    ray_result_batch.origin.y + dir.y * MAX_RANGE,
                    ray_result_batch.origin.z + dir.z * MAX_RANGE
                };
                occupancy_map_ray_cast(occupancy_grid_3d, ray_result_batch.origin, max_range_point, 0, &map_delta);
            }
            if (map_delta.count >= MAX_UPDATED_VOXELS) {
                if (write_exact(write_fd, &(map_delta.count), sizeof(int)) < 0) {
                    exit(1);
                }
                if (write_exact(write_fd, &(map_delta.updates), sizeof(VoxelUpdate) * map_delta.count) < 0) {
                    exit(1);
                }
                map_delta.count = 0;
            }
        }

        if (map_delta.count > 0) {
            if (write_exact(write_fd, &(map_delta.count), sizeof(int)) < 0) {
                exit(1);
            }
            if (write_exact(write_fd, &(map_delta.updates), sizeof(VoxelUpdate) * map_delta.count) < 0) {
                exit(1);
            }
            map_delta.count = 0;
        }
    }
}