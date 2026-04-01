# include "piping/method_dispatcher.h"
# include "scene/occupancy_map.h"
# include "piping/messages.h"
# include "lidar/lidar_sensor.h"
# include "lidar/raycaster.h"
# include "lidar/sensor_control.h"
# include "rover/rover_controller.h"
# include <unistd.h>
# include "core/noise.h"
#include <stdlib.h>




void run_occupancy_updater_loop(int read_fd, int write_fd, OccupancyMap* occupancy_grid_3d){
    RayResultBatch ray_result_batch;
    while (read(read_fd, &ray_result_batch, sizeof(RayResultBatch)) > 0) {
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
            // flush out buffer early if we overflow to avoid large memory usage spikes
            if (map_delta.count >= MAX_UPDATED_VOXELS){
                write(write_fd, &(map_delta.count), sizeof(int));
                write(write_fd, &(map_delta.updates), sizeof(VoxelUpdate) * map_delta.count);
                map_delta.count = 0;
            }
        }

        if (map_delta.count > 0){
            write(write_fd, &(map_delta.count), sizeof(int));
            write(write_fd, &(map_delta.updates), sizeof(VoxelUpdate) * map_delta.count);
            map_delta.count = 0;
        }
    }
    

}

void run_worker_loop(int read_fd, int write_fd, TriangleArray* scene){
    extern float elevations[NUM_RINGS];
    const float noise_factor = 0.01f;
    RayResultBatch ray_result_batch;
    RayBatch ray_batch;
    while (read(read_fd, &ray_batch, sizeof(RayBatch)) > 0) {
        ray_result_batch.origin = ray_batch.origin;
        for (int i = ray_batch.start_ray_idx ; i < ray_batch.end_ray_idx; i++){
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
            }
            else{
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
        ray_result_batch.count = 0; // reset for next batch
    }
}


void run_coordinator_loop(int scan_cmd_read_fd, int ray_batch_writes_fd,
                          int ray_task_pipes[NUM_WORKERS][2], int ray_results_pipes[NUM_WORKERS][2],
                          int point_batch_write_fd)
{

    ScanRequest scan_request;
    int rings_per_worker = NUM_RINGS / NUM_WORKERS; 
    // we might lose some rays if NUM_RINGS is not divisible by NUM_WORKERS, but that's fine
    while (read(scan_cmd_read_fd, &scan_request, sizeof(ScanRequest)) > 0) {
        // printf("Coordinator started, waiting for scan requests...\n");
        RayBatch ray_batches[NUM_WORKERS];
        for (int i = 0; i < NUM_WORKERS; i++) {
            ray_batches[i].origin = scan_request.origin;
            ray_batches[i].theta = scan_request.theta;
            ray_batches[i].start_ray_idx = i * rings_per_worker;
            ray_batches[i].end_ray_idx = (i + 1) * rings_per_worker;
            ray_batches[i].num_rays = rings_per_worker;
            // we won't bother collecting extra rays
            write(ray_task_pipes[i][1], &ray_batches[i], sizeof(RayBatch));
        }
        
        int workers_done = 0;
        int done[NUM_WORKERS] = {0};

        while (workers_done < NUM_WORKERS) {
            // fd_set pattern
            fd_set read_fds;
            FD_ZERO(&read_fds);
            int max_fd = -1;
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i]) {
                    FD_SET(ray_results_pipes[i][0], &read_fds);
                    max_fd = ray_results_pipes[i][0] > max_fd ? ray_results_pipes[i][0] : max_fd;
                }
            }
            select(max_fd + 1, &read_fds, NULL, NULL, NULL);
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i] && FD_ISSET(ray_results_pipes[i][0], &read_fds)) {
                    RayResultBatch ray_result_batch;
                    read(ray_results_pipes[i][0], &ray_result_batch, sizeof(RayResultBatch));
                    write(ray_batch_writes_fd, &ray_result_batch, sizeof(RayResultBatch));
                    write(point_batch_write_fd, &ray_result_batch, sizeof(RayResultBatch)); // also send to occupancy updater
                    done[i] = 1;
                    workers_done++;
                }
            }
        }

        
    }
    
}

void run_frontier_analyzer_loop(int voxel_update_read_fd, int frontier_write_fd,
                                const OccupancyMap *occupancy_grid_3d,
                                OccupancyMap *occupancy_grid_2d)
{
    ColumnSummary *column_summaries = malloc(
        occupancy_grid_3d->width * occupancy_grid_3d->depth * sizeof(ColumnSummary)
    );
    // set all cells to be blocking initially 
    for (int i = 0; i < occupancy_grid_3d->width * occupancy_grid_3d->depth; i++){
        column_summaries[i].is_blocking_count = ROVER_HEIGHT_CELLS;
    }
 
    int count = 0;
    while (read(voxel_update_read_fd, &count, sizeof(int)) > 0) {
        VoxelUpdate updates[MAX_UPDATED_VOXELS];
        read(voxel_update_read_fd, updates, sizeof(VoxelUpdate) * count);
 
        update_column_summaries(column_summaries, updates, count, occupancy_grid_3d);
 
        for (int i = 0; i < count; i++) {
            const VoxelUpdate *update = &updates[i];
            // convert 1D idx back to 3D coordinates
            int x = update->idx % occupancy_grid_3d->width;
            int y = (update->idx / occupancy_grid_3d->width) % occupancy_grid_3d->height;
            int z = update->idx / (occupancy_grid_3d->width * occupancy_grid_3d->height);
            
 
            if (y >= ROVER_HEIGHT_CELLS) continue;
 
            ColumnSummary *col = &(column_summaries[x * occupancy_grid_3d->depth + z]);
 
            if (col->is_blocking_count > 0) {
                occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = OCCUPIED_THRESHOLD + 1.0f;
            }
            else {
                occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = FREE_THRESHOLD - 0.2f;
            }
            // go above and below threshold so get_cell functions properly
        }
 
        // if (should_replan()) {
        //     Waypoint waypoints[MAX_WAYPOINTS];
        //     int n = plan_to_frontier(occupancy_grid_3d, column_summaries, waypoints);
        //     write(frontier_write_fd, &n,        sizeof(int));
        //     write(frontier_write_fd, waypoints, sizeof(Waypoint) * n);
        // }
    }
 
    free(column_summaries);
}
 

