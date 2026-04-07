#include <float.h> // for FLT_MAX
#include "scene/occupancy_map.h"
#include "lidar/lidar_sensor.h"
#include <sys/mman.h>
#include <unistd.h>
#include "rover/rover_controller.h"


#define HIT_LOG_ODDS 6.0f
#define MISS_LOG_ODDS 0.15f
#define REFERENCE_DIST 5.f
#define FLOOR_HIT_IGNORE_CELLS 1

void init_occupancy_map(OccupancyMap *occupancy_grid_3d, int width, int height, int depth, float cell_size, Vector3 origin){
    occupancy_grid_3d->width = width;
    occupancy_grid_3d->height = height; 
    occupancy_grid_3d->depth = depth;
    occupancy_grid_3d->cell_size = cell_size;
    occupancy_grid_3d->origin = origin;
    int total_cells = (int)width * height * depth;
    // use mmap for shared memory between processes
    occupancy_grid_3d->data = mmap(NULL, total_cells * sizeof(float), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    if (occupancy_grid_3d->data == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }
}

void free_occupancy_map(OccupancyMap *occupancy_grid_3d){
    munmap(occupancy_grid_3d->data, (int)occupancy_grid_3d->width * occupancy_grid_3d->height * occupancy_grid_3d->depth * sizeof(float));
    occupancy_grid_3d->data = NULL;
    occupancy_grid_3d->width = 0;
    occupancy_grid_3d->height = 0;
    occupancy_grid_3d->depth = 0;
}

CELL_STATE occupancy_map_get_cell(const OccupancyMap *occupancy_grid_3d, int x, int y, int z){
    if (!occupancy_map_in_bounds(occupancy_grid_3d, x, y, z)) return UNKNOWN;
    float log_prob = occupancy_grid_3d->data[VOXEL_IDX(occupancy_grid_3d, x, y, z)];
    return occupancy_map_classify_log_prob(log_prob);
}

CELL_STATE occupancy_map_classify_log_prob(float log_prob){
    if (log_prob < FREE_THRESHOLD) return FREE;
    else if (log_prob > OCCUPIED_THRESHOLD) return OCCUPIED;
    else return UNKNOWN;
}

int occupancy_map_in_bounds(const OccupancyMap *occupancy_grid_3d, int x, int y, int z){
    return x >= 0 && x < occupancy_grid_3d->width &&
           y >= 0 && y < occupancy_grid_3d->height &&
           z >= 0 && z < occupancy_grid_3d->depth;
}

// @see https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)
// interestingly, this algorithm was developed at UofT -- ignore the YorkU affiliation
// @see http://www.cse.yorku.ca/~amana/research/grid.pdf
void occupancy_map_ray_cast(const OccupancyMap *occupancy_grid_3d, Vector3 origin, 
    Vector3 hit, int did_hit, MapDelta *out_data){
    int x = (int)floorf((origin.x - occupancy_grid_3d->origin.x) / occupancy_grid_3d->cell_size);
    int y = (int)floorf((origin.y - occupancy_grid_3d->origin.y) / occupancy_grid_3d->cell_size);
    int z = (int)floorf((origin.z - occupancy_grid_3d->origin.z) / occupancy_grid_3d->cell_size);

    int hit_x = (int)floorf((hit.x - occupancy_grid_3d->origin.x) / occupancy_grid_3d->cell_size);
    int hit_y = (int)floorf((hit.y - occupancy_grid_3d->origin.y) / occupancy_grid_3d->cell_size);
    int hit_z = (int)floorf((hit.z - occupancy_grid_3d->origin.z) / occupancy_grid_3d->cell_size);

    // determine step direction based on sign of hit coordinate
    Vector3 dir = vector3_normalize(vector3_subtract(hit, origin));
 
    float step_x, step_y, step_z;
    float t_x, t_y, t_z;
    float dx, dy, dz;

    if (fabsf(dir.x) < 1e-6f) {
        step_x = 0;
        t_x = FLT_MAX;
        dx = FLT_MAX;
    }
    else if (dir.x > 0) {
        step_x = 1;
        t_x = ((x + 1) * occupancy_grid_3d->cell_size + occupancy_grid_3d->origin.x - origin.x) / dir.x;
        dx = occupancy_grid_3d->cell_size / fabsf(dir.x);
    }
    else {
        step_x = -1;
        t_x = (x * occupancy_grid_3d->cell_size + occupancy_grid_3d->origin.x - origin.x) / dir.x;
        dx = occupancy_grid_3d->cell_size / fabsf(dir.x);
    }

    if (fabsf(dir.y) < 1e-6f) {
        step_y = 0;
        t_y = FLT_MAX;
        dy = FLT_MAX;
    }
    else if (dir.y > 0) {
        step_y = 1;
        t_y = ((y + 1) * occupancy_grid_3d->cell_size + occupancy_grid_3d->origin.y - origin.y) / dir.y;
        dy = occupancy_grid_3d->cell_size / fabsf(dir.y);
    } 
    else {
        step_y = -1;
        t_y = (y * occupancy_grid_3d->cell_size + occupancy_grid_3d->origin.y - origin.y) / dir.y;
        dy = occupancy_grid_3d->cell_size / fabsf(dir.y);
    }

    if (fabsf(dir.z) < 1e-6f) {
        step_z = 0;
        t_z = FLT_MAX;
        dz = FLT_MAX;
    } 
    else if (dir.z > 0) {
        step_z = 1;
        t_z = ((z + 1) * occupancy_grid_3d->cell_size + occupancy_grid_3d->origin.z - origin.z) / dir.z;
        dz = occupancy_grid_3d->cell_size / fabsf(dir.z);
    } 
    else {
        step_z = -1;
        t_z = (z * occupancy_grid_3d->cell_size + occupancy_grid_3d->origin.z - origin.z) / dir.z;
        dz = occupancy_grid_3d->cell_size / fabsf(dir.z);
    }
  

    while (occupancy_map_in_bounds(occupancy_grid_3d, x, y, z) && \
              (x != hit_x || y != hit_y || z != hit_z))
        {
        float current_dist = fminf(t_x, fminf(t_y, t_z));
        float density_scale = (current_dist * current_dist) / 
                            (REFERENCE_DIST * REFERENCE_DIST);
        density_scale = fmaxf(0.1f, fminf(density_scale, 1.0f)); // clamp
        
        CELL_STATE prev_state = occupancy_map_get_cell(occupancy_grid_3d, x, y, z);
        occupancy_grid_3d->data[VOXEL_IDX(occupancy_grid_3d, x, y, z)] -= MISS_LOG_ODDS * density_scale;
        CELL_STATE new_state = occupancy_map_get_cell(occupancy_grid_3d, x, y, z);

        if (prev_state != new_state){
            if (out_data->count < MAX_UPDATED_VOXELS) {
                out_data->updates[out_data->count++] = (VoxelUpdate){
                    .idx = VOXEL_IDX(occupancy_grid_3d, x, y, z),
                    .new_state = new_state,
                    .prev_state = prev_state
                };
            }
        }

        if (t_x < t_y && t_x < t_z) {
            x += step_x;
            t_x += dx;
        } else if (t_y < t_z) { // && t_y < t_x
            y += step_y;
            t_y += dy;
        } else { // t_z <= t_x && t_z <= t_y
            z += step_z;
            t_z += dz;
        }
    }
    CELL_STATE prev_state = occupancy_map_get_cell(occupancy_grid_3d, hit_x, hit_y, hit_z);
    int is_floor_hit = (hit_y - FLOOR_HIT_IGNORE_CELLS) < 0;
    if (did_hit && !is_floor_hit && occupancy_map_in_bounds(occupancy_grid_3d, hit_x, hit_y, hit_z)) {
        occupancy_grid_3d->data[VOXEL_IDX(occupancy_grid_3d, hit_x, hit_y, hit_z)] += HIT_LOG_ODDS; 
    }
    CELL_STATE new_state = occupancy_map_get_cell(occupancy_grid_3d, hit_x, hit_y, hit_z);
    if (prev_state != new_state){
        if (out_data->count < MAX_UPDATED_VOXELS) {
            out_data->updates[out_data->count++] = (VoxelUpdate){
                .idx = VOXEL_IDX(occupancy_grid_3d, hit_x, hit_y, hit_z),
                .new_state = new_state,
                .prev_state = prev_state
            };
        }
    }
}


int occupancy_map_get_frontier(const OccupancyMap *occupancy_grid_3d, Vector3 *out_pos, int max_frontiers){
    int count = 0;
    for (int z = 0; z < occupancy_grid_3d->depth; z++) 
    for (int y = 0; y < occupancy_grid_3d->height; y++) 
    for (int x = 0; x < occupancy_grid_3d->width; x++) {
        if (occupancy_map_get_cell(occupancy_grid_3d, x, y, z) != FREE) {
            continue;
        }
        else if (is_frontier_point(occupancy_grid_3d, x, y, z)){
            if (count < max_frontiers) {
                out_pos[count++] = (Vector3){
                    occupancy_grid_3d->origin.x + x * occupancy_grid_3d->cell_size + occupancy_grid_3d->cell_size / 2,
                    occupancy_grid_3d->origin.y + y * occupancy_grid_3d->cell_size + occupancy_grid_3d->cell_size / 2,
                    occupancy_grid_3d->origin.z + z * occupancy_grid_3d->cell_size + occupancy_grid_3d->cell_size / 2
                };
            }
        }
    }
    return count;
}

int is_frontier_point(const OccupancyMap *occupancy_grid_3d, int x, int y, int z){

    // float wx = map->origin.x + (x + 0.5f) * map->cell_size;
    // float wy = map->origin.y + (y + 0.5f) * map->cell_size;
    // float wz = map->origin.z + (z + 0.5f) * map->cell_size;
    
    // Vector3 sensor_pos;
    // get_sensor_pos(&sensor_pos);
    // float dx = wx - sensor_pos.x;
    // float dy = wy - sensor_pos.y;
    // float dz = wz - sensor_pos.z;

    // if (dx * dx + dy * dy + dz * dz < 3.0f) return 0; // ignore stuff too close to the sensor
    for (int face_idx = 0; face_idx < 6; face_idx++) {
        if (occupancy_map_get_cell(occupancy_grid_3d, x + (face_idx == 0) - (face_idx == 1), 
                                    y + (face_idx == 2) - (face_idx == 3), 
                                    z + (face_idx == 4) - (face_idx == 5)) == UNKNOWN) {
            return 1;
        }
    }
    return 0;
}

float occupancy_map_get_log_odds(const OccupancyMap *occupancy_grid_3d, int x, int y, int z){
    return occupancy_grid_3d->data[VOXEL_IDX(occupancy_grid_3d, x, y, z)];
}



void update_column_summaries(ColumnSummary *column_summaries, VoxelUpdate *updates, int count, const OccupancyMap *occupancy_grid_3d){
    for (int i = 0; i < count; i++){
        const VoxelUpdate *voxel = &updates[i];

        int x = voxel->idx % occupancy_grid_3d->width;
        int y = (voxel->idx / occupancy_grid_3d->width) % occupancy_grid_3d->height;
        int z = voxel->idx / (occupancy_grid_3d->width * occupancy_grid_3d->height);

        if (y >= ROVER_HEIGHT_CELLS) continue;

        ColumnSummary *col = &column_summaries[x * occupancy_grid_3d->depth + z];

        // undo previous state
        if (voxel->prev_state == OCCUPIED) {
            col->occupied_count--;
        }
        else if (voxel->prev_state == UNKNOWN) {
            col->unknown_count--;
        }

        if (voxel->new_state == OCCUPIED) {
            col->occupied_count++;
        }
        else if (voxel->new_state == UNKNOWN) {
            col->unknown_count++;
        }

        if (col->occupied_count < 0) {
            col->occupied_count = 0;
        }
        if (col->unknown_count < 0) {
            col->unknown_count = 0;
        }
    }
}