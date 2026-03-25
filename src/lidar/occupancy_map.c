#include <float.h> // for FLT_MAX
#include "lidar/occupancy_map.h"

#define FREE_THRESHOLD -0.4f
#define OCCUPIED_THRESHOLD 0.4f

void init_occupancy_map(OccupancyMap *map, int width, int height, int depth, float cell_size, Vector3 origin){
    map->width = width;
    map->height = height; 
    map->depth = depth;
    map->cell_size = cell_size;
    map->origin = origin;
    size_t total_cells = (size_t)width * height * depth;
    map->data = calloc(total_cells, sizeof(float));
    if (map->data == NULL) {
        perror("calloc");
        exit(1);
    }
}

void free_occupancy_map(OccupancyMap *map){
    free(map->data);
    map->data = NULL;
    map->width = 0;
    map->height = 0;
    map->depth = 0;
}

CELL_STATE occupancy_map_get_cell(const OccupancyMap *map, int x, int y, int z){
    if (!occupancy_map_in_bounds(map, x, y, z)) return UNKNOWN;
    float log_prob = map->data[VOXEL_IDX(map, x, y, z)];
    return occupancy_map_classify_log_prob(log_prob);
}

CELL_STATE occupancy_map_classify_log_prob(float log_prob){
    if (log_prob < FREE_THRESHOLD) return FREE;
    else if (log_prob > OCCUPIED_THRESHOLD) return OCCUPIED;
    else return UNKNOWN;
}

int occupancy_map_in_bounds(const OccupancyMap *map, int x, int y, int z){
    return x >= 0 && x < map->width &&
           y >= 0 && y < map->height &&
           z >= 0 && z < map->depth;
}

// @see https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)
// 
// interestingly, this algorithm was developed at UofT -- ignore the YorkU affiliation
// @see http://www.cse.yorku.ca/~amana/research/grid.pdf
void occupancy_map_ray_cast(const OccupancyMap *map, Vector3 origin, Vector3 hit, int did_hit){

    int x = (int)floorf((origin.x - map->origin.x) / map->cell_size);
    int y = (int)floorf((origin.y - map->origin.y) / map->cell_size);
    int z = (int)floorf((origin.z - map->origin.z) / map->cell_size);

    int hit_x = (int)floorf((hit.x - map->origin.x) / map->cell_size);
    int hit_y = (int)floorf((hit.y - map->origin.y) / map->cell_size);
    int hit_z = (int)floorf((hit.z - map->origin.z) / map->cell_size);

    // determine step direction based on sign of hit coordinate
    Vector3 dir = vector3_normalize(vector3_subtract(hit, origin));
 
    float step_x, step_y, step_z;
    float t_x, t_y, t_z;
    float dx, dy, dz;

    if (fabsf(dir.x) < 1e-6f) { // negligibly small, treat as 0
        step_x = 0;
        t_x = FLT_MAX;
    } else if (dir.x > 0) {
        step_x = 1;
        t_x = ((x + 1) * map->cell_size + map->origin.x - origin.x) / dir.x;
    } else {
        step_x = -1;
        t_x = (x * map->cell_size + map->origin.x - origin.x) / dir.x;
    }

    if (fabsf(dir.y) < 1e-6f) {
        step_y = 0;
        t_y = FLT_MAX;
    } else if (dir.y > 0) {
        step_y = 1;
        t_y = ((y + 1) * map->cell_size + map->origin.y - origin.y) / dir.y;
    } else {
        step_y = -1;
        t_y = (y * map->cell_size + map->origin.y - origin.y) / dir.y;
    }

    if (fabsf(dir.z) < 1e-6f) {
        step_z = 0;
        t_z = FLT_MAX;
    } else if (dir.z > 0) {
        step_z = 1;
        t_z = ((z + 1) * map->cell_size + map->origin.z - origin.z) / dir.z;
    } else {
        step_z = -1;
        t_z = (z * map->cell_size + map->origin.z - origin.z) / dir.z;
    }
  

    while (occupancy_map_in_bounds(map, x, y, z) && \
              (x != hit_x || y != hit_y || z != hit_z))
        {
        map->data[VOXEL_IDX(map, x, y, z)] -= 0.1f; // update log-odds since implicitly free by ray passing
        if (t_x < t_y && t_x < t_z) {
            x += step_x * t_x;
            t_x += dx;
        } else if (t_y < t_z) { // && t_y < t_x
            y += step_y * t_y;
            t_y += dy;
        } else { // t_z <= t_x && t_z <= t_y
            z += step_z * t_z;
            t_z += dz;
        }
        map->data[VOXEL_IDX(map, x, y, z)] += did_hit ? 0.1f : -0.1f; 
    }

    if (did_hit && occupancy_map_in_bounds(map, hit_x, hit_y, hit_z)) {
        map->data[VOXEL_IDX(map, hit_x, hit_y, hit_z)] += 0.1f; 
    }
}

int occupancy_map_get_frontier(const OccupancyMap *map, Vector3 *out_pos, int max_frontiers){
    int count = 0;
    for (int z = 0; z < map->depth; z++) 
    for (int y = 0; y < map->height; y++) 
    for (int x = 0; x < map->width; x++) {
        if (occupancy_map_get_cell(map, x, y, z) != FREE) {
            continue;
        }
        else if (is_frontier_point(map, x, y, z)){
            if (count < max_frontiers) {
                out_pos[count++] = (Vector3){
                    map->origin.x + x * map->cell_size + map->cell_size / 2,
                    map->origin.y + y * map->cell_size + map->cell_size / 2,
                    map->origin.z + z * map->cell_size + map->cell_size / 2
                };
            }
        }
    }
    return count;
}

int is_frontier_point(const OccupancyMap *map, int x, int y, int z){
    for (int face_idx = 0; face_idx < 6; face_idx++) {
        if (occupancy_map_get_cell(map, x + (face_idx == 0) - (face_idx == 1), 
                                    y + (face_idx == 2) - (face_idx == 3), 
                                    z + (face_idx == 4) - (face_idx == 5)) == UNKNOWN) {
            return 1;
        }
    }
    return 0;
}