#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include "rendering/vec3.h"

typedef enum {
    UNKNOWN = 0,
    FREE = 1,
    OCCUPIED = 2
} CELL_STATE;

typedef struct {
    float *data;
    int width, height, depth;
    float cell_size;
    Vector3 origin;
} OccupancyMap;

#define VOXEL_IDX(map, x, y, z) ((z) * (map)->width * (map)->height + (y) * (map)->width + (x))

void init_occupancy_map(OccupancyMap *map, int width, int height, int depth, float cell_size, Vector3 origin);
void free_occupancy_map(OccupancyMap *map);

CELL_STATE occupancy_map_get_cell(const OccupancyMap *map, int x, int y, int z);
CELL_STATE occupancy_map_classify_log_prob(float log_prob);
int occupancy_map_in_bounds(const OccupancyMap *map, int x, int y, int z);

void occupancy_map_ray_cast(const OccupancyMap *map, Vector3 origin, Vector3 hit, int did_hit);

int occupancy_map_get_frontier(const OccupancyMap *map, Vector3 *out_pos, int max_frontiers);

int is_frontier_point(const OccupancyMap *map, int x, int y, int z);


#endif