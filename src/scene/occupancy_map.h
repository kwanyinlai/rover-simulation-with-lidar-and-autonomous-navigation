#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#define FREE_THRESHOLD -0.4f
#define OCCUPIED_THRESHOLD 0.4f

#include "core/vec3.h"
#include "piping/messages.h"

typedef struct {
    float *data;
    int width, height, depth;
    float cell_size;
    Vector3 origin;
} OccupancyMap;


#define VOXEL_IDX(occupancy_grid_3d, x, y, z) ((z) * (occupancy_grid_3d)->width * (occupancy_grid_3d)->height + (y) * (occupancy_grid_3d)->width + (x))

/**
 * @brief Allocate and initialize an occupancy map.
 */
void init_occupancy_map(OccupancyMap *occupancy_grid_3d, int width, int height, int depth, float cell_size, Vector3 origin);

/**
 * @brief Free occupancy map storage.
 */
void free_occupancy_map(OccupancyMap *occupancy_grid_3d);

/**
 * @brief Classify a voxel cell by map coordinates.
 * @return Current classified state.
 */
CELL_STATE occupancy_map_get_cell(const OccupancyMap *occupancy_grid_3d, int x, int y, int z);

/**
 * @brief Convert log-odds value to occupancy class label.
 */
CELL_STATE occupancy_map_classify_log_prob(float log_prob);

/**
 * @brief Check whether voxel coordinates are within map bounds.
 */
int occupancy_map_in_bounds(const OccupancyMap *occupancy_grid_3d, int x, int y, int z);

/**
 * @brief Ray-cast update from origin to hit and output voxel state changes.
 */
void occupancy_map_ray_cast(const OccupancyMap *occupancy_grid_3d, Vector3 origin, Vector3 hit, int did_hit, MapDelta *out_data);

/**
 * @brief Extract frontier candidates from occupancy map.
 * @param out_pos Output array of frontier positions.
 * @param max_frontiers Maximum frontier positions to write.
 * @return Number of frontiers written.
 */
int occupancy_map_get_frontier(const OccupancyMap *occupancy_grid_3d, Vector3 *out_pos, int max_frontiers);

/**
 * @brief Check if voxel is a frontier point.
 */
int is_frontier_point(const OccupancyMap *occupancy_grid_3d, int x, int y, int z);

/**
 * @brief Get raw log-odds value at a voxel.
 */
float occupancy_map_get_log_odds(const OccupancyMap *occupancy_grid_3d, int x, int y, int z);

/**
 * @brief Update per-column occupancy summaries from voxel updates.
 */
void update_column_summaries(ColumnSummary *column_summaries, VoxelUpdate *updates, int count, const OccupancyMap *occupancy_grid_3d);

#endif