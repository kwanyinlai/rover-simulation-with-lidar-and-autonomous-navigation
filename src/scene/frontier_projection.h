#ifndef FRONTIER_PROJECTION_H
#define FRONTIER_PROJECTION_H

#include "piping/messages.h"
#include "scene/occupancy_map.h"

/**
 * @brief Build 2D column summaries from a 3D occupancy map.
 * @param occupancy_grid_3d Source 3D occupancy map.
 * @param rover_height_cells Number of cells spanned by rover height.
 * @return Allocated column summary array, owned by caller.
 */
ColumnSummary *create_column_summaries(const OccupancyMap *occupancy_grid_3d,
                                       int rover_height_cells);

/**
 * @brief Apply 3D voxel updates to projected 2D occupancy map.
 */
void apply_updates_to_projected_map(ColumnSummary *column_summaries,
                                    const VoxelUpdate *updates,
                                    int count,
                                    const OccupancyMap *occupancy_grid_3d,
                                    OccupancyMap *occupancy_grid_2d,
                                    int rover_height_cells);

#endif // FRONTIER_PROJECTION_H