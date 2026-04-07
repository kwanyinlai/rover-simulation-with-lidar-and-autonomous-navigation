#include "scene/frontier_projection.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OCCUPIED_CONFIDENCE_MARGIN 1.0f
#define FREE_CONFIDENCE_MARGIN 0.2f

static void seal_enclosed_regions(OccupancyMap *occupancy_grid_2d) {

    int *visited = calloc((occupancy_grid_2d->width * occupancy_grid_2d->depth), 
                           sizeof(int)
                         );
    int *queue = malloc((occupancy_grid_2d->width * occupancy_grid_2d->depth) * 
                         sizeof(int)
                        );

    int head = 0;
    int tail = 0;

    // find all boundary cells that are non-occupied to start multi-source BFS
    for (int x = 0; x < occupancy_grid_2d->width; x++) {
        int top_boundary_idx = x;
        int bottom_boundary_idx = (occupancy_grid_2d->depth - 1) * occupancy_grid_2d->width + x;

        if (occupancy_map_classify_log_prob(occupancy_grid_2d->data[top_boundary_idx]) != OCCUPIED && 
            !visited[top_boundary_idx]) 
        {
            visited[top_boundary_idx] = 1;
            queue[tail++] = top_boundary_idx;
        }

        if (occupancy_map_classify_log_prob(occupancy_grid_2d->data[bottom_boundary_idx]) != OCCUPIED && 
            !visited[bottom_boundary_idx]) 
        {
            visited[bottom_boundary_idx] = 1;
            queue[tail++] = bottom_boundary_idx;
        }
    }

    for (int z = 0; z < occupancy_grid_2d->depth; z++) {
        int left_boundary_idx = z * occupancy_grid_2d->width;
        int right_boundary_idx = z * occupancy_grid_2d->width + (occupancy_grid_2d->width - 1);

        if (occupancy_map_classify_log_prob(occupancy_grid_2d->data[left_boundary_idx]) != OCCUPIED && 
            !visited[left_boundary_idx])
        {
            visited[left_boundary_idx] = 1;
            queue[tail++] = left_boundary_idx;
        }
;
        if (occupancy_map_classify_log_prob(occupancy_grid_2d->data[right_boundary_idx]) != OCCUPIED &&
            !visited[right_boundary_idx])
        {
            visited[right_boundary_idx] = 1;
            queue[tail++] = right_boundary_idx;
        }
    }

    static const int x_dir[4] = {1, -1, 0, 0};
    static const int z_dir[4] = {0, 0, 1, -1};

    while (head < tail) {
        int idx = queue[head++];
        int x = idx % occupancy_grid_2d->width;
        int z = idx / occupancy_grid_2d->width;

        for (int i = 0; i < 4; i++) {
            int neighbour_x = x + x_dir[i];
            int neighbour_z = z + z_dir[i];
            if (neighbour_x < 0 || neighbour_x >= occupancy_grid_2d->width || 
                neighbour_z < 0 || neighbour_z >= occupancy_grid_2d->depth)
            {
                continue;
            }

            int neighbour_idx = neighbour_z * occupancy_grid_2d->width + neighbour_x;
            if (visited[neighbour_idx] ||
                occupancy_map_classify_log_prob(occupancy_grid_2d->data[neighbour_idx]) == OCCUPIED)
            {
                continue;
            }

            visited[neighbour_idx] = 1;
            queue[tail++] = neighbour_idx;
        }
    }

    // non-occupied cell not connected to boundary is enclosed
    for (int idx = 0; idx < occupancy_grid_2d->width * occupancy_grid_2d->depth; idx++) {
        if (occupancy_map_classify_log_prob(occupancy_grid_2d->data[idx]) == OCCUPIED || visited[idx]) {
            continue;
        }
        occupancy_grid_2d->data[idx] = OCCUPIED_THRESHOLD + OCCUPIED_CONFIDENCE_MARGIN;
    }

    free(visited);
    free(queue);
}

ColumnSummary *create_column_summaries(const OccupancyMap *occupancy_grid_3d,
                                       int rover_height_cells) {
    int column_count = occupancy_grid_3d->width * occupancy_grid_3d->depth;
    ColumnSummary *column_summaries = malloc((int)column_count * sizeof(ColumnSummary));
    if (!column_summaries) {
        perror("malloc");
        return NULL;
    }

    for (int i = 0; i < column_count; i++) {
        column_summaries[i].occupied_count = 0;
        column_summaries[i].unknown_count = rover_height_cells;
    }

    return column_summaries;
}

void apply_updates_to_projected_map(ColumnSummary *column_summaries,
                                    const VoxelUpdate *updates,
                                    int count,
                                    const OccupancyMap *occupancy_grid_3d,
                                    OccupancyMap *occupancy_grid_2d,
                                    int rover_height_cells) {

    update_column_summaries(column_summaries, (VoxelUpdate *)updates, count, occupancy_grid_3d);

    for (int i = 0; i < count; i++) {
        const VoxelUpdate *update = &updates[i];
        int x = update->idx % occupancy_grid_3d->width;
        int y = (update->idx / occupancy_grid_3d->width) % occupancy_grid_3d->height;
        int z = update->idx / (occupancy_grid_3d->width * occupancy_grid_3d->height);

        if (y >= rover_height_cells) {
            continue;
        }
        ColumnSummary *col = &(column_summaries[x * occupancy_grid_3d->depth + z]);
        if (col->occupied_count > 0) {
            occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = 
                OCCUPIED_THRESHOLD + OCCUPIED_CONFIDENCE_MARGIN;
        }
        else if (col->unknown_count > 0) {
            occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = 0.0f;
        }
        else {
            occupancy_grid_2d->data[z * occupancy_grid_2d->width + x] = 
                FREE_THRESHOLD - FREE_CONFIDENCE_MARGIN;
        }
    }

    if (count > 0) {
        seal_enclosed_regions(occupancy_grid_2d);
    }
}