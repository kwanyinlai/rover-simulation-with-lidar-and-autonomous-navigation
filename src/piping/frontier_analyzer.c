#include "piping/method_dispatcher.h"
#include "frontier_exploration/frontier_planner.h"
#include "scene/frontier_projection.h"
#include "core/io_utils.h"
#include "lidar/sensor_control.h"
#include "rover/rover_controller.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/select.h>
#include <unistd.h>

#define EXPLORATION_LOG_INTERVAL_UPDATES 200

typedef struct {
    int free_count;
    int occupied_count;
    int unknown_count;
    int total_count;
} ExplorationStats;

static ExplorationStats init_exploration_stats(const OccupancyMap *occupancy_grid_3d) {
    ExplorationStats stats = {0};
    stats.total_count = occupancy_grid_3d->width * occupancy_grid_3d->height * occupancy_grid_3d->depth;
    for (int idx = 0; idx < stats.total_count; idx++) {
        CELL_STATE state = occupancy_map_classify_log_prob(occupancy_grid_3d->data[idx]);
        if (state == FREE) {
            stats.free_count++;
        }
        else if (state == OCCUPIED) {
            stats.occupied_count++;
        }
        else {
            stats.unknown_count++;
        }
    }
    return stats;
}

static void apply_exploration_updates(ExplorationStats *stats,
                                      const VoxelUpdate *updates,
                                      int count) {
    for (int i = 0; i < count; i++) {
        if (updates[i].prev_state == FREE) {
            stats->free_count--;
        }
        else if (updates[i].prev_state == OCCUPIED) {
            stats->occupied_count--;
        }
        else {
            stats->unknown_count--;
        }

        if (updates[i].new_state == FREE) {
            stats->free_count++;
        }
        else if (updates[i].new_state == OCCUPIED) {
            stats->occupied_count++;
        }
        else {
            stats->unknown_count++;
        }
    }
}

static void log_exploration_stats(const ExplorationStats *stats) {
        fprintf(stderr,
            "known=%.2f%%, free=%d occupied=%d unknown=%d\n",
            100.0f * (float)(stats->free_count + stats->occupied_count) / (float)stats->total_count,
            stats->free_count,
            stats->occupied_count,
            stats->unknown_count);
}


void run_frontier_analyzer_loop(int voxel_update_read_fd,
                                int frontier_write_fd,
                                int rover_pose_read_fd,
                                const OccupancyMap *occupancy_grid_3d,
                                OccupancyMap *occupancy_grid_2d) {
    ColumnSummary *column_summaries = create_column_summaries(
        occupancy_grid_3d,
        ROVER_HEIGHT_CELLS
    );
    if (!column_summaries) {
        return;
    }

    SensorState latest_rover_state = {0};
    VoxelUpdate updates[MAX_UPDATED_VOXELS];
    int count = 0;
    ExplorationStats exploration_stats = init_exploration_stats(occupancy_grid_3d);
    int updates_since_log = 0;

    // log_exploration_stats(&exploration_stats);

    while (1) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(voxel_update_read_fd, &read_fds);
        FD_SET(rover_pose_read_fd, &read_fds);
        int max_fd = voxel_update_read_fd > rover_pose_read_fd ? voxel_update_read_fd : rover_pose_read_fd;

        int ready = select(max_fd + 1, &read_fds, NULL, NULL, NULL);
        if (ready < 0) {
            perror("select frontier analyzer inputs");
            exit(1);
        }
        if (ready == 0) {
            continue;
        }

        if (FD_ISSET(rover_pose_read_fd, &read_fds)) {
            int pose_read = read_exact(rover_pose_read_fd, &latest_rover_state, sizeof(SensorState));
            if (pose_read == 0) {
                break;
            }
            if (pose_read < 0) {
                exit(1);
            }

            Waypoint waypoints[MAX_WAYPOINTS];
            int waypoint_count = plan_frontier_path(
                waypoints,
                MAX_WAYPOINTS,
                occupancy_grid_2d,
                &latest_rover_state
            );

            if (waypoint_count < 0) {
                waypoint_count = 0;
            }
            if (waypoint_count > MAX_WAYPOINTS) {
                waypoint_count = MAX_WAYPOINTS;
            }


            if (write_all(frontier_write_fd, &waypoint_count, sizeof(int)) < 0) {
                perror("write frontier waypoint count");
                exit(1);
            }

            if (waypoint_count > 0) {
                if (write_all(frontier_write_fd, waypoints, sizeof(Waypoint) * (size_t)waypoint_count) < 0) {
                    perror("write frontier waypoints");
                    exit(1);
                }
            }
        }

        if (!FD_ISSET(voxel_update_read_fd, &read_fds)) {
            continue;
        }

        count = 0;
        int count_read = read_exact(voxel_update_read_fd, &count, sizeof(int));
        if (count_read == 0) {
            break;
        }
        if (count_read < 0) {
            exit(1);
        }

        int updates_read = read_exact(voxel_update_read_fd, updates, sizeof(VoxelUpdate) * count);
        if (updates_read == 0) {
            break;
        }
        if (updates_read < 0) {
            exit(1);
        }

        apply_exploration_updates(&exploration_stats, updates, count);
        updates_since_log += count;

        if (updates_since_log >= EXPLORATION_LOG_INTERVAL_UPDATES) {
            // log_exploration_stats(&exploration_stats);
            updates_since_log = 0;
        }


        apply_updates_to_projected_map(column_summaries, updates, count,
                                       occupancy_grid_3d, occupancy_grid_2d,
                                       ROVER_HEIGHT_CELLS);
    }

    free(column_summaries);
}