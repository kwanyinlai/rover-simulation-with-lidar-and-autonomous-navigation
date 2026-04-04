#include "piping/method_dispatcher.h"
#include "scene/frontier_projection.h"
#include "core/io_utils.h"
#include "lidar/sensor_control.h"
#include "rover/rover_controller.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/select.h>
#include <unistd.h>


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
            // This is an artifact of autonomous navigation which is not yet implemented yet
            (void) frontier_write_fd;
            /*
            Waypoint waypoints[MAX_WAYPOINTS];
            int waypoint_count = plan_frontier_path(
                waypoints,
                MAX_WAYPOINTS,
                occupancy_grid_3d,
                occupancy_grid_2d,
                &latest_rover_state
            );
            write(frontier_write_fd, &waypoint_count, sizeof(int));
            write(frontier_write_fd, waypoints, sizeof(Waypoint) * waypoint_count);
            */
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

        apply_updates_to_projected_map(column_summaries, updates, count,
                                       occupancy_grid_3d, occupancy_grid_2d,
                                       ROVER_HEIGHT_CELLS);
    }

    free(column_summaries);
}