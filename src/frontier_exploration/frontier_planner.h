#ifndef FRONTIER_PLANNER_H
#define FRONTIER_PLANNER_H

#include "piping/messages.h"
#include "lidar/sensor_control.h"
#include "scene/occupancy_map.h"

/**
 * @brief Build a waypoint path from the current rover pose and occupancy maps.
 *
 * This is the architectural entry point for future frontier selection, target
 * projection, and A* path search. The current implementation is a stub that
 * preserves the data flow without generating a real path yet.
 *
 * @param out_waypoints Destination array for generated waypoints.
 * @param max_waypoints Capacity of @p out_waypoints.
 * @param occupancy_grid_2d Projected 2D navigation map.
 * @param rover_state Latest rover pose snapshot.
 * @return Number of waypoints written to @p out_waypoints.
 */
int plan_frontier_path(Waypoint *out_waypoints,
                       int max_waypoints,
                       const OccupancyMap *occupancy_grid_2d,
                       const SensorState *rover_state);

#endif // FRONTIER_PLANNER_H