#include "frontier_exploration/frontier_planner.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <float.h>
#include "scene/scene_collision.h"

#define MAX_CLUSTERS 1024
#define CLUSTER_MIN_SIZE 10
#define FRONTIER_DIST_WEIGHT 0.5f
#define ROVER_HALO_CELLS 40
#define RDP_EPSILON 0.4f
#define MAX_PATH_BUF 8192
#define PATH_CLEARANCE_MARGIN 0.10f

typedef struct {
    int width;
    int depth;
    int *distance;
} ReachabilityGrid;

typedef struct {
    Vector3 target_world;
    int has_target;
} FrontierTarget;

typedef struct {
    float curr_x;
    float curr_z;
    int size;
} FrontierCluster;


static int is_passable(const OccupancyMap *m, int x, int z) {
    if (!occupancy_map_in_bounds(m, x, 0, z)) return 0;

    CELL_STATE s = occupancy_map_get_cell(m, x, 0, z);
    if (s != FREE) return 0;

    // check clearance around the cell_size that rover can fit
    int clearance_cells = (int)ceilf((ROVER_COLLISION_RADIUS + PATH_CLEARANCE_MARGIN) / m->cell_size);

    for (int line_seg_z_dir = -clearance_cells; line_seg_z_dir <= clearance_cells; line_seg_z_dir++) 
    for (int line_seg_x_dir = -clearance_cells; line_seg_x_dir <= clearance_cells; line_seg_x_dir++) {
        // skip checking cells that are too far to affect clearance since we are checking
        // a circular area
        if (line_seg_x_dir * line_seg_x_dir + line_seg_z_dir * line_seg_z_dir > clearance_cells * clearance_cells) continue;
        int clearance_x = x + line_seg_x_dir;
        int clearance_z = z + line_seg_z_dir;
        if (!occupancy_map_in_bounds(m, clearance_x, 0, clearance_z)) return 0;
        // we'll be conservative and treat UNKNOWN as non-occupied
        if (occupancy_map_get_cell(m, clearance_x, 0, clearance_z) == OCCUPIED) return 0;
    }

    return 1;
}


static int init_reachability_grid(const OccupancyMap *grid,
                                  ReachabilityGrid *out_grid)
{
    int total = grid->width * grid->depth;

    out_grid->width = grid->width;
    out_grid->depth = grid->depth;
    out_grid->distance = malloc(total * sizeof(int));
    if (!out_grid->distance) return 0;

    for (int i = 0; i < total; i++) {
        out_grid->distance[i] = -1;
    }

    return 1;
}




// Seed a start point using BFS to find nearest reachable cell from rover position
static int seed_start_bfs(const OccupancyMap *grid,
                          int start_x,
                          int start_z,
                          int *out_seed_x,
                          int *out_seed_z)
{
    static const int line_seg_x_dir[4] = {1, -1, 0, 0};
    static const int line_seg_z_dir[4] = {0, 0, 1, -1};

    int total = grid->width * grid->depth;

    if (is_passable(grid, start_x, start_z)) {
        *out_seed_x = start_x;
        *out_seed_z = start_z;
        return 1;
    }
    // TODO:
    int *visited = calloc(total, sizeof(int));
    int *queue = malloc(total * sizeof(int));

    if (!visited || !queue) {
        free(visited);
        free(queue);
        return 0;
    }

    int head = 0;
    int tail = 0;

    int start_idx = start_z * grid->width + start_x;
    visited[start_idx] = 1;
    queue[tail++] = start_idx;

    while (head < tail) {
        int idx = queue[head++];
        int x = idx % grid->width;
        int z = idx / grid->width;

        if (occupancy_map_get_cell(grid, x, 0, z) == FREE &&
            is_passable(grid, x, z))
        {
            *out_seed_x = x;
            *out_seed_z = z;

            free(visited);
            free(queue);
            return 1;
        }

        CELL_STATE state = occupancy_map_get_cell(grid, x, 0, z);
        if (state == OCCUPIED) continue;

        int dx = x - start_x;
        int dz = z - start_z;
        if (dx * dx + dz * dz > ROVER_HALO_CELLS * ROVER_HALO_CELLS) {
            continue;
        }

        for (int i = 0; i < 4; i++) {
            int neighbour_x = x + line_seg_x_dir[i];
            int neighbour_z = z + line_seg_z_dir[i];

            if (!occupancy_map_in_bounds(grid, neighbour_x, 0, neighbour_z)) continue;

            int neighbour_idx = neighbour_z * grid->width + neighbour_x;
            if (visited[neighbour_idx]) continue;

            visited[neighbour_idx] = 1;
            queue[tail++] = neighbour_idx;
        }
    }

    free(visited);
    free(queue);
    return 0;
}

static int run_distance_bfs(const OccupancyMap *grid,
                            int seed_x,
                            int seed_z,
                            ReachabilityGrid *out_grid)
{
    static const int line_seg_x_dir[4] = {1, -1, 0, 0};
    static const int line_seg_z_dir[4] = {0, 0, 1, -1};

    int *queue = malloc(grid->width * grid->depth * sizeof(int));
    if (!queue) return 0;

    int head = 0;
    int tail = 0;

    int seed_idx = seed_z * grid->width + seed_x;
    out_grid->distance[seed_idx] = 0;
    queue[tail++] = seed_idx;

    while (head < tail) {
        int idx = queue[head++];
        int x = idx % grid->width;
        int z = idx / grid->width;

        for (int i = 0; i < 4; i++) {
            int neighbour_x = x + line_seg_x_dir[i];
            int neighbour_z = z + line_seg_z_dir[i];

            if (!occupancy_map_in_bounds(grid, neighbour_x, 0, neighbour_z)) continue;
            if (!is_passable(grid, neighbour_x, neighbour_z)) continue;

            int neighbour_idx = neighbour_z * grid->width + neighbour_x;
            if (out_grid->distance[neighbour_idx] != -1) continue;

            out_grid->distance[neighbour_idx] = out_grid->distance[idx] + 1;
            queue[tail++] = neighbour_idx;
        }
    }

    free(queue);
    return 1;
}

static int build_reachability_grid(const OccupancyMap *grid,
                                   const Waypoint *start,
                                   ReachabilityGrid *out_grid,
                                   Waypoint *out_seed)
{
    int start_x, start_z;
    int seed_x, seed_z;

    if (!init_reachability_grid(grid, out_grid)) return 0;


    int x = (int)floorf((start->x - grid->origin.x) / grid->cell_size);
    int z = (int)floorf((start->z - grid->origin.z) / grid->cell_size);

    if (!occupancy_map_in_bounds(grid, x, 0, z)) {
        free(out_grid->distance);
        out_grid->distance = NULL;
        return 0;
    }

    start_x = x;
    start_z = z;
    
    if (!seed_start_bfs(grid, start_x, start_z, &seed_x, &seed_z)) {
        free(out_grid->distance);
        out_grid->distance = NULL;
        return 0;
    }

    if (out_seed) {
        out_seed->x = grid->origin.x + (seed_x + 0.5f) * grid->cell_size;
        out_seed->z = grid->origin.z + (seed_z + 0.5f) * grid->cell_size;
    }

    if (!run_distance_bfs(grid, seed_x, seed_z, out_grid)) {
        free(out_grid->distance);
        out_grid->distance = NULL;
        return 0;
    }

    return 1;
}



/* ── Anchor projection ───────────────────────────────────────────────────── */

static int find_reachable_anchor_for_target(const OccupancyMap *occupancy_grid_2d,
                                            const ReachabilityGrid *reachability,
                                            const Vector3 *target_world,
                                            Waypoint *out_waypoint,
                                            int *out_distance) {

    int total = occupancy_grid_2d->width * occupancy_grid_2d->depth;
    float cell_size = occupancy_grid_2d->cell_size;

    int grid_x = floorf((target_world->x - occupancy_grid_2d->origin.x) / cell_size);
    int grid_z = floorf((target_world->z - occupancy_grid_2d->origin.z) / cell_size);

    if (!occupancy_map_in_bounds(occupancy_grid_2d, grid_x, 0, grid_z)) return 0;

    int *visited = calloc(total, sizeof(int));
    int *queue = malloc(total * sizeof(int));

    static const int line_seg_x_dir[4] = {1, -1, 0, 0};
    static const int line_seg_z_dir[4] = {0, 0, 1, -1};

    int head = 0;
    int tail = 0;
    int start_idx = grid_z * occupancy_grid_2d->width + grid_x;
    visited[start_idx] = 1;

    queue[tail++] = start_idx;

    while (head < tail) {
        int idx = queue[head++];
        int x = idx % occupancy_grid_2d->width;
        int z = idx / occupancy_grid_2d->width;
        CELL_STATE cell_state = occupancy_map_get_cell(occupancy_grid_2d, x, 0, z);

        if (cell_state == FREE &&
            reachability->distance[idx] >= 0 &&
            is_passable(occupancy_grid_2d, x, z)) 
        {
            out_waypoint->x = occupancy_grid_2d->origin.x + (x + 0.5f) * cell_size;
            out_waypoint->z = occupancy_grid_2d->origin.z + (z + 0.5f) * cell_size;
            if (out_distance) *out_distance = reachability->distance[idx];
            free(visited);
            free(queue);
            return 1;
        }
        // skip blocked cells during BFS
        if (cell_state == OCCUPIED) continue;

        for (int i = 0; i < 4; i++) {
            int neighbour_x = x + line_seg_x_dir[i], neighbour_z = z + line_seg_z_dir[i];
            if (!occupancy_map_in_bounds(occupancy_grid_2d, neighbour_x, 0, neighbour_z)) continue;
            int neighbour_idx = neighbour_z * occupancy_grid_2d->width + neighbour_x;
            if (visited[neighbour_idx]) continue;
            if (occupancy_map_get_cell(occupancy_grid_2d, neighbour_x, 0, neighbour_z) == OCCUPIED) continue;
            visited[neighbour_idx] = 1;
            queue[tail++] = neighbour_idx;
        }
    }

    free(visited);
    free(queue);
    return 0;
}

static int is_frontier_cell_2d(const OccupancyMap *occupancy_grid_2d, int x, int z) {
    
    if (!is_passable(occupancy_grid_2d, x, z)) return 0;

    static const int line_seg_x_dir[4] = {1, -1, 0, 0};
    static const int line_seg_z_dir[4] = {0, 0, 1, -1};
    for (int i = 0; i < 4; i++) {
        int neighbour_x = x + line_seg_x_dir[i];
        int neighbour_z = z + line_seg_z_dir[i];
        if (!occupancy_map_in_bounds(occupancy_grid_2d, neighbour_x, 0, neighbour_z)) continue;
        if (occupancy_map_get_cell(occupancy_grid_2d, neighbour_x, 0, neighbour_z) == UNKNOWN) return 1;
    }
    return 0;
}

static int grow_frontier_cluster(const OccupancyMap *grid,
                                 int *visited,
                                 int seed_idx,
                                 int *queue,
                                 FrontierCluster *out_cluster)
{
    static const int line_seg_x_dir[4] = {1, -1, 0, 0};
    static const int line_seg_z_dir[4] = {0, 0, 1, -1};

    float sum_x = 0.0f;
    float sum_z = 0.0f;
    int size = 0;

    int head = 0;
    int tail = 0;

    queue[tail++] = seed_idx;
    visited[seed_idx] = 1;

    while (head < tail) {
        int idx = queue[head++];
        int x = idx % grid->width;
        int z = idx / grid->width;

        sum_x += (float)x;
        sum_z += (float)z;
        size++;

        for (int i = 0; i < 4; i++) {
            int nx = x + line_seg_x_dir[i];
            int nz = z + line_seg_z_dir[i];

            if (!occupancy_map_in_bounds(grid, nx, 0, nz)) continue;

            int neighbour_idx = nz * grid->width + nx;

            if (visited[neighbour_idx]) continue;
            if (!is_frontier_cell_2d(grid, nx, nz)) continue;

            visited[neighbour_idx] = 1;
            queue[tail++] = neighbour_idx;
        }
    }

    if (size < CLUSTER_MIN_SIZE) {
        return 0;
    }

    out_cluster->curr_x = grid->origin.x +
                      (sum_x / (float)size + 0.5f) * grid->cell_size;
    out_cluster->curr_z = grid->origin.z +
                      (sum_z / (float)size + 0.5f) * grid->cell_size;
    out_cluster->size = size;

    return 1;
}

static int collect_frontier_clusters(const OccupancyMap *grid,
                                     FrontierCluster *clusters,
                                     int max_clusters)
{
    int total = grid->width * grid->depth;

    int *visited = calloc((size_t)total, sizeof(int));
    int *queue = malloc((size_t)total * sizeof(int));

    int cluster_count = 0;

    for (int z = 0; z < grid->depth && cluster_count < max_clusters; z++) {
        for (int x = 0; x < grid->width && cluster_count < max_clusters; x++) {
            int idx = z * grid->width + x;

            if (visited[idx]) continue;
            if (!is_frontier_cell_2d(grid, x, z)) continue;

            FrontierCluster cluster;

            if (grow_frontier_cluster(grid,
                                      visited,
                                      idx,
                                      queue,
                                      &cluster))
            {
                clusters[cluster_count++] = cluster;
            }
        }
    }

    free(visited);
    free(queue);

    return cluster_count;
}

static FrontierTarget select_frontier_target(const OccupancyMap *grid,
                                             const SensorState *rover_state,
                                             const ReachabilityGrid *reachability)
{

    (void)rover_state;


    FrontierTarget result = {
        .target_world = {0},
        .has_target = 0
    };


    FrontierCluster *clusters =
        malloc((size_t)MAX_CLUSTERS * sizeof(FrontierCluster));


    int cluster_count = collect_frontier_clusters(grid,
                                                  clusters,
                                                  MAX_CLUSTERS);

    float best_score = -FLT_MAX;
    // score clusters
    for (int i = 0; i < cluster_count; i++) {
        Vector3 cluster_target = {
            clusters[i].curr_x,
            0.0f,
            clusters[i].curr_z
        };

        Waypoint anchor = {0};
        int path_distance = -1;

        if (!find_reachable_anchor_for_target(grid,
                                              reachability,
                                              &cluster_target,
                                              &anchor,
                                              &path_distance))
        {
            continue;
        }

        float score = (float)clusters[i].size
                    - FRONTIER_DIST_WEIGHT * (float)path_distance;

        if (score > best_score) {
            best_score = score;
            result.target_world = (Vector3){
                anchor.x,
                0.0f,
                anchor.z
            };
            result.has_target = 1;
        }
    }

    free(clusters);
    return result;
}


static int find_path(const OccupancyMap *occupancy_grid_2d,
                     const Waypoint *start,
                     const Waypoint *goal,
                     Waypoint *out_waypoints,
                     int max_waypoints) {
  
    int total = occupancy_grid_2d->width * occupancy_grid_2d->depth;
    float cell_size = occupancy_grid_2d->cell_size;

    int start_x = floorf((start->x - occupancy_grid_2d->origin.x) / cell_size);
    int start_z = floorf((start->z - occupancy_grid_2d->origin.z) / cell_size);
    int goal_x = floorf((goal->x - occupancy_grid_2d->origin.x) / cell_size);
    int goal_z = floorf((goal->z - occupancy_grid_2d->origin.z) / cell_size);


    int *came_from = malloc(total * sizeof(int));
    int *queue = malloc(total * sizeof(int));

    for (int i = 0; i < total; i++) came_from[i] = -1;

    int start_idx = start_z * occupancy_grid_2d->width + start_x;
    int goal_idx = goal_z * occupancy_grid_2d->width + goal_x;

    int head = 0, tail = 0;
    queue[tail++] = start_idx;
    came_from[start_idx] = start_idx;

    static const int dir_x[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
    static const int dir_z[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };

    int found = 0;
    while (head < tail && !found) {
        int curr = queue[head++];
        if (curr == goal_idx) { found = 1; break; }
        int curr_x = curr % occupancy_grid_2d->width;
        int curr_z = curr / occupancy_grid_2d->width;
        for (int i = 0; i < 8; i++) {
            int neighbour_x = curr_x + dir_x[i], neighbour_z = curr_z + dir_z[i];
            if (!is_passable(occupancy_grid_2d, neighbour_x, neighbour_z)) continue;
            int neighbour_idx = neighbour_z * occupancy_grid_2d->width + neighbour_x;
            if (came_from[neighbour_idx] != -1) continue;
            came_from[neighbour_idx] = curr;
            queue[tail++]   = neighbour_idx;
        }
    }

    int count = 0;
    if (found) {
        int *reverse_path = malloc(total * sizeof(int));
        if (reverse_path) {
            int len = 0;
            for (int curr = goal_idx; curr != start_idx; curr = came_from[curr])
                reverse_path[len++] = curr;
            reverse_path[len++] = start_idx;

            count = len < max_waypoints ? len : max_waypoints;
            for (int i = 0; i < count; i++) {
                int pidx = reverse_path[len - 1 - i];
                out_waypoints[i].x = occupancy_grid_2d->origin.x + (float)(pidx % occupancy_grid_2d->width) * cell_size + cell_size * 0.5f;
                out_waypoints[i].z = occupancy_grid_2d->origin.z + (float)(pidx / occupancy_grid_2d->width) * cell_size + cell_size * 0.5f;
            }
            free(reverse_path);
        }
    }

    free(came_from);
    free(queue);
    return count;
}


static float perpendicular_distance(Waypoint p, Waypoint a, Waypoint b) {
    float line_seg_x_dir = b.x - a.x;
    float line_seg_z_dir = b.z - a.z;
    // len_sqr = |b-a|^2
    float len_sqr = line_seg_x_dir * line_seg_x_dir + line_seg_z_dir * line_seg_z_dir;
    
    // normalised projection of p onto ab
    float t  = ((p.x - a.x) * line_seg_x_dir + (p.z - a.z) * line_seg_z_dir) / len_sqr;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t); // clamp
    float closest_x = a.x + t * line_seg_x_dir;
    float closest_z = a.z + t * line_seg_z_dir;
    return sqrtf(
        (p.x - closest_x)*(p.x - closest_x) + 
        (p.z - closest_z)*(p.z - closest_z)
    );
}

// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
static int compress_waypoints(const Waypoint *input, int start, int end,
                              Waypoint *output, int max_out)
{
    float dmax = 0.0f;
    int index = start;

    // Find point with max distance
    for (int i = start + 1; i < end; i++) {
        float d = perpendicular_distance(input[i], input[start], input[end]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > RDP_EPSILON) {
        int recResults1 = compress_waypoints(input, start, index, output, max_out);
        int recResults2 = compress_waypoints(input, index, end,
                                             output + recResults1,
                                             max_out - recResults1);

        return recResults1 + recResults2;
    } else {
        output[0] = input[start];
        output[1] = input[end];
        return 2;
    }
}


int plan_frontier_path(Waypoint *out_waypoints,
                       int max_waypoints,
                       const OccupancyMap *occupancy_grid_2d,
                       const SensorState *rover_state) {



    Waypoint rover_start = { rover_state->origin.x, rover_state->origin.z };
    Waypoint seeded_start = {0};

    ReachabilityGrid reachability = {0};
    if (!build_reachability_grid(occupancy_grid_2d, &rover_start, &reachability, &seeded_start)) {
        fprintf(stderr, "Failed to build reachability grid\n");
        return 0;
    }

    FrontierTarget target = select_frontier_target(occupancy_grid_2d,
                                                   rover_state,
                                                   &reachability);
    if (!target.has_target) {
        fprintf(stderr, "No frontier target selected\n");
        free(reachability.distance);
        reachability.distance = NULL;
        reachability.width = 0;
        reachability.depth = 0;
        return 0;
    }

    Waypoint projected_goal = { target.target_world.x, target.target_world.z };

    Waypoint *coarse_path = malloc(MAX_PATH_BUF * sizeof(Waypoint));


    int coarse_count = find_path(occupancy_grid_2d,
                                 &seeded_start,
                                 &projected_goal,
                                 coarse_path,
                                 MAX_PATH_BUF);
    if (coarse_count <= 0) {
        fprintf(stderr,
                "BFS failed. start=(%.2f, %.2f) goal=(%.2f, %.2f)\n",
                seeded_start.x, seeded_start.z,
                projected_goal.x, projected_goal.z);
        free(coarse_path);
        free(reachability.distance);
        reachability.distance = NULL;
        reachability.width = 0;
        reachability.depth = 0;
        return 0;
    }

    int final_count = compress_waypoints(coarse_path, 0, coarse_count - 1,
                                         out_waypoints, max_waypoints);
    free(coarse_path);
    free(reachability.distance);
    reachability.distance = NULL;
    reachability.width = 0;
    reachability.depth = 0;
    return final_count > 0 ? final_count : 0;
}