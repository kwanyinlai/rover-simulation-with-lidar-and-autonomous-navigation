
#ifndef MESSAGES_H
#define MESSAGES_H

#include "core/vec3.h"
#define MAX_WAYPOINTS 10


#define MAX_UPDATED_VOXELS 4028
#define NUM_WORKERS 4
#define NUM_RINGS 256

// MPPI hyperparameters
#define MPPI_SAMPLES 32
#define MPPI_HORIZON 24

typedef enum {
    UNKNOWN = 0,
    FREE = 1,
    OCCUPIED = 2
} CELL_STATE;

typedef struct {
    int x, z;
} Waypoint;

typedef struct {
    Waypoint waypoints[MAX_WAYPOINTS];
    int count;
    int current;
} Path;

typedef enum {
    MODE_MANUAL,
    MODE_AUTO
} RoverMode;

typedef struct {
    float x, z;
    float dir_angle;
    float speed;
    float angular_speed;
    int wp_idx;
} SimState;


typedef struct {
    float theta;
    float max_elev;
    float min_elev;
    float num_rings;
    Vector3 origin;
} ScanRequest; // from main to scan coordinator

typedef struct {
    
} PointBatch; // from scan coordinator to occupancy updater 

typedef struct {
    Vector3 origin;
    float theta;
    int start_ray_idx;
    int end_ray_idx;
    int num_rays;
} RayBatch; // from scan coordinator to ray worker

typedef struct {
    float distance;
    float intensity;
    Vector3 hit;
} RayResult;

typedef struct {
    float theta;
    RayResult rays[NUM_RINGS / NUM_WORKERS]; 
    Vector3 origin;
    int count;
} RayResultBatch; // from ray worker to scan coordinator

typedef struct {
    int idx;
    CELL_STATE new_state;
    CELL_STATE prev_state;
} VoxelUpdate;

typedef struct {
    VoxelUpdate updates[MAX_UPDATED_VOXELS];
    int count;
} MapDelta;

typedef struct {
    int is_blocking_count; // OCCUPIED or UNKNOWN count in this column
} ColumnSummary;

typedef struct {
    int frame_id;
    int sample_count;
    int horizon;
    SimState init_state;
    Path path_snapshot;
    float nom_steer[MPPI_HORIZON];
    float nom_throttle[MPPI_HORIZON];
    float steer_noise[MPPI_SAMPLES][MPPI_HORIZON];
    float throttle_noise[MPPI_SAMPLES][MPPI_HORIZON];
} RolloutRequest;

typedef struct {
    int frame_id;
    int start_sample_idx;
    int end_sample_idx;
    RolloutRequest request;
} RolloutJob;

typedef struct {
    int frame_id;
    int start_sample_idx;
    int end_sample_idx;
    float costs[MPPI_SAMPLES];
} RolloutResult;

typedef struct {
    int frame_id;
    float costs[MPPI_SAMPLES];
} BatchedRolloutResult;


#endif // MESSAGES_H