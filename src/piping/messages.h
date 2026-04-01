
#ifndef MESSAGES_H
#define MESSAGES_H

# include "core/vec3.h"


# define MAX_UPDATED_VOXELS 4028
# define NUM_WORKERS 4
# define NUM_RINGS 256

typedef enum {
    UNKNOWN = 0,
    FREE = 1,
    OCCUPIED = 2
} CELL_STATE;
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
    RayResult rays[NUM_RINGS / NUM_WORKERS]; // assuming NUM_RINGS is divisible by NUM_WORKERS
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


// TODO: some frontier message




#endif // MESSAGES_H