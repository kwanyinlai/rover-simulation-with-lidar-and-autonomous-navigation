// # ifndef MESSAGES_H
// # define MESSAGES_H


# include "vec3.h"

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
    float elev;
} RayTask; // from scan coordinator to ray worker

typedef struct {
    float distance;
    float intensity;
    Vector3 hit_x;
} RayResult; // from ray worker to scan coordinator


// TODO: some frontier message




// # endif // MESSAGES_H