
#include "vec3.h"
#include "raycaster.h"
#include <math.h>
#include "point_cloud.h"

#define NUM_RINGS 256
# define MATH_PI 3.14159f
# define MATH_DEG_TO_RAD (MATH_PI / 180.0f)


typedef struct {
    Vector3 origin;
    float theta;
    float elevations[NUM_RINGS]; // measured as angle from X-Z plane, = MATH_PI / 2 - polar
    float min_elev_angle;
    float max_elev_angle;
} SensorState;

static SensorState ss;
static float step = 0.01f;

void init_sensor_state(){
    ss.min_elev_angle = -60.f * MATH_DEG_TO_RAD;
    ss.max_elev_angle = 60.f * MATH_DEG_TO_RAD;
    ss.origin = (Vector3){0.0f, 3.0f, 0.0f};
    for (int i = 0 ; i < NUM_RINGS ; i++){
        ss.elevations[i] = ss.min_elev_angle +
          i * (ss.max_elev_angle - ss.min_elev_angle) / NUM_RINGS;
    }
}

void cast_all_rays(const TriangleArray *scene, PointCloud *point_cloud){
    for (int i = 0 ; i < NUM_RINGS ; i++){
        Vector3 hit;
        // converting theta and elevation to a normalised vector
        float x = cosf(ss.theta) * cosf(ss.elevations[i]); 
        // technically since should be MATH_PI / 2 - polar, but equivalent to converting 
        // from cos to sin, and vice versa.
        float y = sinf(ss.elevations[i]);
        float z = sinf(ss.theta) * cosf(ss.elevations[i]);
        float intensity;
        float dist = cast_ray(scene, &(ss.origin), (Vector3){x, y, z}, &hit, &intensity);
        if (dist > 0) point_cloud_push_back(point_cloud, hit, dist, intensity);
    }
}

void sensor_step(TriangleArray *scene, PointCloud *point_cloud){
    ss.theta += step;
    cast_all_rays(scene, point_cloud);
}

void sensor_move(float forward, float side){
    printf("forward: %f, side: %f\n", forward, side);
    ss.origin = vector3_add(ss.origin, (Vector3){forward, 0.0f, side});
    printf("new x: %f, new y: %f, new z: %f\n", ss.origin.x, ss.origin.y, ss.origin.z);
    fflush(stdout);
}

void get_sensor_pos(Vector3 *pos){
    *pos = ss.origin;
}

