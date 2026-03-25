


#include "rendering/vec3.h"
#include "lidar/raycaster.h"
#include <math.h>
#include <stdlib.h>
#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"



// Generate a standard normal (mean=0, stddev=1) random value using the Box-Muller transform.
// @see https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
static float gaussian_noise() {
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f); // avoid log(0)
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159f * u2);
    // TODO: hard coded PI, if needed more, then define a constant for it.
}

#define NUM_RINGS 1024
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
    ss.min_elev_angle = -80.f * MATH_DEG_TO_RAD;
    ss.max_elev_angle = 80.f * MATH_DEG_TO_RAD;
    ss.origin = (Vector3){0.0f, 3.0f, 0.0f};
    for (int i = 0 ; i < NUM_RINGS ; i++){
        ss.elevations[i] = ss.min_elev_angle +
          i * (ss.max_elev_angle - ss.min_elev_angle) / NUM_RINGS;
    }
}

void cast_all_rays(const TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *map){
    const float noise_factor = 0.01f; // realistic noise strength (2% of distance)
    for (int i = 0 ; i < NUM_RINGS ; i++){
        Vector3 hit;
        // converting theta and elevation to a normalised vector
        float x = cosf(ss.theta) * cosf(ss.elevations[i]); 
        // technically since should be MATH_PI / 2 - polar, but equivalent to converting 
        // from cos to sin, and vice versa.
        float y = sinf(ss.elevations[i]);
        float z = sinf(ss.theta) * cosf(ss.elevations[i]);
        Vector3 dir = {x, y, z};
        float intensity;
        float dist = cast_ray(scene, &(ss.origin), dir, &hit, &intensity);
        occupancy_map_ray_cast(map, ss.origin, hit, dist > 0.0f);
        if (dist > 0) {
            float noise = gaussian_noise();
            float noisy_dist = fmax(dist + noise * (noise_factor * dist), 0.01f); // clamp to avoid negative or zero distance
            Vector3 noisy_pos = {
                ss.origin.x + dir.x * noisy_dist,
                ss.origin.y + dir.y * noisy_dist,
                ss.origin.z + dir.z * noisy_dist
            };
            point_cloud_push_back(point_cloud, noisy_pos, noisy_dist, intensity);
        }
    }
}

void sensor_step(TriangleArray *scene, PointCloud *point_cloud, OccupancyMap *map){
    ss.theta += step;
    cast_all_rays(scene, point_cloud, map);
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

