
#include "vec3.h"
#include "raycaster.h"
#include <math.h>
#include "point_cloud.h"

#define MATH_PI 3.1415

Vector3 origin;
float theta = 0.0f;
const int num_rays = 64;
float elevations[num_rays]; // measured as angle from X-Z plane, = MATH_PI / 2 - polar

float min_elev_angle = -30.0f;
float max_elev_angle = 30.0f;


void init_elevations(){
    for (int i = 0 ; i < num_rays ; i++){
        elevations[i] = min_elev_angle + i * (max_elev_angle - min_elev_angle) / num_rays;
    }
}

void cast_all_rays(const TriangleArray *scene, PointCloud *point_cloud){
    for (int i = 0 ; i < num_rays ; i++){
        Vector3 hit;
        // converting theta and elevation to a normalised vector
        float x = cosf(theta) * cosf(elevations[i]); 
        // technically since should be MATH_PI / 2 - polar, but equivalent to converting 
        // from cos to sin, and vice versa.
        float y = sinf(elevations[i]);
        float z = sinf(theta) * cosf(elevations[i]);
    
        float dist = cast_ray(scene, &origin, (Vector3){x, y, z}, &hit);
        if (dist > 0) point_cloud_push_back(point_cloud, hit, dist);
    }
}

void rotate(){
    theta += 0.01f;
}

