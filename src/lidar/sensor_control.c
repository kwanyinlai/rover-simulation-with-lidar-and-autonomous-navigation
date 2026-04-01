
# include "core/vec3.h"
# include "lidar/raycaster.h"
# include <math.h>
# include <stdlib.h>
# include "scene/point_cloud.h"
# include "scene/occupancy_map.h"
# include "lidar/lidar_sensor.h"

# include "core/physics_constants.h"



static SensorState ss;

static float throttle; // -1, 0 or 1, for backward and forward
static float steer; // -1, 0 or 1, for left and right'

void init_sensor_state(void){
   
    ss.origin = (Vector3){0.0f, 1.0f, 0.0f};

    ss.speed = 0.0f;
    ss.angular_speed = 0.0f;
    ss.dir_angle = 0.0f;

    init_sensor_rays();
}

void get_sensor_pos(Vector3 *pos){
    *pos = ss.origin;
}


void set_throttle(float value){
    throttle = fmaxf(-1.0f, fminf(1.0f, value));
}

float get_throttle(void){
    return throttle;
}

void set_steer(float value){
    steer = fmaxf(-1.0f, fminf(1.0f, value));
}

float get_steer(void){
    return steer;
}

float get_sensor_dir_angle(void) {
    return ss.dir_angle;
}

float get_sensor_velocity(void) {
    return ss.speed;
}

void rover_control(float dt){
    // simple physics for smooth acceleration and turning
    // printf("Throttle: %.2f, Steer: %.2f\n", throttle, steer);

    if (throttle != 0) {
        ss.speed += throttle * ACCELERATION * dt;
        ss.speed = fmaxf(-MAX_SPEED, fminf(MAX_SPEED, ss.speed));
    } else {
        // natural deceleration
        float friction = FRICTION * dt;
        if (ss.speed > 0) {
            ss.speed = fmaxf(0.0f, ss.speed - friction);
        } else {
            ss.speed = fminf(0.0f, ss.speed + friction);
        }
    }
    if (steer != 0.f) {
        ss.angular_speed += steer * ANGULAR_ACCELERATION * dt;
        ss.angular_speed = fmaxf(-MAX_ANGULAR_SPEED, fminf(MAX_ANGULAR_SPEED, ss.angular_speed));
    } else {
        float friction = ANGULAR_FRICTION * dt;
        if (fabsf(ss.angular_speed) < friction) ss.angular_speed = 0.0f;
        else ss.angular_speed -= friction * (ss.angular_speed > 0 ? 1 : -1);
    }

    ss.dir_angle += ss.angular_speed * dt;
    ss.origin.x += cosf(ss.dir_angle) * ss.speed * dt;
    ss.origin.z += sinf(ss.dir_angle) * ss.speed * dt;
    
    // ss.dir_angle += ss.angular_speed * dt; // TODO: rotate lidar as well? maybe we don't want this though
    // // even if it is more physically acurate
}

const SensorState *get_sensor_state(void) {
    return &ss;
}


