
# include "core/vec3.h"
# include "lidar/raycaster.h"
# include <math.h>
# include <stdlib.h>
# include "scene/point_cloud.h"
# include "scene/occupancy_map.h"
# include "scene/scene_collision.h"
# include "lidar/lidar_sensor.h"
# include "rover/rover_physics.h"



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
    step_rover_physics(&ss.origin.x,
                       &ss.origin.z,
                       &ss.dir_angle,
                       &ss.speed,
                       &ss.angular_speed,
                       throttle,
                       steer,
                       dt,
                       &scene,
                       ROVER_COLLISION_RADIUS);
    
    // ss.dir_angle += ss.angular_speed * dt; // TODO: rotate lidar as well? maybe we don't want this though
    // // even if it is more physically acurate
}

const SensorState *get_sensor_state(void) {
    return &ss;
}


