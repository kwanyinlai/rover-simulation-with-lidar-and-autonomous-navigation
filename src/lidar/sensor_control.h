

#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H



# include "core/vec3.h"
# include "piping/messages.h"

void init_sensor_state(void);

void get_sensor_pos(Vector3 *pos);

void set_throttle(float value);

float get_throttle(void);

void set_steer(float value);

float get_steer(void);

float get_sensor_dir_angle(void);

float get_sensor_velocity(void);

void rover_control(float dt);

typedef struct {
    Vector3 origin;
    float speed;
    float angular_speed;
    float dir_angle;
} SensorState;

const SensorState *get_sensor_state(void);

#endif // SENSOR_CONTROL_H
