

#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H



#include "core/vec3.h"
#include "piping/messages.h"

/**
 * @brief Initialize rover sensor state and ray configuration.
 */
void init_sensor_state(void);

/**
 * @brief Get the current sensor position in world coordinates. (ground truth)
 * @param pos Output position vector.
 */
void get_sensor_pos(Vector3 *pos);

/**
 * @brief Set throttle command.
 * @param value Command in range [-1, 1].
 */
void set_throttle(float value);

/**
 * @brief Get current throttle command.
 * @return Current throttle in range [-1, 1].
 */
float get_throttle(void);

/**
 * @brief Set steering command.
 * @param value Command in range [-1, 1].
 */
void set_steer(float value);

/**
 * @brief Get current steering command.
 * @return Current steer in range [-1, 1].
 */
float get_steer(void);

/**
 * @brief Get current sensor heading angle.
 * @return Heading angle in radians.
 */
float get_sensor_dir_angle(void);

/**
 * @brief Get current sensor linear velocity.
 * @return Linear velocity in m/s.
 */
float get_sensor_velocity(void);

/**
 * @brief Advance rover/sensor physics by one time step.
 * @param dt Simulation time step in seconds.
 */
void rover_control(float dt);

typedef struct {
    Vector3 origin;
    float speed;
    float angular_speed;
    float dir_angle;
} SensorState;

/**
 * @brief Get immutable pointer to current sensor state.
 * @return Pointer to internal sensor state.
 */
const SensorState *get_sensor_state(void);

#endif // SENSOR_CONTROL_H
