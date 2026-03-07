#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include "scene.h"
#include "point_cloud.h"

void init_sensor_state();
void sensor_step(const TriangleArray *scene, PointCloud *point_cloud);
void sensor_move(float forward, float backward);
void get_sensor_pos(Vector3 *pos);

#endif  // LIDAR_SENSOR_H