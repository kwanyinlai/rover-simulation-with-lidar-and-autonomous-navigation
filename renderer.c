#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include "vec3.h"
#include "scene.h"
#include "scene_state.h"
#include "point_cloud.h"
#include "lidar_sensor.h"

void render_wire(){
    glLineWidth(0.4f);
    glColor3f(0.15f, 0.15f, 0.18f);
    for(int i = 0 ; i < scene.size; i++){
        Triangle *t = &(scene.data[i]);
        glBegin(GL_LINE_LOOP);
        glVertex3f(t->v0.x, t->v0.y, t->v0.z);
        glVertex3f(t->v1.x, t->v1.y, t->v1.z);
        glVertex3f(t->v2.x, t->v2.y, t->v2.z);
        glEnd();
    }
}

void render_cloud(PointCloud *cloud){
    glBegin(GL_POINTS);
    float y_min = 0.0f;
    float y_max = 6.0f;
    for (size_t i = 0; i < cloud->size; i++) {
        // gradient based on height
        float t = (cloud->data[i].position.y - y_min) / (y_max - y_min);
        float r = t > 0.5f ? (t - 0.5f) * 2.0f : 0.0f;
        float g = t < 0.5f ? t * 2.0f : (1.0f - t) * 2.0f;
        float b = t < 0.5f ? 1.0f - t * 2.0f : 0.0f;
        glColor3f(r * cloud->data[i].intensity, g * cloud->data[i].intensity, b * cloud->data[i].intensity);
        glVertex3f(cloud->data[i].position.x, cloud->data[i].position.y, cloud->data[i].position.z);
    }
    glEnd();
}

void render_sensor(){
    Vector3 sensor_pos;
    get_sensor_pos(&sensor_pos);
    glPushMatrix();
    glTranslatef(sensor_pos.x, sensor_pos.y, sensor_pos.z);
    glColor3f(1.0f, 0.0f, 0.0f); // red for the sensor
    glutSolidSphere(0.5f, 12, 12);
    glPopMatrix();

}