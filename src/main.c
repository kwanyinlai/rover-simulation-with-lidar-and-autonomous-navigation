/*
    * main.c
    *
    *  Created on: 4th March 2026
*/

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <stdio.h>
#include "rendering/vec3.h"
#include "rendering/scene.h"
#include "rendering/scene_state.h"
#include "rendering/camera.h"
#include "rendering/renderer.h"
#include "lidar/lidar_sensor.h"
#include "lidar/point_cloud.h"
#include "lidar/occupancy_map.h"

TriangleArray scene;
PointCloud cloud;
OccupancyMap map;

static float last_time = 0.0f;

extern int is_render_scene;

void display() {
    float current_time = glutGet(GLUT_ELAPSED_TIME) / 1000.0f; 
    float delta_time = current_time - last_time;
    last_time = current_time;
    // BG
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // CAMERA
    apply_camera();

    // RENDER VISUAL ELEMENTS
    if (is_render_scene) render_wire();

    render_sensor();

    sensor_step(&scene, &cloud, &map);
    glDepthMask(GL_FALSE);
    render_cloud(&cloud, delta_time);
    render_occupancy_map(&map);
    glDepthMask(GL_TRUE);

    // SWAP BUFFERS
    glutSwapBuffers();
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)w / h, 0.1, 500.0);
}


int main(int argc, char** argv) {
    init_sensor_state();
    init_point_cloud(&cloud);
    triangle_array_init(&scene);
    build_scene(&scene);
    init_occupancy_map(&map, 300, 60, 240, 0.1f, (Vector3){-15.0f, 0.0f, -12.0f});
    // x from -15 to 15, y from 0 to 6, z from -12 to 12, with 0.1m resolution, gives us a 300x60x240 grid
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(768, 768);
    glutCreateWindow("LIDAR Simulator");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // display, reshape, keyboard, mouse-pressed and mouse-move callbacks
    glutDisplayFunc(display); 
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse_button);
    glutMotionFunc(mouse_move);

    printf("mouse drag to orbit, +/- to zoom\n");

    glutMainLoop();
    return 0;
}