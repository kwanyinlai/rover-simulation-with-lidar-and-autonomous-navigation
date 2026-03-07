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
#include "vec3.h"
#include "scene.h"
#include "scene_state.h"
#include "camera.h"
#include "renderer.h"

TriangleArray scene;


int main(int argc, char** argv) {

    triangle_array_init(&scene);
    build_scene(&scene);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(768, 768);
    glutCreateWindow("LIDAR Simulator");

    glEnable(GL_DEPTH_TEST);

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