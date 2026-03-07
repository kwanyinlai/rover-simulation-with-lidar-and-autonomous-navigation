
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include <math.h>
#include "vec3.h"
#include "scene.h"
#include "renderer.h"
#include "scene_state.h"
#include "lidar_sensor.h"

# define MATH_PI 3.14159f
# define MATH_DEG_TO_RAD (MATH_PI / 180.0f)

// ========== Camera Parameters ===========
static float cam_dist = 25.0f;
static float cam_theta = 45.0f; // azimuthal angle around Y axis, in degrees
static float cam_phi = 20.0f; // polar angle from Y axis, in degrees
static float cam_target_x = 0.0f;
static float cam_target_y = 0.0f;
static float cam_target_z = 0.0f;
static int prev_x, prev_y, mouse_down = 0;
// ========================================

int is_render_scene = 1.0;


void apply_camera(){
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    float theta_rad=cam_theta*(float)M_PI/180.f, phi_rad=cam_phi*(float)M_PI/180.f;
    float cam_x = cam_target_x + cam_dist * cosf(phi_rad) * sinf(theta_rad);
    float cam_y = cam_target_y + cam_dist * sinf(phi_rad);
    float cam_z = cam_target_z + cam_dist * cosf(phi_rad) * cosf(theta_rad);
    gluLookAt(cam_x, cam_y, cam_z,
              cam_target_x, cam_target_y, cam_target_z,
              0,1,0);
}

void keyboard(unsigned char key, int x, int y) {
    float cam_step = 0.3f;
    float sensor_step = .5f;
    float theta_rad = cam_theta * MATH_DEG_TO_RAD;
    switch (key) {
        case 27: 
            exit(0); 
        case '=': // + and = occasionally mapped together, check both
        case '+': 
            cam_dist -= 1.0f; 
            break;
        case '-': 
            cam_dist += 1.0f; 
            break;
        case 't':
            is_render_scene = !(is_render_scene);
            break;
        case 'w':
            cam_target_x -= sinf(theta_rad) * cam_step;
            cam_target_z -= cosf(theta_rad) * cam_step;
            break;
        case 's':
            cam_target_x += sinf(theta_rad) * cam_step;
            cam_target_z += cosf(theta_rad) * cam_step;
            break;
        case 'a':
            cam_target_x -= cosf(theta_rad) * cam_step;
            cam_target_z += sinf(theta_rad) * cam_step;
            break;
        case 'd':
            cam_target_x += cosf(theta_rad) * cam_step;
            cam_target_z -= sinf(theta_rad) * cam_step;
            break;
        case 'i':
            printf("movement forward should be %f", sinf(theta_rad) * sensor_step);
            sensor_move(-sinf(theta_rad) * sensor_step, -cosf(theta_rad) * sensor_step);
            break;
        case 'k':
            sensor_move(sinf(theta_rad) * sensor_step, cosf(theta_rad) * sensor_step);
            break;
        case 'j':
            sensor_move(-cosf(theta_rad) * sensor_step, sinf(theta_rad) * sensor_step);
            break;
        case 'l':
            sensor_move(cosf(theta_rad) * sensor_step, -sinf(theta_rad) * sensor_step);
            break;
    }
    glutPostRedisplay();
}

void mouse_button(int btn, int state, int x, int y) {
    mouse_down = (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
    prev_x = x;
    prev_y = y;
}

void mouse_move(int x, int y) {
    if (!mouse_down) return;
    cam_theta   += (x - prev_x) * 0.5f;
    cam_phi += (y - prev_y) * 0.5f;
    if (cam_phi> 89.0f) cam_phi = 89.0f;
    if (cam_phi <  2.0f) cam_phi =  2.0f;
    prev_x = x;
    prev_y = y;
    glutPostRedisplay();
}