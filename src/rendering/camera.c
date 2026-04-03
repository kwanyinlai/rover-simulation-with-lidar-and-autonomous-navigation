#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <math.h>
#include "core/physics_constants.h"
#include "core/vec3.h"
#include "rendering/scene.h"
#include "rendering/renderer.h"
#include "scene/scene_state.h"
#include "lidar/sensor_control.h"
#include "rover/rover_controller.h"

#define CAMERA_MIN_PHI 2.f
#define CAMERA_MAX_PHI 89.f
#define CAMERA_ROTATE_SENSITIVITY 0.5f
#define CAMERA_PAN_SENSITIVITY 0.05f
#define CAMERA_MIN_Z 0.1f
#define CAM_MIN_ZOOM 1.0f
#define CAM_MAX_ZOOM 100.0f

// ========== Camera Parameters ===========
static float cam_dist = 25.0f;
static float cam_theta = 45.0f;
static float cam_phi = 20.0f;
static float cam_target_x = 0.0f;
static float cam_target_y = 0.0f;
static float cam_target_z = 0.0f;

static int right_down = 0;
static int middle_down = 0;
static int prev_rotate_x, prev_rotate_y;
static int prev_pan_x, prev_pan_y;

// ========================================

int is_render_scene = 1;
int is_paused = 0;
int toggle_occupancy_map_2d = 0;
int toggle_occupancy_map_3d = 0;

static int manual_pause = 0;

static void sync_pause_state(void) {
    is_paused = manual_pause || toggle_occupancy_map_3d;
}

extern RoverMode rover_mode;
extern void handle_sigint(int sig);

void apply_camera(void) {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float theta_rad = cam_theta * MATH_DEG_TO_RAD;
    float phi_rad = cam_phi * MATH_DEG_TO_RAD;
    float cam_x = cam_target_x + cam_dist * cosf(phi_rad) * sinf(theta_rad);
    float cam_y = cam_target_y + cam_dist * sinf(phi_rad);
    float cam_z = cam_target_z + cam_dist * cosf(phi_rad) * cosf(theta_rad);
    gluLookAt(cam_x, cam_y, cam_z,
              cam_target_x, cam_target_y, cam_target_z,
              0, 1, 0);
}

void keyboard(unsigned char key, int x, int y) {
    (void) x; // unused but required by glutKeyboardFunc signature
    (void) y; // unused but required by glutKeyboardFunc signature
    switch (key) {
        case 27:
            handle_sigint(0);
            break;
        case 't':
        case 'T':
            is_render_scene = !is_render_scene;
            break;
        case '=': // '=' is sometimes reported as '=' apparently, so we check for both
        case '+':
            cam_dist -= cam_dist * CAMERA_PAN_SENSITIVITY;
            cam_dist = fmaxf(cam_dist, CAM_MIN_ZOOM); // prevent zooming in too much
            break;
        case '-':
            cam_dist += cam_dist * CAMERA_PAN_SENSITIVITY;
            cam_dist = fminf(cam_dist, CAM_MAX_ZOOM); // prevent zooming out too much
            break;
        case 'p':
        case 'P':
            manual_pause = !manual_pause;
            sync_pause_state();
            break;
        case 'f':
        case 'F':
            toggle_occupancy_map_2d = !toggle_occupancy_map_2d;
            break;
        case 'g':
        case 'G':
            toggle_occupancy_map_3d = !toggle_occupancy_map_3d;
            sync_pause_state();
            break;
        case 'w':
        case 'W':
            set_throttle(1.0f);
            break;
        case 's':
        case 'S':
            set_throttle(-1.0f);
            break;
        case 'a':
        case 'A':
            set_steer(-1.0f);
            break;
        case 'd':
        case 'D':
            set_steer(1.0f);
            break;
        case 'c':
        case 'C':
            if (rover_mode == MODE_MANUAL) {
                rover_mode = MODE_AUTO;
            }
            else {
                rover_mode = MODE_MANUAL;
            }
            // clear stale controls
            set_throttle(0.0f);
            set_steer(0.0f);

    }
    glutPostRedisplay();
}

void keyboard_up(unsigned char key, int x, int y) {
    (void) x; // unused but required by glutKeyboardFunc signature
    (void) y; // unused but required by glutKeyboardFunc signature
    switch(key) {
        case 'w': case 'W':
        case 's': case 'S':
            set_throttle(0.0f);
            break;
        case 'a': case 'A':
        case 'd': case 'D':
            set_steer(0.0f);
            break;
    }
}

void mouse_button(int btn, int state, int x, int y) {
    // R-BTN to rotate, M-BTN to pan
    if (btn == GLUT_RIGHT_BUTTON) {
        right_down = (state == GLUT_DOWN);
        prev_rotate_x = x; prev_rotate_y = y;
    }
    if (btn == GLUT_MIDDLE_BUTTON) {
        middle_down = (state == GLUT_DOWN);
        prev_pan_x = x; prev_pan_y = y;
    }
}

void mouse_move(int x, int y) {
    if (right_down) {
        cam_theta += (x - prev_rotate_x) * CAMERA_ROTATE_SENSITIVITY;
        cam_phi += (y - prev_rotate_y) * CAMERA_ROTATE_SENSITIVITY;

        cam_phi = fmaxf(cam_phi, CAMERA_MIN_PHI); // prevent going below ground plane
        cam_phi = fminf(cam_phi, CAMERA_MAX_PHI); // prevent going above zenith

        prev_rotate_x = x; prev_rotate_y = y;
        glutPostRedisplay();
    }
    if (middle_down) {
        float dx = (x - prev_pan_x) * CAMERA_PAN_SENSITIVITY;
        float dy = (y - prev_pan_y) * CAMERA_PAN_SENSITIVITY;

        float theta_rad = cam_theta * MATH_DEG_TO_RAD;
        cam_target_x -= cosf(theta_rad) * dx;
        cam_target_z += sinf(theta_rad) * dx;

        cam_target_y += dy;
        cam_target_y = fmaxf(cam_target_y, CAMERA_MIN_Z); // prevent going below ground

        prev_pan_x = x; prev_pan_y = y;
        glutPostRedisplay();
    }
}