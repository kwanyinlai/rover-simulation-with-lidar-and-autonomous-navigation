
/**
 * @file camera.h
 * @brief Camera and input handling for rendering.
 *
 * Provides functions for camera control, user input, and display callbacks in the rendering system.
 */

#ifndef CAMERA_H
#define CAMERA_H

/**
 * @brief Apply the current camera transformation.
 */
void apply_camera();

/**
 * @brief Mouse button event handler.
 */
void mouse_button(int button, int state, int x, int y);

/**
 * @brief Mouse movement event handler.
 */
void mouse_move(int x, int y);

/**
 * @brief Keyboard event handler.
 */
void keyboard(unsigned char key, int x, int y);

/**
 * @brief Keyboard key-release event handler.
 */
void keyboard_up(unsigned char key, int x, int y);

/**
 * @brief Display callback for rendering.
 */
void display();

/**
 * @brief Window reshape event handler.
 */
void reshape(int width, int height);

#endif // CAMERA_H