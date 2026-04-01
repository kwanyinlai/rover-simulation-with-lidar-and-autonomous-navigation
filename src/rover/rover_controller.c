# ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#   include <GL/glew.h>
#   include <GL/glut.h>
# endif

# include "rover_controller.h"
# include "lidar/sensor_control.h"
# include "core/physics_constants.h"
# include "core/noise.h"
# include "core/math_utils.h"

# include <math.h>
# include <stdio.h>
# include <float.h>


# define WAYPOINT_REACH_DIST 0.4f
# define WAYPOINT_REACH_THRESHOLD (WAYPOINT_REACH_DIST * WAYPOINT_REACH_DIST)

# define STEER_CLAMP_MIN -1.0f
# define STEER_CLAMP_MAX 1.0f
# define ERROR_FACTOR_MAX 1.0f
# define ERROR_FACTOR_MIN 0.0f
# define THROTTLE_BASE 0.3f
# define THROTTLE_SCALE 0.7f
# define SLOW_DIST_SQR 2.25f

# define PREDICT_THROTTLE_BASE 0.05f
# define PREDICT_THROTTLE_SCALE 0.95f
# define PREDICT_SLOW_DIST_SQR 1.5f

# define SPEED_NOISE 0.f // no noise for now while testing
# define ANGULAR_NOISE 0.f



# define LOOK_AHEAD_STEPS 20
# define LOOK_AHEAD_DT 0.05f

# define PREDICT_BLEND_WEIGHT 0.4f

# define PREDICTION_STEPS  60
# define PREDICTION_DT 0.05f // 60 x 0.05 = 3s prediction


SensorState rover_pose  = {0}; // 
Path active_path = {0};
RoverMode rover_mode = MODE_AUTO;


static void sim_control(const SimState *sim_state,
                        float *throttle, float *steer,
                        int use_predictive_gains) {
    if (sim_state->wp_idx >= active_path.count) { *throttle = 0.0f; *steer = 0.0f; return; }

    Waypoint *goal = &active_path.waypoints[sim_state->wp_idx];
    float dx = goal->x - sim_state->x;
    float dz = goal->z - sim_state->z;
    float dist_sqr = dx*dx + dz*dz;

    float desired = atan2f(dz, dx);
    float error = wrap_angle(desired - sim_state->dir_angle);

    // PD controller mechanics
    float p_steer = KP * error - KD * sim_state->ang_speed;
    p_steer = fmaxf(STEER_CLAMP_MIN, fminf(STEER_CLAMP_MAX, p_steer));

    float error_factor = ERROR_FACTOR_MAX - fminf(ERROR_FACTOR_MAX, fabsf(error) / (float)M_PI);
    float throttle_base = use_predictive_gains ? PREDICT_THROTTLE_BASE  : THROTTLE_BASE;
    float throttle_scale = use_predictive_gains ? PREDICT_THROTTLE_SCALE : THROTTLE_SCALE;
    float slow_dist_sqr = use_predictive_gains ? PREDICT_SLOW_DIST_SQR  : SLOW_DIST_SQR;
    float p_throttle  = throttle_base + throttle_scale * (error_factor * error_factor);
    if (dist_sqr < slow_dist_sqr) p_throttle *= (dist_sqr / slow_dist_sqr);

    *throttle = p_throttle;
    *steer = p_steer;
}

// same physics as sensor_control.c for accurate prediction, but without noise
static void sim_step(SimState *sim_state, float throttle, float steer, float dt) {

    if (throttle != 0.0f) {
        sim_state->speed += throttle * ACCELERATION * dt;
    }
    else {
        float f = FRICTION * dt;
        if (sim_state->speed > 0.0f){ 
            sim_state->speed = fmaxf(0.0f, sim_state->speed - f);
        }
        else {
            sim_state->speed = fminf(0.0f, sim_state->speed + f);
        }
    }
    
    sim_state->speed = fmaxf(-MAX_SPEED, fminf(MAX_SPEED, sim_state->speed));

    if (steer != 0.0f) {
        sim_state->ang_speed += steer * ANGULAR_ACCELERATION * dt;
    }
    else {
        float friction_amt = ANGULAR_FRICTION * dt;
        if (fabsf(rover_pose.angular_speed) < friction_amt) {
            rover_pose.angular_speed = 0.0f;
        } else {
            rover_pose.angular_speed -= friction_amt * (rover_pose.angular_speed > 0.0f ? 1.0f : -1.0f);
        }
    }
    sim_state->ang_speed = fmaxf(-MAX_ANGULAR_SPEED, fminf(MAX_ANGULAR_SPEED, sim_state->ang_speed));

    sim_state->dir_angle += sim_state->ang_speed * dt;
    sim_state->x += cosf(sim_state->dir_angle) * sim_state->speed * dt;
    sim_state->z += sinf(sim_state->dir_angle) * sim_state->speed * dt;
}



static SimState predict_future(int steps, float dt) {
    SimState sim_state = {
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = rover_pose.speed,
        .ang_speed = rover_pose.angular_speed,
        .wp_idx = active_path.current
    };

    for (int i = 0; i < steps; i++) {
        
        if (sim_state.wp_idx >= active_path.count) break;

        // advance waypoint if reached
        const Waypoint *goal = &active_path.waypoints[sim_state.wp_idx];
        float dx = goal->x - sim_state.x;
        float dz = goal->z - sim_state.z;
        if (dx*dx + dz*dz < WAYPOINT_REACH_THRESHOLD) {
            sim_state.wp_idx++;
            if (sim_state.wp_idx >= active_path.count) break;
        }

        float throttle, steer;
        sim_control(&sim_state, &throttle, &steer, 1);
        sim_step(&sim_state, throttle, steer, dt);
    }
    return sim_state;
}

// =======

void init_rover_controller(void) {
    rover_pose.origin.x    = 0.0f;
    rover_pose.origin.z    = 0.0f;
    rover_pose.dir_angle   = 0.0f;
    rover_pose.speed       = 0.0f;
    rover_pose.angular_speed = 0.0f;
    active_path.count   = 0;
    active_path.current = 0;
    rover_mode = MODE_AUTO;
}


void update_odometry(float dt) {
    float throttle = get_throttle();
    float steer = get_steer();

    // same physics as sim_step()/sensor_control.c with noise injection
    if (throttle != 0.0f) {
        float noise = gaussian_noise() * SPEED_NOISE * ACCELERATION;
        rover_pose.speed += (throttle * ACCELERATION + noise) * dt;
        rover_pose.speed  = fmaxf(-MAX_SPEED, fminf(MAX_SPEED, rover_pose.speed));
    } else {
        float friction_amt = FRICTION * dt;
        if (rover_pose.speed > 0.0f){
            rover_pose.speed = fmaxf(0.0f, rover_pose.speed - friction_amt);
        }
        else {
            rover_pose.speed = fminf(0.0f, rover_pose.speed + friction_amt);
        }
    }

    if (steer != 0.0f) {
        float noise = gaussian_noise() * ANGULAR_NOISE * ANGULAR_ACCELERATION;
        rover_pose.angular_speed += (steer * ANGULAR_ACCELERATION + noise) * dt;
        rover_pose.angular_speed  = fmaxf(-MAX_ANGULAR_SPEED, fminf(MAX_ANGULAR_SPEED, rover_pose.angular_speed));
    } else {
        float friction = ANGULAR_FRICTION * dt;
        if (fabsf(rover_pose.angular_speed) < friction) {
            rover_pose.angular_speed = 0.0f;
        } else {
            rover_pose.angular_speed -= friction * (rover_pose.angular_speed > 0.0f ? 1.0f : -1.0f);
        }
    }

    rover_pose.dir_angle += rover_pose.angular_speed * dt;
    rover_pose.origin.x  += cosf(rover_pose.dir_angle) * rover_pose.speed * dt;
    rover_pose.origin.z  += sinf(rover_pose.dir_angle) * rover_pose.speed * dt;
}


// runs lookahead to correct errors and follow path
void update_path_follower(float dt) {
    if (active_path.current >= active_path.count) {
        set_throttle(0.0f);
        set_steer(0.0f);
        return;
    }

    // current error
    const Waypoint *goal = &active_path.waypoints[active_path.current];
    float dx = goal->x - rover_pose.origin.x;
    float dz = goal->z - rover_pose.origin.z;
    float dist_sqr = dx*dx + dz*dz;

    if (dist_sqr < WAYPOINT_REACH_THRESHOLD) {
        active_path.current++;
        printf("Waypoint %d reached\n", active_path.current);
        return;
    }

    float desired_curr = atan2f(dz, dx);
    float error_curr = wrap_angle(desired_curr - rover_pose.dir_angle);

    // predicted future error
    SimState p_state = predict_future(LOOK_AHEAD_STEPS, LOOK_AHEAD_DT);

    float p_error = 0.0f;
    if (p_state.wp_idx < active_path.count) {
        const Waypoint *future_goal = &active_path.waypoints[p_state.wp_idx];
        float p_dx = future_goal->x - p_state.x;
        float p_dz = future_goal->z - p_state.z;
        float desired_future = atan2f(p_dz, p_dx);
        p_error = wrap_angle(desired_future - p_state.dir_angle);
    }

    // blend/weight errors
    // same PD controller mechanics as sim_control but with blended error and current velocity
    float blended_error = (1.0f - PREDICT_BLEND_WEIGHT) * error_curr + PREDICT_BLEND_WEIGHT  * p_error;
    float steer = KP * blended_error - KD * rover_pose.angular_speed;
    steer = fmaxf(STEER_CLAMP_MIN, fminf(STEER_CLAMP_MAX, steer));

    float error_factor = ERROR_FACTOR_MAX - fminf(ERROR_FACTOR_MAX,
                                                  fabsf(error_curr) / (float)M_PI);
    float throttle = THROTTLE_BASE + THROTTLE_SCALE * (error_factor * error_factor);
    if (dist_sqr < SLOW_DIST_SQR) throttle *= (dist_sqr / SLOW_DIST_SQR);

    set_throttle(throttle);
    set_steer(steer);
}


void set_waypoints(Waypoint *points, int count) {
    int n = count < MAX_WAYPOINTS ? count : MAX_WAYPOINTS;
    for (int i = 0; i < n; i++) active_path.waypoints[i] = points[i];
    active_path.count   = n;
    active_path.current = 0;
}





/* BELOW IS FOR RENDERING MOVEMENT DATA */
void render_predicted_path(void)
{
    if (active_path.current >= active_path.count) return;

    SimState sim_state = {
        .x = rover_pose.origin.x,
        .z = rover_pose.origin.z,
        .dir_angle = rover_pose.dir_angle,
        .speed = rover_pose.speed,
        .ang_speed = rover_pose.angular_speed,
        .wp_idx = active_path.current
    };

    glBegin(GL_LINE_STRIP);
    glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
    // TODO: fixed y, doesn't account for terrain height, beyond scope of current project though
    glVertex3f(sim_state.x, 0.25f, sim_state.z);

    // draw out simulated path
    for (int i = 0; i < PREDICTION_STEPS; i++) {
        if (sim_state.wp_idx >= active_path.count) break;
        const Waypoint *goal = &active_path.waypoints[sim_state.wp_idx];
        float dx = goal->x - sim_state.x, dz = goal->z - sim_state.z;
        if (dx*dx + dz*dz < WAYPOINT_REACH_THRESHOLD) {
            sim_state.wp_idx++;
            if (sim_state.wp_idx >= active_path.count) break;
        }

        float throttle, steer;
        sim_control(&sim_state, &throttle, &steer, 1); // 1 = use predictive gains
        sim_step(&sim_state, throttle, steer, PREDICTION_DT);

        glColor3f(1.0f, 1.0f, 1.0f);
        glVertex3f(sim_state.x, 0.25f, sim_state.z);

        // mark lookahead point with a yellow dot
        if (i == LOOK_AHEAD_STEPS - 1) {
            glEnd();
            glPointSize(8.0f);
            glBegin(GL_POINTS);
            glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
            glVertex3f(sim_state.x, 0.3f, sim_state.z);
            glEnd();
            glPointSize(1.0f);

            // continue prediction line
            glBegin(GL_LINE_STRIP);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            glVertex3f(sim_state.x, 0.25f, sim_state.z);
        }
    }

    glEnd();
}


void render_pose_error(void) {
    const SensorState *truth = get_sensor_state();

    float true_x = truth->origin.x;
    float true_z = truth->origin.z;
    float expected_x = rover_pose.origin.x;
    float expected_z = rover_pose.origin.z;

    // red line showing drift gap
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.2f, 0.2f);
    glVertex3f(true_x, 0.3f, true_z);
    glVertex3f(expected_x, 0.3f, expected_z);
    glEnd();
    glLineWidth(1.0f);

    glPointSize(10.0f);
    glBegin(GL_POINTS);

    glColor3f(0.0f, 1.0f, 0.0f);  // green for ground truth
    glVertex3f(true_x, 0.3f, true_z);
    glColor3f(0.0f, 0.0f, 1.0f);  // blue for believed pose
    glVertex3f(expected_x, 0.3f, expected_z);

    glEnd();
    glPointSize(1.0f);

}


void render_waypoints(void)
{
    if (active_path.count == 0) return;


    glPointSize(8.0f);
    glBegin(GL_POINTS);
    for (int i = active_path.current; i < active_path.count; i++) {
        if (i == active_path.current) {
            glColor3f(1.0f, 1.0f, 0.0f);
        } else {
            glColor3f(0.3f, 0.8f, 0.6f); 
        }
        glVertex3f(active_path.waypoints[i].x, 0.25f, active_path.waypoints[i].z);
    }
    glEnd();
    glPointSize(1.0f);
}


/* 
   Autonomous loop checklist
   ─────────────────────────
   1. Path follower        -  DONE
   2. Odometry noise       -  DONE
   3. Frontier detection   —  finds frontier cells in the 2D column map
   4. Frontier clustering  —  groups them into meaningful targets
   5. A*                   —  plans a path from rover to chosen frontier
   6. Replanning logic     —  ties it all together into an autonomous loop
   7. Collision Detection
*/