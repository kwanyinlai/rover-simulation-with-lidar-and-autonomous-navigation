# ifndef ROVER_CONTROLLER_H
# define ROVER_CONTROLLER_H

# define MAX_WAYPOINTS 10

# define KP 0.8f
# define KD 0.6f

# define ROVER_HEIGHT_CELLS 3


typedef struct {
    int x, z;
} Waypoint;

typedef struct {
    Waypoint waypoints[MAX_WAYPOINTS];
    int count;
    int current;
} Path;

typedef enum {
    MODE_MANUAL,
    MODE_AUTO
} RoverMode;

typedef struct {
    float x, z;
    float dir_angle;
    float speed;
    float ang_speed;
    int wp_idx;
} SimState;


extern Path active_path;

void init_rover_controller(void);
void update_odometry(float dt);
void update_path_follower(float dt);
void set_waypoints(Waypoint *wps, int count);
void render_predicted_path(void);
void render_pose_error(void);
void render_waypoints(void);



# endif // ROVER_CONTROLLER_H