# ifndef ROVER_CONTROLLER_H
# define ROVER_CONTROLLER_H

# define MAX_WAYPOINTS 10





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

extern Path active_path;

void init_rover_controller();
void update_odometry(float dt);
void update_path_follower(float dt);
void set_waypoints(Waypoint *wps, int count);
void render_predicted_path();
void render_pose_error();



# endif // ROVER_CONTROLLER_H