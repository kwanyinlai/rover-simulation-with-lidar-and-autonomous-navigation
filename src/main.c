#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "core/io_utils.h"
#include "core/vec3.h"
#include "rendering/scene.h"
#include "scene/scene_state.h"
#include "rendering/camera.h"
#include "rendering/renderer.h"
#include "lidar/lidar_sensor.h"
#include "lidar/sensor_control.h"
#include "scene/point_cloud.h"
#include "scene/occupancy_map.h"
#include "piping/method_dispatcher.h"
#include "rover/rover_controller.h"
#include "rover/ekf_fusion.h"


TriangleArray scene;
PointCloud cloud;
OccupancyMap occupancy_grid_3d;
OccupancyMap occupancy_grid_2d;

static float last_time = 0.0f;
extern int is_render_scene;
extern int is_paused;
extern int toggle_occupancy_map_2d;
extern int toggle_occupancy_map_3d;
extern int toggle_point_cloud;
extern RoverMode rover_mode;


#include <signal.h>
#include <sys/select.h>
#include <sys/wait.h>

// signal handler for handling forks
static int scan_coord_pid = -1;
static int rollout_coord_pid = -1;
// static int updated_voxels_pipe_rd = -1; // pipe for receiving updated voxels from occupancy updater process
static int frontier_waypoints_read_fd = -1;

static void sigterm_handler(int sig) {
    (void)sig;
    exit(0);
}

void handle_sigint(int sig) {
    (void)sig;
    if (scan_coord_pid > 0) {
        kill(scan_coord_pid, SIGTERM);
        waitpid(scan_coord_pid, NULL, 0);
    }
    if (rollout_coord_pid > 0) {
        kill(rollout_coord_pid, SIGTERM);
        waitpid(rollout_coord_pid, NULL, 0);
    }
    exit(0);
}

// store worker ids to kill before terminating coordinator
static int worker_pids[NUM_WORKERS] = {0};
static int rollout_workers_pid[NUM_WORKERS] = {0};

static void coordinator_sigterm(int sig) {
    (void)sig;
    for (int i = 0; i < NUM_WORKERS; i++) {
        if (worker_pids[i] > 0) {
            if (kill(worker_pids[i], SIGTERM) < 0) {
                perror("kill ray worker");
            }
            if (waitpid(worker_pids[i], NULL, 0) < 0) {
                perror("waitpid ray worker");
            }
        }
    }
    exit(0);
}

static void rollout_coordinator_sigterm(int sig) {
    (void)sig;
    for (int i = 0; i < NUM_WORKERS; i++) {
        if (rollout_workers_pid[i] > 0) {
            if (kill(rollout_workers_pid[i], SIGTERM) < 0) {
                perror("kill rollout worker");
            }
            if (waitpid(rollout_workers_pid[i], NULL, 0) < 0) {
                perror("waitpid rollout worker");
            }
        }
    }
    exit(0);
}

// TODO: this is redundant right now because the frontier analyzer does not assign
// waypoints based on frontier points. This will not be complted for the project
// but is intended to be for extensibility when frontier-based exploration is fully
// implemented. For now, only artifacts of this infrastructure exist.

// Several bugs related to this architecture that caused issues preventing its 
// completion in the current implementation:
// - LIDAR cannot scan down so there is a radius around the rover that is UNKNOWN
//.  because we assume UNKNOWN cells are blocking for pathfinding, the rover gets stuck
// - Finding the best frontier point to explore, and importantly, projecting that point
//.  to the closest navigable point
// - Add pausing to navigation to allow time for LIDAR scanning and frontier analysis before
//.  moving away or finding a new frontier to explore
static void consume_frontier_waypoints(void) {
    if (frontier_waypoints_read_fd < 0) {
        return;
    }

    fd_set waypoint_set;
    struct timeval timeout = {0, 0};

    while (1) {
        FD_ZERO(&waypoint_set);
        FD_SET(frontier_waypoints_read_fd, &waypoint_set);

        int ready = select(frontier_waypoints_read_fd + 1, &waypoint_set, NULL, NULL, &timeout);
        if (ready < 0) {
            perror("select frontier waypoints");
            return;
        }
        if (ready == 0 || !FD_ISSET(frontier_waypoints_read_fd, &waypoint_set)) {
            return;
        }

        int waypoint_count = 0;
        int count_status = read_exact(frontier_waypoints_read_fd, &waypoint_count, sizeof(int));
        if (count_status <= 0) {
            return;
        }

        if (waypoint_count < 0) {
            continue;
        }
        if (waypoint_count > MAX_WAYPOINTS) {
            waypoint_count = MAX_WAYPOINTS;
        }

        Waypoint waypoints[MAX_WAYPOINTS];
        if (waypoint_count > 0) {
            size_t expected = sizeof(Waypoint) * (size_t)waypoint_count;
            if (read_exact(frontier_waypoints_read_fd, waypoints, expected) <= 0) {
                return;
            }
        }

        set_waypoints(waypoints, waypoint_count);
    }
}

void create_workers(void){

    signal(SIGTERM, sigterm_handler);

    int scan_cmd_pipe[2];
    int point_batch_pipe[2];
    int ray_batch_results_pipe[2];    
    int updated_voxels_pipe[2];
    int frontier_waypoints_pipe[2];
    int rover_pose_pipe[2];
    int rollout_cmd_pipe[2];
    int rollout_result_pipe[2];
    int scan_match_cmd_pipe[2];
    int scan_match_result_pipe[2];

    if (pipe(rover_pose_pipe) < 0) { 
        perror("pipe rover_pose"); 
        exit(1);
    }
    if (pipe(ray_batch_results_pipe) < 0) {
        perror("pipe ray_batch_results"); 
        exit(1);
    }
    if (pipe(scan_cmd_pipe) < 0) {
        perror("pipe scan_cmd");
        exit(1);
    }
    if (pipe(point_batch_pipe) < 0) { 
        perror("pipe point_batch"); 
        exit(1);
    }
    if (pipe(updated_voxels_pipe) < 0) { 
        perror("pipe updated_voxels"); 
        exit(1);
    }
    if (pipe(frontier_waypoints_pipe) < 0) { 
        perror("pipe frontier_waypoints"); 
        exit(1);
    }
    if (pipe(rollout_cmd_pipe) < 0) { 
        perror("pipe rollout_cmd"); 
        exit(1);
    }
    if (pipe(rollout_result_pipe) < 0) { 
        perror("pipe rollout_result"); 
        exit(1);
    }
    if (pipe(scan_match_cmd_pipe) < 0) {
        perror("pipe scan_match_cmd");
        exit(1);
    }
    if (pipe(scan_match_result_pipe) < 0) {
        perror("pipe scan_match_result");
        exit(1);
    }


    scan_coord_pid = fork();
    if (scan_coord_pid < 0) {
        perror("Failed to fork process");
        exit(1);
    }
    else if (scan_coord_pid == 0) {
        // scan coordinator

        signal(SIGTERM, coordinator_sigterm);

        // receive scan commands from main process
        close(scan_cmd_pipe[1]); // close write end of scan cmd 
        // coordinator will write point batches to occupancy updater
        close(point_batch_pipe[0]); // close read end
        // unused updated voxel pipes for occupancy manager
        close(updated_voxels_pipe[0]); 
        close(updated_voxels_pipe[1]);  
        close(frontier_waypoints_pipe[0]);
        close(frontier_waypoints_pipe[1]);
        close(rover_pose_pipe[0]);
        close(rover_pose_pipe[1]);
        close(rollout_cmd_pipe[0]);
        close(rollout_cmd_pipe[1]);
        close(rollout_result_pipe[0]);
        close(rollout_result_pipe[1]);
        
        close(scan_match_cmd_pipe[1]);
        close(scan_match_result_pipe[0]);

        int ray_task_pipe[NUM_WORKERS][2];
        int ray_results_pipe[NUM_WORKERS][2];
        for (int i = 0; i < NUM_WORKERS; i++) {
            if (pipe(ray_task_pipe[i]) < 0) { 
                perror("pipe ray_task");
                exit(1);
            }
            if (pipe(ray_results_pipe[i]) < 0) {
                perror("pipe ray_results");
                exit(1);
            }
        }
        for (int i = 0; i < NUM_WORKERS; i++) {
            int wpid = fork();
            if (wpid < 0) {
                perror("Failed to fork ray worker");
                for (int j = 0; j < i; j++) {
                    kill(worker_pids[j], SIGTERM);
                    waitpid(worker_pids[j], NULL, 0);
                }
                exit(1);
            }
            if (wpid == 0) {
                signal(SIGTERM, sigterm_handler);
                // additionally close scan_cmd read
                close(scan_cmd_pipe[0]);
                // additionally close write end of ray batch results
                close(point_batch_pipe[1]);
                // additionally close read end of ray batch results
                close(ray_batch_results_pipe[0]);
                // close scan match pipes
                close(scan_match_cmd_pipe[0]);
                close(scan_match_result_pipe[1]);
                // worker process
                for (int j = 0; j < NUM_WORKERS; j++) {
                    if (j != i) {
                        // close all pipes not related to this worker
                        close(ray_task_pipe[j][0]);
                        close(ray_task_pipe[j][1]);
                        close(ray_results_pipe[j][0]);
                        close(ray_results_pipe[j][1]);
                    }
                    else{
                        // keep pipes related to this worker, but close the ends not used by this worker
                        close(ray_task_pipe[j][1]); // close write end
                        close(ray_results_pipe[j][0]); // close read end
                    }
                }
                run_worker_loop(ray_task_pipe[i][0], ray_results_pipe[i][1], &scene);
                exit(0);
            } else {
                // record worker_pids for cleanup
                worker_pids[i] = wpid;
            }
        }

        run_coordinator_loop(scan_cmd_pipe[0],
                             ray_batch_results_pipe[1], 
                             scan_match_cmd_pipe[0], 
                             scan_match_result_pipe[1],
                             ray_task_pipe, 
                             ray_results_pipe, 
                             point_batch_pipe[1]
        );
        exit(0);
    }

    int updater_pid = fork();
    if (updater_pid < 0) {
        perror("fork updater");
        exit(1);
    }
    else if (updater_pid == 0) {
        // occupancy updater
        close(scan_cmd_pipe[0]); // close read end
        close(point_batch_pipe[1]); // close write end
        close(updated_voxels_pipe[0]); // close read end
        close(frontier_waypoints_pipe[0]); // close read end
        close(frontier_waypoints_pipe[1]); // close write end
        close(rover_pose_pipe[0]); // close read end
        close(rover_pose_pipe[1]); // close write end
        close(rollout_cmd_pipe[0]);
        close(rollout_cmd_pipe[1]);
        close(rollout_result_pipe[0]);
        close(rollout_result_pipe[1]);
        close(scan_match_cmd_pipe[0]);
        close(scan_match_cmd_pipe[1]);
        close(scan_match_result_pipe[0]);
        close(scan_match_result_pipe[1]);
        run_occupancy_updater_loop(point_batch_pipe[0], updated_voxels_pipe[1], &occupancy_grid_3d);
        exit(0);
        // run occupancy updater loop
    }

    int frontier_pid = fork();
    if (frontier_pid < 0) {
        perror("fork frontier_analyzer");
        exit(1);
    }
    else if (frontier_pid == 0) {
        // frontier analyzer
        close(scan_cmd_pipe[0]); // close read end
        close(scan_cmd_pipe[1]); // close write end
        close(point_batch_pipe[0]); // close read end
        close(point_batch_pipe[1]); // close write end
        close(ray_batch_results_pipe[0]); // close read end
        close(ray_batch_results_pipe[1]); // close write end
        close(updated_voxels_pipe[1]); // close write end
        close(rover_pose_pipe[1]); // close write end
        close(rollout_cmd_pipe[0]);
        close(rollout_cmd_pipe[1]);
        close(rollout_result_pipe[0]);
        close(rollout_result_pipe[1]);
        close(scan_match_cmd_pipe[0]);
        close(scan_match_cmd_pipe[1]);
        close(scan_match_result_pipe[0]);
        close(scan_match_result_pipe[1]);
        run_frontier_analyzer_loop(updated_voxels_pipe[0], frontier_waypoints_pipe[1], rover_pose_pipe[0], &occupancy_grid_3d, &occupancy_grid_2d);
        exit(0);
    }

    rollout_coord_pid = fork();
    if (rollout_coord_pid < 0) {
        perror("fork rollout_coordinator");
        exit(1);
    }
    else if (rollout_coord_pid == 0) {
        signal(SIGTERM, rollout_coordinator_sigterm);

        close(scan_cmd_pipe[0]);
        close(scan_cmd_pipe[1]);
        close(point_batch_pipe[0]);
        close(point_batch_pipe[1]);
        close(ray_batch_results_pipe[0]);
        close(ray_batch_results_pipe[1]);
        close(updated_voxels_pipe[0]);
        close(updated_voxels_pipe[1]);
        close(frontier_waypoints_pipe[0]);
        close(frontier_waypoints_pipe[1]);
        close(rover_pose_pipe[0]);
        close(rover_pose_pipe[1]);
        close(rollout_cmd_pipe[1]);
        close(rollout_result_pipe[0]);
        close(scan_match_cmd_pipe[0]);
        close(scan_match_cmd_pipe[1]);
        close(scan_match_result_pipe[0]);
        close(scan_match_result_pipe[1]);

        int rollout_task_pipes[NUM_WORKERS][2];
        int rollout_costs_pipes[NUM_WORKERS][2];
        // deal with pipes separately so we can exit on error cleanly
        for (int i = 0; i < NUM_WORKERS; i++) {
            if (pipe(rollout_task_pipes[i]) < 0) {
                perror("pipe rollout_task");
                exit(1);
            }
            if (pipe(rollout_costs_pipes[i]) < 0) {
                perror("pipe rollout_cost");
                exit(1);
            }
        }
        for (int i = 0; i < NUM_WORKERS; i++){
            int wpid = fork();
            if (wpid < 0) {
                perror("fork rollout_worker");
                // kill any workers already spawned
                for (int j = 0; j < i; j++) {
                    kill(rollout_workers_pid[j], SIGTERM);
                    waitpid(rollout_workers_pid[j], NULL, 0);
                }
                exit(1);;
            }
            else if (wpid == 0) {
                signal(SIGTERM, sigterm_handler);

                close(rollout_cmd_pipe[0]);
                close(rollout_result_pipe[1]);

                for (int j = 0; j < NUM_WORKERS; j++) {
                    if (j != i) {
                        close(rollout_task_pipes[j][0]);
                        close(rollout_task_pipes[j][1]);
                        close(rollout_costs_pipes[j][0]);
                        close(rollout_costs_pipes[j][1]);
                    } else {
                        close(rollout_task_pipes[j][1]);
                        close(rollout_costs_pipes[j][0]);
                    }
                }

                run_rollout_worker_loop(rollout_task_pipes[i][0], rollout_costs_pipes[i][1], &scene);
                exit(0);
            } else {
                rollout_workers_pid[i] = wpid;
            }
        }

        run_rollout_coordinator_loop(rollout_cmd_pipe[0], 
                                     rollout_result_pipe[1], 
                                     rollout_task_pipes, 
                                     rollout_costs_pipes
        );
        exit(0);
    }
   
    close(scan_cmd_pipe[0]);
    close(point_batch_pipe[0]);
    close(point_batch_pipe[1]);
    // keep ray_batch read end open to read and render point clouds
    close(ray_batch_results_pipe[1]);
    close(frontier_waypoints_pipe[1]);
    close(updated_voxels_pipe[0]);
    close(updated_voxels_pipe[1]); 
    close(rover_pose_pipe[0]);
    close(rollout_cmd_pipe[0]);
    close(rollout_result_pipe[1]);

    close(scan_match_cmd_pipe[0]);
    close(scan_match_result_pipe[1]);
    set_scan_match_pipe_fds(scan_match_cmd_pipe[1], scan_match_result_pipe[0]);
    // updated_voxels_pipe_rd = updated_voxels_pipe[0]; // store read end for main loop to read updated voxels from occupancy updater

    set_scan_pipe_fds(scan_cmd_pipe[1], ray_batch_results_pipe[0]);
    frontier_waypoints_read_fd = frontier_waypoints_pipe[0];
    set_replan_pipe_fd(rover_pose_pipe[1]);
    set_rollout_pipe_fds(rollout_cmd_pipe[1], rollout_result_pipe[0]);
    
}

void display() {
    float current_time = glutGet(GLUT_ELAPSED_TIME) / 1000.0f; 
    float delta_time = current_time - last_time;
    last_time = current_time;
    // BG
    glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // CAMERA
    apply_camera();

    // MOVE ROVER
    if (!is_paused) {
        if (rover_mode == MODE_AUTO) {
            update_path_follower(delta_time);
        }

        update_odometry(delta_time);
        rover_control(delta_time);
        consume_frontier_waypoints();
    }
    

    // RENDER VISUAL ELEMENTS
    if (is_render_scene) render_wire();

    render_sensor();
    render_predicted_path();

    // move sensor
    if (!is_paused) {
        sensor_step(&scene, &cloud, &occupancy_grid_3d);
        // EKF integration is temporarily disabled.
        // update_lidar_fusion(&cloud, get_scan_theta());
    }

    // point cloud render
    glDepthMask(GL_FALSE);
    if (toggle_point_cloud) {
        render_cloud(&cloud, delta_time);
    }
    if (toggle_occupancy_map_3d) {
        render_occupancy_map(&occupancy_grid_3d);
    }
    if (toggle_occupancy_map_2d) {
        render_occupancy_map(&occupancy_grid_2d);
    }
    glDepthMask(GL_TRUE);

    render_pose_error();
    render_waypoints();

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
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    init_sensor_state();
    init_rover_controller();

    // TODO: we want to move to auto generated paths eventually, not for the course project though
    Waypoint test_path[] = {
        {6, 0},
        {12, 0},
        {12, 6},
        {6, 6},
        {6, 10},
        {-2, 10},
    };
    // set_waypoints(test_path, 6);

    init_point_cloud(&cloud);
    triangle_array_init(&scene);
    build_scene(&scene);
    init_occupancy_map(&occupancy_grid_3d, 300, 60, 240, 0.1f, (Vector3){-15.0f, 0.0f, -12.0f});
    init_occupancy_map(&occupancy_grid_2d, 300, 1, 240, 0.1f, (Vector3){-15.0f, 0.0f, -12.0f});

    create_workers();
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
    glutKeyboardUpFunc(keyboard_up);
    glutMouseFunc(mouse_button);
    glutMotionFunc(mouse_move);

    printf("mouse drag to orbit, +/- to zoom\n");
    printf("WASD to drive, P to pause, F to toggle frontier visualization, G to toggle 3D occupancy map, T to toggle 2D occupancy map, V to toggle point cloud, C to toggle between driving modes\n");
    glutShowWindow();
    glutMainLoop();
    return 0;
}


