#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

#include "rendering/vec3.h"
#include "rendering/scene.h"
#include "rendering/scene_state.h"
#include "lidar/point_cloud.h"
#include "lidar/sensor_control.h"
#include "lidar/occupancy_map.h"


#define DECAY_RATE 0.50f

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

void render_cloud(PointCloud *cloud, float dt){
    // point_cloud_age(cloud, dt); 
    // disabling fading for now, but can enable by uncommenting above line, and changing alpha calculation below to use age.
    glBegin(GL_POINTS);

    float stops[7][3] = {
        {0,0,0}, {0.3f,0,0.5f}, {0.6f,0,0},
        {0.9f,0.4f,0}, {1,0.8f,0}, {1,1,0.5f}, {1,1,1}
    };

    float y_min = 0.0f;
    float y_max = 6.0f;
    for (size_t i = 0; i < cloud->size; i++) {
        float t = (cloud->data[i].position.y - y_min) / (y_max - y_min);
        t = fmaxf(0.0f, fminf(t, 1.0f));  // clamp so stops[] index never goes OOB

        float idx = t * 6.0f;
        int   si  = (int)idx; if (si > 5) si = 5;
        float f   = idx - si;

        float r = stops[si][0] + (stops[si+1][0] - stops[si][0]) * f;
        float g = stops[si][1] + (stops[si+1][1] - stops[si][1]) * f;
        float b = stops[si][2] + (stops[si+1][2] - stops[si][2]) * f;
        // float alpha = expf(-DECAY_RATE * cloud->data[i].age);
        float alpha = 1.0f; // disabling and enabling fading

        glColor4f(r * alpha, g * alpha, b * alpha, alpha);
        glVertex3f(cloud->data[i].position.x,
                   cloud->data[i].position.y,
                   cloud->data[i].position.z);
    }
    glEnd();
}

void render_occupancy_map(const OccupancyMap *map){
    glBegin(GL_LINES);
    for (int z = 0; z < map->depth; z++)
    for (int y = 0; y < map->height; y++)
    for (int x = 0; x < map->width; x++) {
        CELL_STATE state = occupancy_map_get_cell(map, x, y, z);
        if (state == OCCUPIED){
            float log_odds = occupancy_map_get_log_odds(map, x, y, z);
            float confidence = fminf(1.0f, log_odds / 5.0f) * 0.5f;
            glColor4f(confidence, 0.0f, 0.0f, 0.5f);
            float r = 1.0f, g = 0.0f, b = 0.0f; // red for occupied
            glColor4f(r * confidence, g * confidence, b * confidence, 0.5f);
            
        }
        else if (state == FREE && is_frontier_point(map, x, y, z)){
            float r = 0.80f, g = 0.80f, b = 0.80f; // green for frontier
            glColor4f(r, g, b, 0.1f);
        }
        else continue; // skip free and unknown cells
        
        float centre_x = map->origin.x + (x + 0.5f) * map->cell_size;
        float centre_y = map->origin.y + (y + 0.5f) * map->cell_size;
        float centre_z = map->origin.z + (z + 0.5f) * map->cell_size;
        float h = map->cell_size * 0.45f; // half extent


        // drawing cube using line segments
        // buffering all vertices in a single glBegin/glEnd block for performance, since there can be many cubes to draw
        // bottom face
        glVertex3f(centre_x-h,centre_y-h,centre_z-h); glVertex3f(centre_x+h,centre_y-h,centre_z-h);
        glVertex3f(centre_x+h,centre_y-h,centre_z-h); glVertex3f(centre_x+h,centre_y-h,centre_z+h);
        glVertex3f(centre_x+h,centre_y-h,centre_z+h); glVertex3f(centre_x-h,centre_y-h,centre_z+h);
        glVertex3f(centre_x-h,centre_y-h,centre_z+h); glVertex3f(centre_x-h,centre_y-h,centre_z-h);
        // top face
        glVertex3f(centre_x-h,centre_y+h,centre_z-h); glVertex3f(centre_x+h,centre_y+h,centre_z-h);
        glVertex3f(centre_x+h,centre_y+h,centre_z-h); glVertex3f(centre_x+h,centre_y+h,centre_z+h);
        glVertex3f(centre_x+h,centre_y+h,centre_z+h); glVertex3f(centre_x-h,centre_y+h,centre_z+h);
        glVertex3f(centre_x-h,centre_y+h,centre_z+h); glVertex3f(centre_x-h,centre_y+h,centre_z-h);
        // verticals
        glVertex3f(centre_x-h,centre_y-h,centre_z-h); glVertex3f(centre_x-h,centre_y+h, centre_z-h);
        glVertex3f(centre_x+h, centre_y-h, centre_z-h); glVertex3f(centre_x+h, centre_y+h, centre_z-h);
        glVertex3f(centre_x+h, centre_y-h, centre_z+h); glVertex3f(centre_x+h, centre_y+h, centre_z+h);
        glVertex3f(centre_x-h, centre_y-h, centre_z+h); glVertex3f(centre_x-h, centre_y+h, centre_z+h);
    }
    glEnd();

}

float lerp(float a, float b, float t){
    return a + (b - a) * t;
}

void render_sensor(){
    Vector3 sensor_pos;
    get_sensor_pos(&sensor_pos);
    glPushMatrix();
    glTranslatef(sensor_pos.x, sensor_pos.y, sensor_pos.z);
    glColor3f(1.0f, 0.0f, 0.0f); // red for the sensor
    glutSolidSphere(0.5f, 12, 12);
    glPopMatrix();

    // direction arrow
    float dir_angle = get_sensor_dir_angle();
    float velocity = get_sensor_velocity();
    float throttle = get_throttle(); 

    float idle_scale = 0.4f;
    float throttle_scale = 1.8f;

    // Pick scale based on throttle
    float target_scale = (throttle > 0.5f) ? throttle_scale : idle_scale;

    // Smooth toward it
    static float current_scale = 0.4f;
    current_scale = lerp(current_scale, target_scale, 0.05f);

    float arrow_length = 0.4 * velocity * current_scale + 1.f;
    

    Vector3 arrow_dir = {cosf(dir_angle), 0.0f, sinf(dir_angle)};
    Vector3 arrow_end = vector3_add(sensor_pos, vector3_scale(arrow_dir, arrow_length));

    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);

    // arrow shaft

    glVertex3f(sensor_pos.x, sensor_pos.y, sensor_pos.z);
    glVertex3f(arrow_end.x, arrow_end.y, arrow_end.z);


    // arrow head
    float head_size = 0.3f;
    float head_angle = 0.5f; // radians
    float left_angle = dir_angle + M_PI - head_angle;
    float right_angle = dir_angle + M_PI + head_angle;
    Vector3 left_head = {
        arrow_end.x + head_size * cosf(left_angle),
        arrow_end.y,
        arrow_end.z + head_size * sinf(left_angle)
    };
    Vector3 right_head = {
        arrow_end.x + head_size * cosf(right_angle),
        arrow_end.y,
        arrow_end.z + head_size * sinf(right_angle)
    };

    
    glVertex3f(arrow_end.x, arrow_end.y, arrow_end.z);
    glVertex3f(left_head.x, left_head.y, left_head.z);
    glVertex3f(arrow_end.x, arrow_end.y, arrow_end.z);
    glVertex3f(right_head.x, right_head.y, right_head.z);
    glEnd();
    glLineWidth(1.0f);

}


