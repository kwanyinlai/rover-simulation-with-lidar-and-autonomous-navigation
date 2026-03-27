#include "rendering/scene.h"
#include "core/vec3.h"

#define INIT_SIZE 32

void triangle_array_init(TriangleArray *scene) {
    scene->data = NULL;
    scene->size = 0;
    scene->capacity = 0;
}

void triangle_array_push_back(TriangleArray *scene, Triangle triangle) {
    if (scene->size >= scene->capacity) {
        size_t new_capacity = scene->capacity == 0 ? INIT_SIZE : scene->capacity * 2;
        Triangle *new_data = realloc(scene->data, new_capacity * sizeof(Triangle));
        if (new_data == NULL) {
            fprintf(stderr, "Failed to allocate memory for TriangleArray\n");
            exit(1);
        }
        scene->data = new_data;
        scene->capacity = new_capacity;
    }
    scene->data[scene->size++] = triangle;
}

void triangle_array_free(TriangleArray *scene) {
    free(scene->data);
    scene->data = NULL;
    scene->size = 0;
    scene->capacity = 0;
}

void mesh_add_quad(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3) {
    Triangle t1 = {v0, v1, v2};
    Triangle t2 = {v0, v2, v3};
    triangle_array_push_back(scene, t1);
    triangle_array_push_back(scene, t2);
}

void mesh_add_triangle(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2) {
    Triangle t = {v0, v1, v2};
    triangle_array_push_back(scene, t);
}

void mesh_add_quad_tesselated(TriangleArray *scene, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3, int divs) {
    for (int i = 0; i < divs; i++) {
        float t0 = (float)i / divs;
        float t1 = (float)(i + 1) / divs;
        for (int j = 0; j < divs; j++) {
            float s0 = (float)j / divs;
            float s1 = (float)(j + 1) / divs;
            Vector3 tl = {v0.x + s0 * (v1.x - v0.x) + t0 * (v3.x - v0.x),
                          v0.y + s0 * (v1.y - v0.y) + t0 * (v3.y - v0.y),
                          v0.z + s0 * (v1.z - v0.z) + t0 * (v3.z - v0.z)};
            Vector3 tr = {v0.x + s1 * (v1.x - v0.x) + t0 * (v3.x - v0.x),
                          v0.y + s1 * (v1.y - v0.y) + t0 * (v3.y - v0.y),
                          v0.z + s1 * (v1.z - v0.z) + t0 * (v3.z - v0.z)};
            Vector3 bl = {v0.x + s0 * (v1.x - v0.x) + t1 * (v3.x - v0.x),
                          v0.y + s0 * (v1.y - v0.y) + t1 * (v3.y - v0.y),
                          v0.z + s0 * (v1.z - v0.z) + t1 * (v3.z - v0.z)};
            Vector3 br = {v0.x + s1 * (v1.x - v0.x) + t1 * (v3.x - v0.x),
                          v0.y + s1 * (v1.y - v0.y) + t1 * (v3.y - v0.y),
                          v0.z + s1 * (v1.z - v0.z) + t1 * (v3.z - v0.z)};
            mesh_add_quad(scene, tl, tr, br, bl);
        }
    }
}

void mesh_add_box(TriangleArray *scene, float centre_x, float centre_y, float centre_z,
                    float half_width, float half_height, float half_depth){
    int divs = 5;
    float x0=centre_x-half_width, x1=centre_x+half_width;
    float y0=centre_y, y1=centre_y+half_height*2;
    float z0=centre_z-half_depth, z1=centre_z+half_depth;
    mesh_add_quad_tesselated(scene, 
        (Vector3){x0,y0,z1}, 
        (Vector3){x1,y0,z1}, 
        (Vector3){x1,y1,z1}, 
        (Vector3){x0,y1,z1}, 
        divs
    );
    mesh_add_quad_tesselated(scene, 
        (Vector3){x1,y0,z0},
        (Vector3){x0,y0,z0},
        (Vector3){x0,y1,z0},
        (Vector3){x1,y1,z0},divs
    );
    mesh_add_quad_tesselated(scene,
        (Vector3){x0,y0,z0},
        (Vector3){x0,y0,z1},
        (Vector3){x0,y1,z1},
        (Vector3){x0,y1,z0},
        divs
    );
    mesh_add_quad_tesselated(scene, 
        (Vector3){x1,y0,z1}, 
        (Vector3){x1,y0,z0},
        (Vector3){x1,y1,z0},
        (Vector3){x1,y1,z1},
        divs
    );
    mesh_add_quad_tesselated(scene, 
        (Vector3){x0,y1,z1},
        (Vector3){x1,y1,z1},
        (Vector3){x1,y1,z0},
        (Vector3){x0,y1,z0},
        divs
    );
}

// ======================================

void build_floor(TriangleArray *scene){
    int div_floor = 20; // floor tesselation
    mesh_add_quad_tesselated(
        scene,
        (Vector3){-25.f, 0.f, -25.f},
        (Vector3){25.f, 0.f, -25.f},
        (Vector3){25.f, 0.f, 25.f},
        (Vector3){-25.f, 0.f, 25.f},
        div_floor
    );
}

void height_map(TriangleArray *scene, int min_x, int max_x, int min_z, int max_z, int grid_resolution){
    float height_map[grid_resolution][grid_resolution];
    
}




// ======================================


void build_scene(TriangleArray *scene) {
    int div_walls = 8;  // walls tesselation


    // Floor
    build_floor(scene);

    // Back wall
    mesh_add_quad_tesselated(
        scene,
        (Vector3){-15.f, 0.f, -12.f},
        (Vector3){15.f, 0.f, -12.f},
        (Vector3){15.f, 6.f, -12.f},
        (Vector3){-15.f, 6.f, -12.f},
        div_walls
    );

    // Left wall
    mesh_add_quad_tesselated(scene,
        (Vector3){-15.f, 0.f, 12.f},
        (Vector3){-15.f, 0.f, -12.f},
        (Vector3){-15.f, 6.f, -12.f},
        (Vector3){-15.f, 6.f, 12.f},
        div_walls
    );

    // Right wall
    mesh_add_quad_tesselated(scene,
        (Vector3){15.f, 0.f, -12.f},
        (Vector3){15.f, 0.f, 12.f},
        (Vector3){15.f, 6.f, 12.f},
        (Vector3){15.f, 6.f, -12.f},
        div_walls
    );

    // Obstacles
    mesh_add_box(scene, 3.0f, 0.0f, -5.f, 1.2f, 1.0f, 1.2f);
    mesh_add_box(scene, -4.0f, 0, 2.0f, 0.8f, 1.5f, 0.8f);
    mesh_add_box(scene, 6.0f, 0.0f, 3.0f, 1.5f, 0.7f, 1.5f);
    mesh_add_box(scene, -2.0f, 0.0f, -6.0f, 0.6f, 2.0f, 0.6f);
    mesh_add_box(scene, 0.5f, 0.0f, 5.0f, 2.0f, 0.5f, 0.8f);
    mesh_add_box(scene, -7.0f,0, -3.0f, 1.0f, 1.2f, 1.0f);

    printf("%zu triangles\n", scene->size);
}