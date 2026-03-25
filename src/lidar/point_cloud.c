#include "lidar/point_cloud.h"
#define INIT_SIZE 32

void init_point_cloud(PointCloud *pc) {
    pc->data = NULL;
    pc->size = 0;
    pc->capacity = 0;
}

void point_cloud_push_back(PointCloud *pc, Vector3 pos, float dist, float intensity) {
    if (pc->size >= pc->capacity) {
        size_t new_capacity = pc->capacity == 0 ? INIT_SIZE : pc->capacity * 2;
        PointCloudEntry *new_data = realloc(pc->data, new_capacity * sizeof(PointCloudEntry));
        if (new_data == NULL) {
            perror("realloc");
            exit(1);
        }
        pc->data = new_data;
        pc->capacity = new_capacity;
    }
    PointCloudEntry new_entry = (PointCloudEntry){pos, dist, intensity, 0.0f};
    pc->data[pc->size++] = new_entry;
}

void point_cloud_free(PointCloud *pc) {
    free(pc->data);
    pc->data = NULL;
    pc->size = 0;
    pc->capacity = 0;
}

void point_cloud_age(PointCloud *pc, float delta_time) {
    static const float max_age = 5.0f; 
    for (size_t i = 0; i < pc->size; i++) {
        pc->data[i].age += delta_time;
        if (pc->data[i].age > max_age) {
            // Remove old point by swapping with last and reducing size
            pc->data[i] = pc->data[--pc->size];
        }
    }
}