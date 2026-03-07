#include "point_cloud.h"
#define INIT_SIZE 32

void point_cloud(PointCloud *pc) {
    pc->data = NULL;
    pc->size = 0;
    pc->capacity = 0;
}

void point_cloud_push_back(PointCloud *pc, Vector3 pos, float dist) {
    if (pc->size >= pc->capacity) {
        size_t new_capacity = pc->capacity == 0 ? INIT_SIZE : pc->capacity * 2;
        Vector3 *new_data = realloc(pc->data, new_capacity * sizeof(PointCloudEntry));
        if (new_data == NULL) {
            fprintf(stderr, "Failed to allocate memory for PointCloud\n");
            exit(1);
        }
        pc->data = new_data;
        pc->capacity = new_capacity;
    }
    PointCloudEntry new_entry = (PointCloudEntry){pos, dist};
    pc->data[pc->size++] = new_entry;
}

void point_cloud_free(PointCloud *pc) {
    free(pc->data);
    pc->data = NULL;
    pc->size = 0;
    pc->capacity = 0;
}