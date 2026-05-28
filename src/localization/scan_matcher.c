#include "localization/scan_matcher.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

#define CONVERGENCE_THRESHOLD (1e-2f)
#define MAX_MATCH_DIST 1.0f

typedef struct {
    int idx; // idx in PC array
    int left, right; // child indices in PC array, -1 if no child
    int axis; // the axis we split by
} KDNode;

typedef struct {
    const PointCloud *scan;
    KDNode *nodes;
    int node_count;
    int root;
    int *indices;
} KDTree;

typedef struct {
    const PointCloud *scan;
    const int axis;
} KDSearchContext;


static int compar(void *ctx, const void *a, const void *b) {
    KDSearchContext *search_ctx = (KDSearchContext *)ctx;
    const int axis = search_ctx->axis;
    const PointCloud *scan = search_ctx->scan;
    const int idx_a = *(const int *)a;
    const int idx_b = *(const int *)b;
    float coordinate_axis_a = (
        axis == 0 ? scan->data[idx_a].position.x
        : scan->data[idx_a].position.z
    );
    float coordinate_axis_b = (
        axis == 0 ? scan->data[idx_b].position.x
        : scan->data[idx_b].position.z
    );
    if (coordinate_axis_a < coordinate_axis_b) return -1;
    else if (coordinate_axis_a > coordinate_axis_b) return 1;
    else return 0;
}

static int kd_build_recursive(const PointCloud *scan,
                              int *indices,
                              int start,
                              int end,
                              int depth,
                              KDNode *nodes,
                              int *node_count) {
    if (start >= end) return -1;
    int axis = depth % 2; // alternate between x and z axis
    KDSearchContext search_ctx = {scan, axis};
    qsort_r(indices + start,
            (size_t)(end - start),
            sizeof(int),
            &search_ctx,
            compar);
    int mid = (start + end) / 2;
    int node_index = (*node_count)++;
    nodes[node_index].idx = indices[mid];
    nodes[node_index].axis = axis;
    nodes[node_index].left = kd_build_recursive(scan, indices, start, mid, depth + 1, nodes, node_count);
    nodes[node_index].right = kd_build_recursive(scan, indices, mid + 1, end, depth + 1, nodes, node_count);
    return node_index;
}

static KDTree build_kd_tree(const PointCloud *scan) {

    KDTree tree = {0};
    tree.scan = scan;

    tree.nodes = malloc(sizeof(KDNode) * scan->size);
    tree.indices = malloc(sizeof(int) * scan->size);
    for (int i = 0; i < scan->size; i++) {
        tree.indices[i] = i;
    }
    tree.node_count = 0;
    tree.root = kd_build_recursive(scan, tree.indices, 0, scan->size, 0, tree.nodes, &tree.node_count);
    return tree;
}

static void free_kd_tree(KDTree *tree) {
    free(tree->nodes);
    free(tree->indices);
}

// static int nearest_neighbor(float node_x, float node_z, const PointCloud *reference_scan) {
//     float smallest_dist_sqr = FLT_MAX;
//     int smallest_dist_idx  = 0;
//     for (int i = 0; i < reference_scan->size; i++) {
//         float dx = reference_scan->data[i].position.x - node_x;
//         float dz = reference_scan->data[i].position.z - node_z;
//         float dist_sqr = dx * dx + dz * dz;
//         if (dist_sqr < smallest_dist_sqr) { 
//             smallest_dist_sqr = dist_sqr; 
//             smallest_dist_idx = i; 
//         }
//     }
//     return smallest_dist_idx;
// } // legacy


static void kd_nearest(const PointCloud *scan,
                       const KDNode *nodes,
                       int node_index,
                       float node_x,
                       float node_z,
                       int *best_idx,
                       float *best_dist_sqr) {

    if (node_index < 0) {
        return;
    }
    KDNode *node = &nodes[node_index];
    Vector3 *p = &scan->data[node->idx].position;
    float dx = p->x - node_x;
    float dz = p->z - node_z;
    float dist_sqr = dx * dx + dz * dz;

    if (dist_sqr < *best_dist_sqr) {
        *best_dist_sqr = dist_sqr;
        *best_idx = node->idx;
    }

    int near_child, far_child, diff;
    if (node->axis == 0){
        diff = dx;
        if (dx < 0.0f) {
            near_child = node->left;
            far_child = node->right;
        } else {
            near_child = node->right;
            far_child = node->left;
        }
    }
    else {
        diff = dz;
        if (dz < 0.0f) {
            near_child = node->left;
            far_child = node->right;
        } else {
            near_child = node->right;
            far_child = node->left;
        }
    }

    kd_nearest(scan, nodes, near_child, node_x, node_z, best_idx, best_dist_sqr);
    if (node->axis == 0) {
        if (dx * dx < *best_dist_sqr) {
            kd_nearest(scan, nodes, far_child, node_x, node_z, best_idx, best_dist_sqr);
        }
    }
    else {
        if (dz * dz < *best_dist_sqr) {
            kd_nearest(scan, nodes, far_child, node_x, node_z, best_idx, best_dist_sqr);
        }
    }
}

static int nearest_neighbor_kd(float node_x, float node_z, const KDTree *tree) {
    float best_dist_sqr = FLT_MAX;
    int best_idx = -1;
    kd_nearest(tree->scan, tree->nodes, tree->root, node_x, node_z, &best_idx, &best_dist_sqr);
    if (best_idx >= 0) {
        return best_idx;
    }
    fprintf(stderr, "Error: KDTree nearest neighbor search failed to find a neighbor\n");
    exit(1);
}


// TODO: couple of things to add here:
// - ICP assume stationary, so perhaps stationary scans must be used as reference
// - in that case, we may need to store PointCloud snapshots in some rolling buffer of most recent local PCs
// - we will just implement ICP directly
// - raw point cloud is expensive, may need to store spatial hash or something for fast NN search
// - we are working with a static scene so this is effective but in real autonomous rovers scenes are generally dynamic
//.  or at laeast slow moving

// https://learnopencv.com/iterative-closest-point-icp-explained/
ICPResult run_icp(const PointCloud *current_scan,   // source: current scan
                  const PointCloud *reference_scan, // target: reference scan
                  int max_iterations) {

    ICPResult result = {0};
    int n = current_scan->size;

    float *P[2] = {malloc(n * sizeof(float)), malloc(n * sizeof(float))};
    float *Q[2] = {malloc(n * sizeof(float)), malloc(n * sizeof(float))};

    // Seed P with the original source positions.
    for (int i = 0; i < n; i++) {
        P[0][i] = current_scan->data[i].position.x;
        P[1][i] = current_scan->data[i].position.z;
    }

    // T_total = (R, t), composes incremental transformations in form rotation R and then translation vector t
    float T_total[3] = {0.0f, 0.0f, 0.0f}; // d_theta, dx, dz
    T_total[0] = 0.0f;
    T_total[1] = 0.0f;
    T_total[2] = 0.0f;

    float prev_error = FLT_MAX;
    

    KDTree tree = build_kd_tree(reference_scan);

    for (int iter = 0; iter < max_iterations; ++iter) {

        // mu = centroids, used for centering point sets
        // error_sum used for convergence check
        // Q populated with nearest neighbours each iteration
        float mu_P[2] = {0.0f, 0.0f};
        float mu_Q[2] = {0.0f, 0.0f};
        float error_sum = 0.0f;

        for (int i = 0; i < n; i++) {
            int nn = nearest_neighbor_kd(P[0][i], P[1][i], &tree);
            Q[0][i] = reference_scan->data[nn].position.x;
            Q[1][i] = reference_scan->data[nn].position.z;

            float error_x = Q[0][i] - P[0][i];
            float error_z = Q[1][i] - P[1][i];
            float err = sqrtf(error_x * error_x + error_z * error_z);

            if (err > MAX_MATCH_DIST) {
                continue;
            }

            error_sum += err;
            mu_P[0] += P[0][i];
            mu_P[1] += P[1][i];
            mu_Q[0] += Q[0][i];
            mu_Q[1] += Q[1][i];
        }

        result.error = error_sum / (float)n;
        mu_P[0] /= n;
        mu_P[1] /= n;
        mu_Q[0] /= n;
        mu_Q[1] /= n;

        // Cross-covariance matrix H between centered point sets P and Q
        // Build H = Sigma_i(p_i q_i^T)
        // Recover rotation (arctan equivalent to SVD in 2D)
        //   θ = atan2(H01 - H10, H00 + H11)
        float H[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

        for (int i = 0; i < n; i++) {
            float P_centred[2] = {P[0][i] - mu_P[0], P[1][i] - mu_P[1]};
            float Q_centred[2] = {Q[0][i] - mu_Q[0], Q[1][i] - mu_Q[1]};
            float dx = Q_centred[0] - P_centred[0];
            float dz = Q_centred[1] - P_centred[1];
            if (sqrtf(dx * dx + dz * dz) > MAX_MATCH_DIST) {
                continue;
            }
            H[0][0] += P_centred[0] * Q_centred[0];
            H[0][1] += P_centred[0] * Q_centred[1];
            H[1][0] += P_centred[1] * Q_centred[0];
            H[1][1] += P_centred[1] * Q_centred[1];
        }

        float theta = atan2f(H[0][1] - H[1][0], H[0][0] + H[1][1]);
        float R[2][2] = {{ cosf(theta), -sinf(theta) },
                         { sinf(theta),  cosf(theta) }}; // build rotation matrix

        // translation vector, t
        // t = mu_Q - R * mu_P
        float t[2] = {
            mu_Q[0] - (R[0][0] * mu_P[0] + R[0][1] * mu_P[1]),
            mu_Q[1] - (R[1][0] * mu_P[0] + R[1][1] * mu_P[1])
        };

        // increental transformation
        for (int i = 0; i < n; i++) {
            float R_p_i[2] = {
                R[0][0] * P[0][i] + R[0][1] * P[1][i],
                R[1][0] * P[0][i] + R[1][1] * P[1][i]
            }; // apply rotation
            P[0][i] = R_p_i[0] + t[0];
            P[1][i] = R_p_i[1] + t[1];
        }

        // accumulate T_total = (R, t) with previous transformations
        // t is expressed in the current frame, so rotate back to original frame
        // as reference
        float R_total[2][2] = {{ cosf(T_total[0]), -sinf(T_total[0]) },
                               { sinf(T_total[0]),  cosf(T_total[0]) }};
        T_total[0] += theta;
        T_total[1] += R_total[0][0] * t[0] + R_total[0][1] * t[1];
        T_total[2] += R_total[1][0] * t[0] + R_total[1][1] * t[1];

        // convergence check
        if (fabsf(prev_error - result.error) < CONVERGENCE_THRESHOLD) {
            result.converged = 1;
            break;
        }
        prev_error = result.error;
    }

    free(P[0]);
    free(P[1]);
    free(Q[0]);
    free(Q[1]);
    free_kd_tree(&tree);

    result.delta_theta = T_total[0];
    result.dx = T_total[1];
    result.dz = T_total[2];

    return result;
}