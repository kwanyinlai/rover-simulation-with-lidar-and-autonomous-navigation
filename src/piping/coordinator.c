#include "piping/method_dispatcher.h"
#include "piping/messages.h"
#include "core/io_utils.h"
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>

void run_coordinator_loop(int scan_cmd_read_fd, int ray_batch_writes_fd,
                          /*int scan_match_cmd_read_fd, int scan_match_result_write_fd,*/
                          int ray_task_pipes[NUM_WORKERS][2], 
                          int ray_results_pipes[NUM_WORKERS][2],
                          int point_batch_write_fd) {

    ScanRequest scan_request;
    int rings_per_worker = NUM_RINGS / NUM_WORKERS;
    int max_cmd_fd = scan_cmd_read_fd + 1;
    fd_set read_fds;

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(scan_cmd_read_fd, &read_fds);

        if (select(max_cmd_fd, &read_fds, NULL, NULL, NULL) < 0) {
            perror("select coordinator command pipes");
            break;
        }

        int cmd_fd = scan_cmd_read_fd;
        int result_fd = ray_batch_writes_fd;
        if (!FD_ISSET(scan_cmd_read_fd, &read_fds)) {
            continue;
        }

        if (read_exact(cmd_fd, &scan_request, sizeof(ScanRequest)) <= 0) {
            break;
        }

        RayBatch ray_batches[NUM_WORKERS];
        for (int i = 0; i < NUM_WORKERS; i++) {
            ray_batches[i].origin = scan_request.origin;
            ray_batches[i].theta = scan_request.theta;
            ray_batches[i].start_ray_idx = i * rings_per_worker;
            ray_batches[i].end_ray_idx = (i + 1) * rings_per_worker;
            ray_batches[i].num_rays = rings_per_worker;
            if (write_all(ray_task_pipes[i][1], &ray_batches[i], sizeof(RayBatch)) < 0) {
                exit(1);
            }
        }

        int workers_done = 0;
        int done[NUM_WORKERS] = {0};
        while (workers_done < NUM_WORKERS) {
            FD_ZERO(&read_fds);
            int max_fd = -1;
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i]) {
                    FD_SET(ray_results_pipes[i][0], &read_fds);
                    if (ray_results_pipes[i][0] > max_fd) max_fd = ray_results_pipes[i][0];
                }
            }
            if (max_fd < 0) { fprintf(stderr, "no worker pipes\n"); exit(1); }
            if (select(max_fd + 1, &read_fds, NULL, NULL, NULL) < 0) {
                perror("select worker results"); exit(1);
            }
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i] && FD_ISSET(ray_results_pipes[i][0], &read_fds)) {
                    RayResultBatch batch;
                    if (read_exact(ray_results_pipes[i][0], &batch, sizeof(RayResultBatch)) <= 0) exit(1);
                    
                    if (write_all(result_fd, &batch, sizeof(RayResultBatch)) < 0) exit(1);
                    
                    if (write_all(point_batch_write_fd, &batch, sizeof(RayResultBatch)) < 0) exit(1);
                    
                    done[i] = 1;
                    workers_done++;
                }
            }
        }
    }
}

void run_rollout_coordinator_loop(int rollout_cmd_read_fd,
                                  int rollout_result_write_fd,
                                  int rollout_task_pipes[NUM_WORKERS][2],
                                  int rollout_costs_pipes[NUM_WORKERS][2]) {
    RolloutRequest request;
    while (1) {
        int cmd_read = read_exact(rollout_cmd_read_fd, &request, sizeof(RolloutRequest));
        if (cmd_read == 0) {
            break;
            // closed pipe
        }
        if (cmd_read < 0) {
            exit(1);
        }
        int samples_per_worker = request.sample_count / NUM_WORKERS;
        // assumption of divisibility, and losing samples is fine
        int start = 0;

        for (int i = 0; i < NUM_WORKERS; i++) {
            RolloutJob job = {
                .request = request,
                .start_sample_idx = start,
                .end_sample_idx = start + samples_per_worker
            };

            if (write_all(rollout_task_pipes[i][1], &job, sizeof(RolloutJob)) < 0) {
                exit(1);
            }
            start += samples_per_worker;
        }

        BatchedRolloutResult merged;
        memset(merged.costs, 0, sizeof(merged.costs));

        int workers_done = 0;
        int done[NUM_WORKERS] = {0};
        while (workers_done < NUM_WORKERS) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            int max_fd = -1;
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i]) {
                    FD_SET(rollout_costs_pipes[i][0], &read_fds);
                    if (rollout_costs_pipes[i][0] > max_fd) {
                        max_fd = rollout_costs_pipes[i][0];
                    }
                }
            }
            if (max_fd < 0) {
                fprintf(stderr, "no rollout cost pipes to read from\n");
                exit(1);
            }

            if (select(max_fd + 1, &read_fds, NULL, NULL, NULL) < 0) {
                perror("select rollout costs");
                exit(1);
            }

            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i] && FD_ISSET(rollout_costs_pipes[i][0], &read_fds)) {
                    RolloutResult batch;
                    int got = read_exact(rollout_costs_pipes[i][0], &batch, sizeof(RolloutResult));
                    if (got <= 0) {
                        if (got == 0) {
                            fprintf(stderr, "rollout cost pipe closed unexpectedly\n");
                        }
                        exit(1);
                    }
                    for (int s = batch.start_sample_idx; s < batch.end_sample_idx; s++) {
                        merged.costs[s] = batch.costs[s];
                    }
                    done[i] = 1;
                    workers_done++;
                }
            }
        }

        if (write_all(rollout_result_write_fd, &merged, sizeof(BatchedRolloutResult)) < 0) {
            exit(1);
        }
    }
}
