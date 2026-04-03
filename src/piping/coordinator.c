#include "piping/method_dispatcher.h"
#include "piping/messages.h"
#include <string.h>


#include <unistd.h>

void run_coordinator_loop(int scan_cmd_read_fd, int ray_batch_writes_fd,
                          int ray_task_pipes[NUM_WORKERS][2], int ray_results_pipes[NUM_WORKERS][2],
                          int point_batch_write_fd) {
    ScanRequest scan_request;
    int rings_per_worker = NUM_RINGS / NUM_WORKERS;

    while (read(scan_cmd_read_fd, &scan_request, sizeof(ScanRequest)) > 0) {
        RayBatch ray_batches[NUM_WORKERS];
        for (int i = 0; i < NUM_WORKERS; i++) {
            ray_batches[i].origin = scan_request.origin;
            ray_batches[i].theta = scan_request.theta;
            ray_batches[i].start_ray_idx = i * rings_per_worker;
            ray_batches[i].end_ray_idx = (i + 1) * rings_per_worker;
            ray_batches[i].num_rays = rings_per_worker;
            write(ray_task_pipes[i][1], &ray_batches[i], sizeof(RayBatch));
        }

        int workers_done = 0;
        int done[NUM_WORKERS] = {0};

        while (workers_done < NUM_WORKERS) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            int max_fd = -1;
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i]) {
                    FD_SET(ray_results_pipes[i][0], &read_fds);
                    max_fd = ray_results_pipes[i][0] > max_fd ? ray_results_pipes[i][0] : max_fd;
                }
            }
            select(max_fd + 1, &read_fds, NULL, NULL, NULL);
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i] && FD_ISSET(ray_results_pipes[i][0], &read_fds)) {
                    RayResultBatch ray_result_batch;
                    read(ray_results_pipes[i][0], &ray_result_batch, sizeof(RayResultBatch));
                    write(ray_batch_writes_fd, &ray_result_batch, sizeof(RayResultBatch));
                    write(point_batch_write_fd, &ray_result_batch, sizeof(RayResultBatch));
                    done[i] = 1;
                    workers_done++;
                }
            }
        }
    }
}

void run_mppi_coordinator_loop(int mppi_cmd_read_fd,
                               int mppi_result_write_fd,
                               int mppi_task_pipes[NUM_WORKERS][2],
                               int mppi_result_pipes[NUM_WORKERS][2]) {
    MppiEvalRequest request;
    while (read(mppi_cmd_read_fd, &request, sizeof(MppiEvalRequest)) > 0) {
        int samples_per_worker = request.sample_count / NUM_WORKERS;
        // assumption of divisibility, and losing samples is fine
        int remainder = request.sample_count % NUM_WORKERS;
        int start = 0;

        for (int i = 0; i < NUM_WORKERS; i++) {
            MppiWorkerJob job = {
                .request = request,
                .frame_id = request.frame_id,
                .start_sample_idx = start,
                .end_sample_idx = start + samples_per_worker
            };

            write(mppi_task_pipes[i][1], &job, sizeof(MppiWorkerJob));
            start += samples_per_worker;
        }

        MppiEvalResult merged;
        merged.frame_id = request.frame_id;
        memset(merged.costs, 0, sizeof(merged.costs));

        int workers_done = 0;
        int done[NUM_WORKERS] = {0};
        while (workers_done < NUM_WORKERS) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            int max_fd = -1;
            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i]) {
                    FD_SET(mppi_result_pipes[i][0], &read_fds);
                    if (mppi_result_pipes[i][0] > max_fd) {
                        max_fd = mppi_result_pipes[i][0];
                    }
                }
            }

            select(max_fd + 1, &read_fds, NULL, NULL, NULL);

            for (int i = 0; i < NUM_WORKERS; i++) {
                if (!done[i] && FD_ISSET(mppi_result_pipes[i][0], &read_fds)) {
                    MppiWorkerResult batch;
                    if (read(mppi_result_pipes[i][0], &batch, sizeof(MppiWorkerResult)) <= 0) {
                        continue;
                    }
                    for (int s = batch.start_sample_idx; s < batch.end_sample_idx; s++) {
                        merged.costs[s] = batch.costs[s];
                    }
                    done[i] = 1;
                    workers_done++;
                }
            }
        }

        write(mppi_result_write_fd, &merged, sizeof(MppiEvalResult));
    }
}