#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <stddef.h>
#include <stdio.h>
#include <unistd.h>

static inline int read_exact(int fd, void *buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t n = read(fd, (char *)buf + total, len - total);
        if (n < 0) {
            perror("read");
            return -1;
        }
        if (n == 0) {
            return 0;
        }
        total += (size_t)n;
    }
    return 1;
}

static inline int write_exact(int fd, const void *buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t n = write(fd, (const char *)buf + total, len - total);
        if (n < 0) {
            perror("write");
            return -1;
        }
        if (n == 0) {
            fprintf(stderr, "write returned 0\n");
            return -1;
        }
        total += (size_t)n;
    }
    return 1;
}

#endif
