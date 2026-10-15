/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SLAM_UTIL_H
#define SLAM_UTIL_H

#include <stdint.h>
#include <stddef.h>

#ifndef MIN
#define MIN(a, b)   ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)   ((a) > (b) ? (a) : (b))
#endif

#ifndef SIZEOFARRAY
#define SIZEOFARRAY(x)  (sizeof(x) / sizeof((x)[0]))
#endif

int compare_int16(const void *a, const void *b);

#endif // SLAM_UTIL_H
