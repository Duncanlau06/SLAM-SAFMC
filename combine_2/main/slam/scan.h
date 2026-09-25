/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SCAN_H
#define SCAN_H

#include <stdint.h>
#include <stdbool.h>

#include "swarm_graph.h"
#include "icp/icp.h"
#include "tof_task.h"

/* Every ToF sensor is used (ETH used 4 and rotated the drone 45° per scan to
 * fill the gaps). Slot k holds sensor (TOF_FRONT_SENSOR_IDX + k) % count,
 * which points k * (360 / count) degrees CCW (to the left) of the nose. */
#define SCAN_NUM_DIRS   TOF_SENSOR_COUNT

#define SCAN_NUM_FRAMES 5
#define POINTS_IN_SCAN  (SCAN_NUM_FRAMES * SCAN_NUM_DIRS * 8)
#define SCAN_TIME_MS    400
/* ~20% of the logged points were at 0.2-0.4 m (prop guards / chassis). */
#define SCAN_MIN_VALID_MM 300

typedef struct {
    int16_t dists[SCAN_NUM_DIRS][8];
    float dx, dy, dyaw;
} scan_frame_t;

typedef struct scan {
    swarm_pose_id_t pose;
    scan_frame_t frames[SCAN_NUM_FRAMES];
} scan_t;

void scan_extract_points(const scan_t *scan, float x, float y, float yaw,
                         icp_points_t *points);

void scan_acquire(scan_t *scan);

#endif // SCAN_H
