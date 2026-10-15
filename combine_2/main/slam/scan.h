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
/* Beams are ~30 cm wide at 3 m; far readings were the noisiest. Separate from
 * TOF_MAX_VALID_MM, which obstacle avoidance uses. */
#define SCAN_MAX_VALID_MM 2500
/* Drop a sensor frame measured this long before the pose it is attached to */
#define SCAN_MAX_FRAME_AGE_MS 100
/* ...or if the drone turned more than this (~3°, half a column) since then */
#define SCAN_MAX_YAW_ERR_RAD 0.05f
/* Centre rows that must be valid and agree for a column to count, which
 * rejects mixed pixels where a beam straddles an edge */
#define SCAN_MIN_ROWS 2
#define SCAN_MAX_ROW_SPREAD_MM 100

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
