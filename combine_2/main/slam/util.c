/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "util.h"

int compare_int16(const void *a, const void *b) {
    return (*(const int16_t *) a - *(const int16_t *) b);
}
