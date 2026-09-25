/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef NODE_H
#define NODE_H

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    int16_t index;
    int16_t neighbors;
    int8_t visited;
} node;

#endif // NODE_H
