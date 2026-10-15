/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SWARM_H
#define SWARM_H

#include <stdint.h>

#define SWARM_NUM_DRONES    4                       // max supported active drones in swarm
#define SWARM_NUM_DEVICES   (SWARM_NUM_DRONES + 1)  // drones + base station coordinator

extern uint8_t swarm_id;

void swarm_init(void);

#endif // SWARM_H
