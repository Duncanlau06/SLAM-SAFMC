/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SLAM_H
#define SLAM_H

#include <stdbool.h>

void slam_init(void);

void run_slam(void);

void slam_trigger_optimization(void);

bool slam_is_optimizing(void);

#endif // SLAM_H
