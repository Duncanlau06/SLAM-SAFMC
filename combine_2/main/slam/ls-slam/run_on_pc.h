/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef RUN_ON_PC_H
#define RUN_ON_PC_H

#include <stdint.h>
#include <stdio.h>

#ifdef OFFDRONE
#ifndef DEBUG_PRINT
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#endif
#else
#include "esp_log.h"
#ifndef DEBUG_PRINT
#define DEBUG_PRINT(...) ESP_LOGD("SLAM", __VA_ARGS__)
#endif
#endif

#endif // RUN_ON_PC_H
