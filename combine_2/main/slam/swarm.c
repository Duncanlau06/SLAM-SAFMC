/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "swarm.h"
#include "esp_log.h"

#ifdef CONFIG_DRONE_ID
#define DEFAULT_DRONE_ID CONFIG_DRONE_ID
#else
#define DEFAULT_DRONE_ID 0
#endif

uint8_t swarm_id = DEFAULT_DRONE_ID;

static const char *TAG = "swarm";

void swarm_init(void) {
#ifdef CONFIG_DRONE_ID
    swarm_id = (uint8_t) CONFIG_DRONE_ID;
#else
    swarm_id = 0;
#endif
    ESP_LOGI(TAG, "Swarm node initialized with Drone ID: %d", swarm_id);
}
