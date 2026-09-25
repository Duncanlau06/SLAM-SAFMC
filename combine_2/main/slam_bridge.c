/**
 * @file slam_bridge.c
 * @brief Implementation of the bridge between ESP32 tasks and ETH Zurich SLAM.
 */

#include "slam_bridge.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "mavlink_task.h"
#include "slam.h"
#include "swarm.h"
#include "swarm_graph.h"
#include "scan.h"

static const char *TAG = "slam_bridge";

static SemaphoreHandle_t s_bridge_mutex = NULL;
static StaticSemaphore_t s_bridge_mutex_buffer;
static uint16_t s_node_idx = 0;
/* Pre-optimisation odometry poses, exported so the plotter can show what
 * SLAM actually changed. */
static float s_raw_poses[GRAPH_MAX_POSES][3];

void slam_bridge_init(void) {
    if (s_bridge_mutex == NULL) {
        s_bridge_mutex = xSemaphoreCreateMutexStatic(&s_bridge_mutex_buffer);
        s_node_idx = 0;
        swarm_init();
        swarm_graph_init();
        slam_init();
        ESP_LOGI(TAG, "SLAM Bridge initialized for Swarm ID %d (Max %d poses in RAM)",
                 swarm_id, GRAPH_MAX_POSES);
    }
}

bool slam_bridge_record_pose(float x_m, float y_m, float yaw_rad) {
    if (s_bridge_mutex == NULL) {
        slam_bridge_init();
    }

    xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);

    if (s_node_idx >= GRAPH_MAX_POSES) {
        ESP_LOGW(TAG, "SLAM pose graph full (%d poses)", GRAPH_MAX_POSES);
        xSemaphoreGive(s_bridge_mutex);
        return false;
    }

    swarm_pose_t p = {
        .id = {
            .drone_id = swarm_id,
            .node_id = s_node_idx++,
        },
        .x = x_m,
        .y = y_m,
        .yaw = yaw_rad,
        .num_edges = 0,
    };

    swarm_graph_store_pose(&p);
    s_raw_poses[p.id.node_id][0] = x_m;
    s_raw_poses[p.id.node_id][1] = y_m;
    s_raw_poses[p.id.node_id][2] = yaw_rad;

    scan_t sc;
    sc.pose = p.id;
    scan_acquire(&sc);

    ESP_LOGI(TAG, "Stored SLAM pose [%d:%d] -> x=%.2f, y=%.2f, yaw=%.2f rad",
             p.id.drone_id, p.id.node_id, p.x, p.y, p.yaw);

    xSemaphoreGive(s_bridge_mutex);
    return true;
}

bool slam_bridge_record_step(void) {
    drone_state_t st = mavlink_get_state();
    return slam_bridge_record_pose(st.x, st.y, st.heading);
}

void slam_bridge_run_optimization(void) {
    ESP_LOGI(TAG, "Starting post-flight Graph LS-SLAM optimization...");
    uint64_t t_start_us = esp_timer_get_time();

    run_slam();

    uint64_t elapsed_ms = (esp_timer_get_time() - t_start_us) / 1000;
    ESP_LOGI(TAG, "Graph LS-SLAM finished in %llu ms", (unsigned long long)elapsed_ms);
}

void slam_bridge_export_json(void) {
    ESP_LOGI(TAG, "Exporting SLAM Map JSON to console...");

    /* Printed before the map JSON: the live plotter plots on SLAM_MAP_JSON_END */
    printf("\n=== SLAM_RAW_POSES_START ===\n[");
    for (uint16_t i = 0; i < s_node_idx; i++) {
        printf("%s[%.3f,%.3f,%.3f]", i ? "," : "",
               s_raw_poses[i][0], s_raw_poses[i][1], s_raw_poses[i][2]);
    }
    printf("]\n=== SLAM_RAW_POSES_END ===\n");
    fflush(stdout);

    swarm_graph_debug_print();
}

size_t slam_bridge_get_pose_count(void) {
    return swarm_graph_get_pose_count();
}

bool slam_bridge_is_optimizing(void) {
    return slam_is_optimizing();
}
