/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "swarm_graph.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "icp/icp.h"
#include "scan.h"

static const char *TAG = "swarm_graph";

static SemaphoreHandle_t mutex = NULL;
static StaticSemaphore_t mutex_buffer;

static size_t num_poses = 0;
static swarm_pose_t poses[GRAPH_MAX_POSES];

static size_t num_scans = 0;
static scan_t scans[GRAPH_MAX_POSES];

void swarm_graph_init(void) {
    if (mutex == NULL) {
        mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
        num_poses = 0;
        num_scans = 0;
        memset(poses, 0, sizeof(poses));
        memset(scans, 0, sizeof(scans));
        ESP_LOGI(TAG, "Swarm graph initialized (max %d poses/scans in RAM)", GRAPH_MAX_POSES);
    }
}

bool swarm_graph_has_pose(swarm_pose_id_t id) {
    if (mutex == NULL) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (size_t i = 0; i < num_poses; i++) {
        if (poses[i].id.raw == id.raw) {
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

void swarm_graph_store_pose(const swarm_pose_t *pose) {
    if (mutex == NULL) swarm_graph_init();
    xSemaphoreTake(mutex, portMAX_DELAY);

    bool found = false;
    for (size_t i = 0; i < num_poses; i++) {
        if (poses[i].id.raw == pose->id.raw) {
            poses[i] = *pose;
            found = true;
            break;
        }
    }

    if (!found) {
        if (num_poses < GRAPH_MAX_POSES) {
            poses[num_poses++] = *pose;
        } else {
            ESP_LOGW(TAG, "Graph pose capacity reached (%d)", GRAPH_MAX_POSES);
        }
    }

    xSemaphoreGive(mutex);
}

bool swarm_graph_load_pose(swarm_pose_t *pose) {
    if (mutex == NULL) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);

    for (size_t i = 0; i < num_poses; i++) {
        if (poses[i].id.raw == pose->id.raw) {
            *pose = poses[i];
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

bool swarm_graph_has_scan(swarm_pose_id_t pose_id) {
    if (mutex == NULL) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);

    for (size_t i = 0; i < num_scans; i++) {
        if (scans[i].pose.raw == pose_id.raw) {
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

void swarm_graph_store_scan(const scan_t *scan) {
    if (mutex == NULL) swarm_graph_init();
    xSemaphoreTake(mutex, portMAX_DELAY);

    bool found = false;
    for (size_t i = 0; i < num_scans; i++) {
        if (scans[i].pose.raw == scan->pose.raw) {
            scans[i] = *scan;
            found = true;
            break;
        }
    }

    if (!found) {
        if (num_scans < GRAPH_MAX_POSES) {
            scans[num_scans++] = *scan;
        } else {
            ESP_LOGW(TAG, "Scan storage capacity reached (%d)", GRAPH_MAX_POSES);
        }
    }

    xSemaphoreGive(mutex);
}

bool swarm_graph_load_scan(scan_t *scan) {
    if (mutex == NULL) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);

    for (size_t i = 0; i < num_scans; i++) {
        if (scans[i].pose.raw == scan->pose.raw) {
            *scan = scans[i];
            xSemaphoreGive(mutex);
            return true;
        }
    }
    xSemaphoreGive(mutex);
    return false;
}

size_t swarm_graph_get_pose_count(void) {
    if (mutex == NULL) return 0;
    xSemaphoreTake(mutex, portMAX_DELAY);
    size_t count = num_poses;
    xSemaphoreGive(mutex);
    return count;
}

void swarm_graph_debug_print(void) {
    if (mutex == NULL) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "=== SWARM GRAPH (Poses: %d, Scans: %d) ===", (int)num_poses, (int)num_scans);
    for (size_t i = 0; i < num_poses; i++) {
        ESP_LOGI(TAG, "Pose [%d:%d] -> x=%.2f, y=%.2f, yaw=%.2f (edges=%d)",
                 poses[i].id.drone_id, poses[i].id.node_id,
                 poses[i].x, poses[i].y, poses[i].yaw, poses[i].num_edges);
        for (int j = 0; j < poses[i].num_edges; j++) {
            ESP_LOGI(TAG, "   edge -> [%d:%d] rel=(%.2f, %.2f, %.2f)",
                     poses[i].edges[j].to.drone_id, poses[i].edges[j].to.node_id,
                     poses[i].edges[j].t_x, poses[i].edges[j].t_y, poses[i].edges[j].t_yaw);
        }
    }
    ESP_LOGI(TAG, "==========================================");

    /* Export structured JSON for Python map visualizer (using clean printf to avoid log header bloat) */
    printf("\n=== SLAM_MAP_JSON_START ===\n");
    printf("{\"poses\":[\n");
    for (size_t i = 0; i < num_poses; i++) {
        printf("  {\"id\":%d,\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f}%s\n",
               poses[i].id.node_id, poses[i].x, poses[i].y, poses[i].yaw,
               (i + 1 < num_poses) ? "," : "");
    }
    printf("],\"scans\":[\n");
#if SLAM_EXPORT_POINTS
    POINTS_STATIC_ALLOC(cloud, POINTS_IN_SCAN)
    for (size_t i = 0; i < num_scans; i++) {
        float px = 0.0f, py = 0.0f, pyaw = 0.0f;
        for (size_t k = 0; k < num_poses; k++) {
            if (poses[k].id.raw == scans[i].pose.raw) {
                px = poses[k].x;
                py = poses[k].y;
                pyaw = poses[k].yaw;
                break;
            }
        }
        scan_extract_points(&scans[i], px, py, pyaw, cloud);
        printf("  {\"pose_id\":%d,\"pts\":[\n", scans[i].pose.node_id);
        for (size_t p = 0; p < cloud->num; p++) {
            printf("    [%.3f,%.3f]%s\n",
                   cloud->items[p].x, cloud->items[p].y,
                   (p + 1 < cloud->num) ? "," : "");
        }
        printf("  ]}%s\n", (i + 1 < num_scans) ? "," : "");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(5)); /* Yield to FreeRTOS IDLE task to prevent task watchdog triggers */
    }
#endif
    printf("]}\n");
    printf("=== SLAM_MAP_JSON_END ===\n");
    fflush(stdout);

    xSemaphoreGive(mutex);
}
