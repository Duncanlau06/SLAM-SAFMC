/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "swarm_comm.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "swarm.h"
#include "swarm_graph.h"
#include "scan.h"

static const char *TAG = "swarm_comm";

typedef struct {
    enum {
        COMM_TX_QUEUE_ITEM_POSE,
        COMM_TX_QUEUE_ITEM_SCAN_REQ,
        COMM_TX_QUEUE_ITEM_SCAN_RES,
        COMM_TX_QUEUE_ITEM_CTRL,
    } type;
    uint8_t addr;
    union {
        swarm_pose_id_t pose_id;
        swarm_comm_ctrl_msg_t ctrl_msg;
    };
} tx_queue_item_t;

static QueueHandle_t tx_queue = NULL;
static SemaphoreHandle_t scan_mutex = NULL;
static StaticSemaphore_t scan_mutex_buffer;
static SemaphoreHandle_t scan_semaphore = NULL;
static StaticSemaphore_t scan_semaphore_buf;
static swarm_pose_id_t scan_req_id;

static void swarm_comm_tx_task(void *parameters);

void swarm_comm_init(void) {
    if (scan_mutex == NULL) {
        scan_mutex = xSemaphoreCreateMutexStatic(&scan_mutex_buffer);
        scan_semaphore = xSemaphoreCreateBinaryStatic(&scan_semaphore_buf);
        tx_queue = xQueueCreate(16, sizeof(tx_queue_item_t));

        xTaskCreatePinnedToCore(swarm_comm_tx_task, "swarm_tx", 4096, NULL, 2, NULL, 0);
        ESP_LOGI(TAG, "Swarm comm initialized");
    }
}

void swarm_comm_broadcast_pose(swarm_pose_id_t id) {
    if (!tx_queue) return;
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_POSE,
            .pose_id = id,
    };
    xQueueSend(tx_queue, &item, pdMS_TO_TICKS(100));
}

void swarm_comm_fetch_scan(uint8_t addr, swarm_pose_id_t id) {
    if (!scan_mutex || !tx_queue) return;
    scan_req_id = id;
    xSemaphoreTake(scan_mutex, portMAX_DELAY);
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_SCAN_REQ,
            .addr = addr,
            .pose_id = id,
    };
    xQueueSend(tx_queue, &item, portMAX_DELAY);
    // Wait for scan to be returned with timeout (2 sec)
    xSemaphoreTake(scan_semaphore, pdMS_TO_TICKS(2000));
    xSemaphoreGive(scan_mutex);
}

void swarm_comm_send_scan(uint8_t addr, swarm_pose_id_t id) {
    if (!tx_queue) return;
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_SCAN_RES,
            .addr = addr,
            .pose_id = id,
    };
    xQueueSend(tx_queue, &item, pdMS_TO_TICKS(100));
}

void swarm_comm_send_ctrl_msg(uint8_t addr, swarm_comm_ctrl_msg_t msg) {
    if (!tx_queue) return;
    tx_queue_item_t item = {
            .type = COMM_TX_QUEUE_ITEM_CTRL,
            .addr = addr,
            .ctrl_msg = msg,
    };
    xQueueSend(tx_queue, &item, pdMS_TO_TICKS(100));
}

void swarm_comm_broadcast_ctrl_msg(swarm_comm_ctrl_msg_t msg) {
    swarm_comm_send_ctrl_msg(RADIO_ADR_BROADCAST, msg);
}

static void handle_tx_queue_item(tx_queue_item_t *item) {
    switch (item->type) {
        case COMM_TX_QUEUE_ITEM_POSE: {
            swarm_pose_t pose = {
                    .id = item->pose_id,
            };
            if (swarm_graph_load_pose(&pose)) {
                ESP_LOGD(TAG, "TX Broadcast Pose [%d:%d]: (%.2f, %.2f, %.2f)",
                         pose.id.drone_id, pose.id.node_id, pose.x, pose.y, pose.yaw);
            }
            break;
        }
        case COMM_TX_QUEUE_ITEM_SCAN_REQ: {
            ESP_LOGD(TAG, "TX Request Scan [%d:%d] from addr %d",
                     item->pose_id.drone_id, item->pose_id.node_id, item->addr);
            break;
        }
        case COMM_TX_QUEUE_ITEM_SCAN_RES: {
            scan_t scan = {
                    .pose = item->pose_id,
            };
            if (swarm_graph_has_scan(item->pose_id)) {
                swarm_graph_load_scan(&scan);
                ESP_LOGD(TAG, "TX Scan Data [%d:%d] to addr %d",
                         scan.pose.drone_id, scan.pose.node_id, item->addr);
            }
            break;
        }
        case COMM_TX_QUEUE_ITEM_CTRL: {
            ESP_LOGD(TAG, "TX Control message type %d to addr %d",
                     item->ctrl_msg.type, item->addr);
            break;
        }
    }
}

static void swarm_comm_tx_task(void *parameters) {
    tx_queue_item_t item;
    while (true) {
        if (xQueueReceive(tx_queue, &item, portMAX_DELAY)) {
            handle_tx_queue_item(&item);
        }
    }
}

void swarm_comm_handle_msg(uint8_t src, uint8_t tag, const uint8_t *data, size_t len) {
    switch (tag) {
        case COMM_MSG_TAG_POSE: {
            if (len == sizeof(swarm_pose_t)) {
                swarm_graph_store_pose((const swarm_pose_t *) data);
            }
            break;
        }
        case COMM_MSG_TAG_SCAN_REQ: {
            if (len == sizeof(swarm_pose_id_t)) {
                swarm_comm_send_scan(src, *(const swarm_pose_id_t *) data);
            }
            break;
        }
        case COMM_MSG_TAG_SCAN_RES: {
            if (len == sizeof(scan_t)) {
                const scan_t *scan = (const scan_t *) data;
                swarm_graph_store_scan(scan);
                if (scan_req_id.raw == scan->pose.raw && scan_semaphore) {
                    xSemaphoreGive(scan_semaphore);
                }
            }
            break;
        }
        case COMM_MSG_TAG_CTRL: {
            if (len == sizeof(swarm_comm_ctrl_msg_t)) {
                ESP_LOGI(TAG, "Received CTRL msg from src %d", src);
            }
            break;
        }
    }
}
