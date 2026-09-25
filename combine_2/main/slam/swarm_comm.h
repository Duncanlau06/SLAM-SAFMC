/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SWARM_COMM_H
#define SWARM_COMM_H

#include <stdint.h>
#include <stddef.h>

#include "swarm_graph.h"

#define COMM_MSG_TAG_POSE       0
#define COMM_MSG_TAG_SCAN_REQ   1
#define COMM_MSG_TAG_SCAN_RES   2
#define COMM_MSG_TAG_CTRL       3

#define RADIO_ADR_BROADCAST     0xFF
#define RADIO_ADR_BRIDGE        0xFE

typedef struct {
    enum {
        CTRL_MSG_TYPE_START = 0,
        CTRL_MSG_TYPE_DONE,
        CTRL_MSG_TYPE_POSE,
    } type;
    swarm_pose_id_t pose;
} swarm_comm_ctrl_msg_t;

void swarm_comm_init(void);

void swarm_comm_broadcast_pose(swarm_pose_id_t id);

void swarm_comm_fetch_scan(uint8_t addr, swarm_pose_id_t id);

void swarm_comm_send_scan(uint8_t addr, swarm_pose_id_t id);

void swarm_comm_send_ctrl_msg(uint8_t addr, swarm_comm_ctrl_msg_t msg);

void swarm_comm_broadcast_ctrl_msg(swarm_comm_ctrl_msg_t msg);

void swarm_comm_handle_msg(uint8_t src, uint8_t tag, const uint8_t *data, size_t len);

#endif // SWARM_COMM_H
