/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SWARM_GRAPH_H
#define SWARM_GRAPH_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

typedef struct scan scan_t;

#define GRAPH_POSE_MAX_EDGES    8
#define GRAPH_MAX_POSES         64

typedef union {
    uint16_t raw;
    struct {
        uint16_t node_id: 12;
        uint16_t drone_id: 4;
    };
} swarm_pose_id_t;

typedef struct {
    float t_x, t_y, t_yaw;
    swarm_pose_id_t to;
} swarm_edge_t;

typedef struct {
    float x, y, yaw;
    swarm_pose_id_t id;
    uint8_t num_edges;
    swarm_edge_t edges[GRAPH_POSE_MAX_EDGES];
} swarm_pose_t;

void swarm_graph_init(void);

bool swarm_graph_has_pose(swarm_pose_id_t id);

void swarm_graph_store_pose(const swarm_pose_t *pose);

bool swarm_graph_load_pose(swarm_pose_t *pose);

bool swarm_graph_has_scan(swarm_pose_id_t pose_id);

void swarm_graph_store_scan(const scan_t *scan);

bool swarm_graph_load_scan(scan_t *scan);

size_t swarm_graph_get_pose_count(void);

void swarm_graph_debug_print(void);

#endif // SWARM_GRAPH_H
