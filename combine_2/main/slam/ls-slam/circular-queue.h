/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>
#include "node.h"
#include "config_size.h"

typedef struct {
    node nodes[MAX_SIZE];
    int16_t head;
    int16_t tail;
    int16_t size;
} circularQueue;

int16_t init_queue(circularQueue *queue);
int16_t is_full(circularQueue *queue);
int16_t is_empty(circularQueue *queue);
int16_t push_to_queue(circularQueue *queue, node element);
int16_t pop_from_queue(circularQueue *queue, node *element);

#endif // CIRCULAR_QUEUE_H
