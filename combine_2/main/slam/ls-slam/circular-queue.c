/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include <stdio.h>
#include <string.h>

#include "circular-queue.h"

int16_t init_queue(circularQueue *queue) {
    memset(queue, 0, sizeof(circularQueue));
    return 1;
}

int16_t is_full(circularQueue *queue) {
    return (queue->size >= MAX_SIZE) ? 1 : 0;
}

int16_t is_empty(circularQueue *queue) {
    return (queue->size == 0) ? 1 : 0;
}

int16_t push_to_queue(circularQueue *queue, node element) {
    if (is_full(queue))
        return 0;

    queue->nodes[queue->tail] = element;
    queue->tail = (queue->tail + 1) % MAX_SIZE;
    queue->size++;

    return 1;
}

int16_t pop_from_queue(circularQueue *queue, node *element) {
    if (is_empty(queue))
        return 0;

    *element = queue->nodes[queue->head];
    queue->head = (queue->head + 1) % MAX_SIZE;
    queue->size--;

    return 1;
}
