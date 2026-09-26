/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SLAM_H
#define SLAM_H

#include <stdbool.h>
#include <stddef.h>

#include "swarm_graph.h"

void slam_init(void);

/* Runs ICP constraints + LS-SLAM on the poses currently in the graph and
 * writes the optimised result to `poses`, without touching the graph.
 * Returns the number of poses optimised (0 if there were too few).
 * Not re-entrant; slam_bridge serialises calls. */
size_t slam_optimize(float poses[GRAPH_MAX_POSES][3]);

/* Writes optimised poses 0..n-1 back into the graph and broadcasts them. */
void slam_commit_poses(const float poses[GRAPH_MAX_POSES][3], size_t n);

bool slam_is_optimizing(void);

#endif // SLAM_H
