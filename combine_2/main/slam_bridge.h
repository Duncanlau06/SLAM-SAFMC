/**
 * @file slam_bridge.h
 * @brief Bridge interface between ESP32 hardware/tasks and ETH Zurich SLAM.
 *
 * This module cleanly decouples the FreeRTOS drone flight tasks
 * (mavlink_task, tof_task, odom, nav_task) from the pure mathematical
 * graph SLAM engine imported from ETH Zurich.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise SLAM subsystem, mutexes, and pose graph memory.
 * Must be called in app_main before any flight tasks record scans.
 */
void slam_bridge_init(void);

/**
 * @brief Record a single SLAM pose vertex and 360-degree ToF point cloud.
 * Samples current PX4 odometry (x, y, yaw), stores a new node in swarm_graph,
 * and captures an integrated scan from all 4 directional ToF sensors.
 *
 * @return true if node and scan were successfully stored, false if graph full.
 */
bool slam_bridge_record_step(void);

/**
 * @brief Record a SLAM pose vertex with explicitly provided coordinates.
 * Every SLAM_INFLIGHT_EVERY_N poses this also starts a background
 * optimisation on core 0.
 *
 * @param x_m North coordinate in meters (jump-free odometry, NED frame)
 * @param y_m East coordinate in meters (jump-free odometry, NED frame)
 * @param yaw_rad Heading in radians (clockwise from North, [0, 2*pi))
 * @return true if node and scan were successfully stored, false if graph full.
 */
bool slam_bridge_record_pose(float x_m, float y_m, float yaw_rad);

/**
 * @brief Execute the full Graph-Based Least Squares SLAM optimization.
 * Runs ICP point matching between overlapping scans, computes geometric
 * constraints, and solves H * dx = -b via Cholesky decomposition.
 * Call this post-flight upon landing or at major exploration checkpoints.
 */
void slam_bridge_run_optimization(void);

/**
 * @brief Stream out the complete SLAM pose graph and extracted 2D obstacle
 * point clouds as structured JSON over UART for real-time visualization.
 */
void slam_bridge_export_json(void);

/**
 * @brief Convert between jump-free odometry (drone_state_t.slam_x/y) and the
 * SLAM map frame using the latest committed correction. Thread-safe and
 * non-blocking; used by odom.c so navigation follows SLAM corrections.
 */
void slam_bridge_odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y);
void slam_bridge_map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y);

/**
 * @brief Get the total number of poses recorded in the current mission.
 */
size_t slam_bridge_get_pose_count(void);

/**
 * @brief Check if SLAM optimization is currently executing.
 */
bool slam_bridge_is_optimizing(void);

#ifdef __cplusplus
}
#endif
