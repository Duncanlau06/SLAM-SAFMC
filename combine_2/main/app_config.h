#pragma once

/* ---------------------------------------------------------------------------
 * Application Feature Flags & SLAM Configuration
 * --------------------------------------------------------------------------- */
#define ENABLE_WIFI                  0   /* 0 = disabled (pure standalone flight & SLAM) */
#define ENABLE_APRILTAG              0   /* 0 = disabled (camera & visual detection off) */

/* Autonomous flight duration in seconds before landing & SLAM map generation */
#define AUTONOMOUS_FLIGHT_DURATION_S 30

/* SLAM Pose & Scan Acquisition Interval (milliseconds) */
#define SLAM_RECORD_INTERVAL_MS      1500

/* In-flight SLAM: optimise in the background every N recorded poses
 * (N * SLAM_RECORD_INTERVAL_MS = every 7.5 s). 0 = only after landing. */
#define SLAM_INFLIGHT_EVERY_N        5

/* 1 = navigation uses the SLAM-corrected position (via odom map_to_odom /
 * odom_to_map), which also cancels PX4 position resets. 0 = raw PX4 odometry.
 * Do not combine with AprilTag relocalisation: both would correct the same
 * offset. */
#define SLAM_CORRECT_NAV             1

/* Reject an in-flight optimisation that would move the drone's current
 * position estimate by more than this. Protects navigation from a bad loop
 * closure; the post-flight optimisation is not limited. */
#define SLAM_NAV_MAX_SHIFT_M         0.5f

#if SLAM_CORRECT_NAV && ENABLE_APRILTAG
#error "SLAM_CORRECT_NAV and ENABLE_APRILTAG both correct map_T_odom; enable only one"
#endif
