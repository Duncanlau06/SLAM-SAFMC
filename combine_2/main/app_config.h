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
