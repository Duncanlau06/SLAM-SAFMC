# Combine 2: Autonomous Drone Swarm SLAM Firmware

## Overview

**`combine_2`** is the second-generation integrated firmware combining **`esp32 everything`** (ESP-IDF v5.5.5, FreeRTOS dual-core flight stack on ESP32-S3) with the **`eth zurich`** pure mathematical 2D graph SLAM and swarm engine, based on [ESP32_PORTING_AND_VERIFICATION_GUIDE.md](../ESP32_PORTING_AND_VERIFICATION_GUIDE.md).

## Architecture & The 2 Bridge Files

Rather than scattering SLAM algorithm logic inside `main.c`, `combine_2` introduces a modular 2-file bridge layer:

- **`main/slam_bridge.h`**: The clean public interface providing:
  - `slam_bridge_init()`: Initializes mutexes, swarm graph, and SLAM engine.
  - `slam_bridge_record_step()`: Samples current PX4 odometry, allocates a pose node, and captures multi-zone 360° ToF point clouds.
  - `slam_bridge_run_optimization()`: Executes ICP scan matching and Gauss-Newton least squares graph optimization ($H \Delta x = -b$) with RCM bandwidth reduction.
  - `slam_bridge_export_json()`: Streams the complete pose graph and 2D obstacle coordinates over UART.
- **`main/slam_bridge.c`**: The FreeRTOS-safe implementation decoupling PX4 flight controls from the SLAM solver.

```
                      ┌───────────────────────────────────────────────────────────┐
                      │                 combine_2 (ESP32-S3)                      │
                      └─────────────────────────────┬─────────────────────────────┘
                                                    │
         ┌──────────────────────────────────────────┴──────────────────────────────────────────┐
         │                                                                                     │
┌────────▼────────────────────────────────────────┐                   ┌────────────────────────▼────────────────────────┐
│     HARDWARE / FLIGHT STACK (esp-everything)    │                   │       MATHEMATICAL SLAM ENGINE (eth zurich)     │
├─────────────────────────────────────────────────┤                   ├─────────────────────────────────────────────────┤
│ • MAVLink Task (PX4 UART setpoints & state)     │   slam_bridge.h   │ • 2D ICP Point Matching (main/slam/icp/)        │
│ • Wi-Fi UDP Telemetry & Laptop Coordinator      │ <===============> │ • Graph Least-Squares SLAM (main/slam/ls-slam/) │
│ • 4x VL53L5CX I2C MUX Driver (TCA9548A)         │   slam_bridge.c   │ • 2D Metric Ray Extractor (main/slam/scan.c)    │
│ • PX4 Odometry & Transform Pipeline (odom.c)    │                   │ • Swarm Pose & Scan Graph (swarm_graph.c)       │
│ • Dual-core FreeRTOS Scheduler (main.c)         │                   │ • Multi-drone Coordination (swarm_comm.c)       │
└─────────────────────────────────────────────────┘                   └─────────────────────────────────────────────────┘
```

## Key Fixes & Quality Improvements

1. **Standard NED Directions**: Fixed left (`DIR_LEFT` = $-90^\circ$) and right (`DIR_RIGHT` = $+90^\circ$) ray projections in `scan.c`.
2. **Floor Reflection Elimination**: Switched row extraction to upper physical half (rows 4–7) to eliminate ground returns caused by upside-down ToF mounting.
3. **Chassis Cutoff**: Set minimum valid distance to $200\text{ mm}$ to discard reflections from the drone's own frame and propeller guards.
4. **ICP Outlier Gating**: Strict rotation ($\le 0.22\text{ rad}$) and translation ($\le 0.40\text{ m}$) thresholds reject bad local minima before adding graph constraints.

## Build & Flash (ESP-IDF)

From the ESP-IDF PowerShell or terminal:

```powershell
# 1. Activate ESP-IDF environment (if not already active)
& 'C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1'

# 2. Build firmware
cd combine_2
$env:IDF_TARGET="esp32s3"
idf.py build

# 3. Flash to ESP32-S3 and monitor
idf.py -p COM4 flash monitor
```

## Running Verification Suites

```bash
# Run math and coordinate checks
python tests/verify_slam_fixes.py

# Run ICP and graph optimization tests
python tests/test_slam_math.py
python tests/test_slam_pipeline.py
```

## Live 2D SLAM Map Visualizer

```bash
# Live stream from ESP32 over serial (plots and saves image upon landing)
python plot_slam_map.py --live

# Plot existing map from laptop/data/slam_map.json
python plot_slam_map.py
```
