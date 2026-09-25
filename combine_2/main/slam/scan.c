/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "scan.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>

#include "icp/icp.h"
#include "mavlink_task.h"
#include "swarm_graph.h"
#include "tof_task.h"
#include "util.h"

static const char *TAG = "scan";

void scan_extract_points(const scan_t *scan, float x, float y, float yaw,
                         icp_points_t *points) {

  /* Sensor lens distance from the drone centre (m) */
  const float sensor_offset = 0.025f;
  const float step = -((float)M_PI_4) / 8.0f;

  points->num = 0;

  for (int frame = 0; frame < SCAN_NUM_FRAMES; frame++) {

    float frame_x = x + scan->frames[frame].dx;
    float frame_y = y + scan->frames[frame].dy;
    float frame_yaw = yaw + scan->frames[frame].dyaw;

    for (int dir = 0; dir < SCAN_NUM_DIRS; dir++) {

      /* NED yaw is CW-positive, so a sensor k slots to the left is at -k*spacing */
      float dir_yaw = frame_yaw - (float)dir * (2.0f * (float)M_PI / SCAN_NUM_DIRS);

      float angle = -4.0f * step + step / 2.0f;
      for (int col = 0; col < 8; angle += step, col++) {

        int16_t measurement = scan->frames[frame].dists[dir][col];
        if (measurement < SCAN_MIN_VALID_MM || measurement > 3500)
          continue;

        // Apply true radial ToF beam projection
        float range = (float)measurement / 1000.0f;
        float dist_x = range * cosf(angle) + sensor_offset;
        float dist_y = range * sinf(angle);

        icp_point_t p = {
            .x = frame_x + dist_x * cosf(dir_yaw) - dist_y * sinf(dir_yaw),
            .y = frame_y + dist_x * sinf(dir_yaw) + dist_y * cosf(dir_yaw),
        };
        points->items[points->num++] = p;
      }
    }
  }
}

static void extract_measurements_from_frame(const tof_frame_t *frame,
                                            int16_t measurements[8]) {
  for (int col = 0; col < 8; col++) {
    int16_t rows[4];
    uint8_t num_rows = 0;
    /* Rows 2-5 = the four centre rows (~±11° about horizontal), as in the ETH
     * original. Symmetric, so it is independent of sensor mounting
     * orientation. The outer rows (0-1, 6-7) are ~14-20° off horizontal and at
     * 0.5 m cruise altitude hit the floor/ceiling at ~1.4-2 m, which produced
     * rings of fake obstacles around every pose. */
    for (int row = 2; row < 6; row++) {
      int idx = row * 8 + col;
      uint8_t status = frame->target_status[idx];
      uint16_t dist = frame->distance_mm[idx];
      /* Only accept valid status 5 (valid) and 9 (valid weak).
       * Reject status 6 (wrap-around error) and status 10 (low confidence).
       * Minimum distance SCAN_MIN_VALID_MM excludes chassis/prop-guard hits. */
      if ((status == 5 || status == 9) && dist >= SCAN_MIN_VALID_MM &&
          dist <= TOF_MAX_VALID_MM) {
        rows[num_rows++] = (int16_t)dist;
      }
    }

    if (num_rows == 0) {
      measurements[col] = -1;
      continue;
    }

    qsort(rows, num_rows, sizeof(int16_t), compare_int16);
    if (num_rows & 1) {
      measurements[col] = rows[num_rows >> 1];
    } else {
      measurements[col] =
          (rows[num_rows >> 1] + rows[(num_rows >> 1) - 1]) >> 1;
    }
  }
}

void scan_acquire(scan_t *scan) {
  drone_state_t st0 = mavlink_get_state();
  float x0 = st0.x;
  float y0 = st0.y;
  float yaw0 = st0.heading;

  TickType_t prev_wake_time = xTaskGetTickCount();
  for (int i = 0; i < SCAN_NUM_FRAMES; i++) {
    drone_state_t st = mavlink_get_state();
    scan->frames[i].dx = st.x - x0;
    scan->frames[i].dy = st.y - y0;
    scan->frames[i].dyaw = st.heading - yaw0;

    tof_scan_t raw_scan = tof_get_scan();
    for (int dir = 0; dir < SCAN_NUM_DIRS; dir++) {
      int sensor_idx = (TOF_FRONT_SENSOR_IDX + dir) % TOF_SENSOR_COUNT;
      if (raw_scan.sensor_ok[sensor_idx] && raw_scan.frame[sensor_idx].valid) {
        extract_measurements_from_frame(&raw_scan.frame[sensor_idx],
                                        scan->frames[i].dists[dir]);
      } else {
        for (int j = 0; j < 8; j++) {
          scan->frames[i].dists[dir][j] = -1;
        }
      }
    }

    vTaskDelayUntil(&prev_wake_time,
                    pdMS_TO_TICKS(SCAN_TIME_MS / SCAN_NUM_FRAMES));
  }

  swarm_graph_store_scan(scan);
  ESP_LOGI(TAG, "Scan acquired and stored for pose [%d:%d]",
           scan->pose.drone_id, scan->pose.node_id);
}
