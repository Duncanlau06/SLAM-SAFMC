/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "scan.h"

#include "esp_log.h"
#include "esp_timer.h"
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
        if (measurement < SCAN_MIN_VALID_MM || measurement > SCAN_MAX_VALID_MM)
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

/* Returns the number of columns rejected because their rows disagreed or too
 * few were valid (for the per-scan diagnostics). */
static int extract_measurements_from_frame(const tof_frame_t *frame,
                                           int16_t measurements[8]) {
  int rejected = 0;
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
          dist <= SCAN_MAX_VALID_MM) {
        rows[num_rows++] = (int16_t)dist;
      }
    }

    if (num_rows == 0) {
      measurements[col] = -1;
      continue;
    }

    qsort(rows, num_rows, sizeof(int16_t), compare_int16);
    if (num_rows < SCAN_MIN_ROWS ||
        rows[num_rows - 1] - rows[0] > SCAN_MAX_ROW_SPREAD_MM) {
      measurements[col] = -1;
      rejected++;
      continue;
    }
    if (num_rows & 1) {
      measurements[col] = rows[num_rows >> 1];
    } else {
      measurements[col] =
          (rows[num_rows >> 1] + rows[(num_rows >> 1) - 1]) >> 1;
    }
  }
  return rejected;
}

static void invalidate_dists(int16_t dists[8]) {
  for (int j = 0; j < 8; j++) {
    dists[j] = -1;
  }
}

void scan_acquire(scan_t *scan) {
  drone_state_t st0 = mavlink_get_state();
  float x0 = st0.slam_x;
  float y0 = st0.slam_y;
  float yaw0 = st0.heading;

  /* Per-scan drop counts, in columns (8 per sensor frame) */
  int n_stale = 0, n_spin = 0, n_rows = 0;

  TickType_t prev_wake_time = xTaskGetTickCount();
  for (int i = 0; i < SCAN_NUM_FRAMES; i++) {
    drone_state_t st = mavlink_get_state();
    scan->frames[i].dx = st.slam_x - x0;
    scan->frames[i].dy = st.slam_y - y0;
    scan->frames[i].dyaw = st.heading - yaw0;

    tof_scan_t raw_scan = tof_get_scan();
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    for (int dir = 0; dir < SCAN_NUM_DIRS; dir++) {
      int sensor_idx = (TOF_FRONT_SENSOR_IDX + dir) % TOF_SENSOR_COUNT;
      const tof_frame_t *frame = &raw_scan.frame[sensor_idx];
      if (!raw_scan.sensor_ok[sensor_idx] || !frame->valid) {
        invalidate_dists(scan->frames[i].dists[dir]);
        continue;
      }

      /* The frame was measured before this pose sample. Points are placed
       * using the current pose, so an old frame lands in the wrong spot. */
      uint32_t age_ms = now_ms - frame->timestamp_ms;
      if (age_ms > SCAN_MAX_FRAME_AGE_MS) {
        invalidate_dists(scan->frames[i].dists[dir]);
        n_stale += 8;
        continue;
      }
      if (fabsf(st.yaw_rate) * (float)age_ms / 1000.0f > SCAN_MAX_YAW_ERR_RAD) {
        invalidate_dists(scan->frames[i].dists[dir]);
        n_spin += 8;
        continue;
      }

      n_rows += extract_measurements_from_frame(frame, scan->frames[i].dists[dir]);
    }

    vTaskDelayUntil(&prev_wake_time,
                    pdMS_TO_TICKS(SCAN_TIME_MS / SCAN_NUM_FRAMES));
  }

  int n_kept = 0;
  for (int i = 0; i < SCAN_NUM_FRAMES; i++)
    for (int dir = 0; dir < SCAN_NUM_DIRS; dir++)
      for (int col = 0; col < 8; col++)
        if (scan->frames[i].dists[dir][col] >= 0) n_kept++;

  swarm_graph_store_scan(scan);
  ESP_LOGI(TAG, "Scan acquired and stored for pose [%d:%d] (kept %d / stale %d / spin %d / rows %d)",
           scan->pose.drone_id, scan->pose.node_id, n_kept, n_stale, n_spin, n_rows);
}
