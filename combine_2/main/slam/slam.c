/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#include "slam.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "icp/icp.h"
#include "ls-slam/graph-based-slam.h"
#include "scan.h"
#include "swarm.h"
#include "swarm_comm.h"
#include "swarm_graph.h"

static const char *TAG = "slam";

static volatile bool is_optimizing = false;

/* A point counts as an ICP inlier if it lands within this of a ref point. */
#define ICP_INLIER_DIST_M      0.15f
#define ICP_MIN_INLIER_RATIO   0.5f
/* Loop closures: only poses at least this many nodes apart, and only the
 * single closest candidate per pose, so one revisit adds one constraint. */
#define LOOP_MIN_NODE_GAP      5
#define LOOP_MAX_DIST_M        1.5f
/* Max pose correction ICP may apply. Logged PX4 positions were off by up to
 * 0.9 m (headings by <15°), so the old 0.30/0.40 m gates rejected exactly the
 * corrections SLAM exists to make. The inlier-ratio check is the real guard. */
#define ICP_MAX_ROT_RAD        0.35f
#define ICP_MAX_TRANS_M        1.0f
#define ICP_ITERATIONS         30
#define LOOP_MAX_DYAW_RAD      0.8f

static void log_slam_memory(const char *phase) {
  ESP_LOGI(
      TAG, "[%s] Free internal SRAM: %u bytes, Min ever: %u bytes", phase,
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
      (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL |
                                                MALLOC_CAP_8BIT));
}

static float wrap_pi(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

static bool comp_constr(icp_points_t *ref_scan, icp_points_t *cur_scan,
                        float *ref_pose, float *cur_pose,
                        int16_t ref_pose_idx, int16_t cur_pose_idx,
                        float max_rot, float max_trans,
                        constraint *out_cons) {

  if (ref_scan->num < 15 || cur_scan->num < 15) {
    return false;
  }

  // Execute ICP point matching
  icp_point_t icp_tr = {0};
  float icp_rot = 0;
  ESP_LOGD(TAG, "Starting ICP matching (%d pts vs %d pts)", (int)ref_scan->num,
           (int)cur_scan->num);
  icp(ref_scan, cur_scan, ICP_ITERATIONS, 0.6f, &icp_tr, &icp_rot);
  ESP_LOGD(TAG, "ICP result: T=(%.2f, %.2f) R=%.2f rad", icp_tr.x, icp_tr.y,
           icp_rot);

  float c = cosf(icp_rot);
  float s = sinf(icp_rot);
  float x = cur_pose[0] * c - cur_pose[1] * s + icp_tr.x;
  float y = cur_pose[0] * s + cur_pose[1] * c + icp_tr.y;
  float corrected_pose[3] = {x, y, cur_pose[2] + icp_rot};

  // Quality gating on how far ICP moves the *drone pose*. The raw ICP
  // translation is about the world origin, so it also contains R*pose and
  // grows with distance from takeoff even for a pure small rotation.
  float trans_dist = hypotf(x - cur_pose[0], y - cur_pose[1]);
  if (fabsf(icp_rot) > max_rot || trans_dist > max_trans) {
    ESP_LOGD(TAG, "ICP rejected: rot=%.2f (max %.2f), trans=%.2f (max %.2f)",
             icp_rot, max_rot, trans_dist, max_trans);
    return false;
  }

  // Reject alignments where too few points actually land on the reference
  // scan: ICP always converges to *something*, even on unrelated scans.
  size_t inliers = 0;
  for (size_t k = 0; k < cur_scan->num; k++) {
    icp_point_t q = {
        .x = cur_scan->items[k].x * c - cur_scan->items[k].y * s + icp_tr.x,
        .y = cur_scan->items[k].x * s + cur_scan->items[k].y * c + icp_tr.y,
    };
    for (size_t r = 0; r < ref_scan->num; r++) {
      if (points_dist_squared(q, ref_scan->items[r]) <
          ICP_INLIER_DIST_M * ICP_INLIER_DIST_M) {
        inliers++;
        break;
      }
    }
  }
  float inlier_ratio = (float)inliers / (float)cur_scan->num;
  if (inlier_ratio < ICP_MIN_INLIER_RATIO) {
    ESP_LOGD(TAG, "ICP rejected: inlier ratio %.2f (min %.2f)", inlier_ratio,
             ICP_MIN_INLIER_RATIO);
    return false;
  }

  out_cons->fromIdx = cur_pose_idx;
  out_cons->toIdx = ref_pose_idx;
  measurement_from_poses(corrected_pose, ref_pose, out_cons->z);

  ESP_LOGI(TAG, "Constraint %d -> %d ACCEPTED: z=(%.2f, %.2f, %.2f) (icp_rot=%.2f, trans=%.2f, inliers=%.2f)",
           cur_pose_idx, ref_pose_idx, out_cons->z[0], out_cons->z[1], out_cons->z[2],
           icp_rot, trans_dist, inlier_ratio);

  return true;
}

#define SLAM_MAX_CONSTRAINTS 64

void run_slam(void) {
  is_optimizing = true;
  uint64_t t_start_us = esp_timer_get_time();

  size_t total_poses = swarm_graph_get_pose_count();
  if (total_poses < 2) {
    ESP_LOGW(TAG, "Not enough poses in graph (%d) to run SLAM",
             (int)total_poses);
    is_optimizing = false;
    return;
  }

  if (total_poses > GRAPH_MAX_POSES) {
    total_poses = GRAPH_MAX_POSES;
  }

  float poses[GRAPH_MAX_POSES][3];
  uint8_t skip[GRAPH_MAX_POSES] = {0};
  constraint constraints[SLAM_MAX_CONSTRAINTS];
  size_t num_constraints = 0;

  // Load initial poses from swarm graph
  swarm_pose_t p;
  for (size_t i = 0; i < total_poses; i++) {
    p.id.drone_id = swarm_id;
    p.id.node_id = i;
    if (swarm_graph_load_pose(&p)) {
      poses[i][0] = p.x;
      poses[i][1] = p.y;
      poses[i][2] = p.yaw;
    } else {
      poses[i][0] = 0;
      poses[i][1] = 0;
      poses[i][2] = 0;
    }
  }

  // Allocate memory for scans and point clouds. Static (run_slam is never
  // re-entered) to keep ~7 KB off the stack now that all 8 sensors are used;
  // `static` binds to the macro's first statement, the backing buffer.
  static scan_t ref_scan;
  static scan_t cur_scan;
  static POINTS_STATIC_ALLOC(ref_cloud, POINTS_IN_SCAN)
  static POINTS_STATIC_ALLOC(cur_cloud, POINTS_IN_SCAN)

  // 1. Sequential ICP constraints between consecutive poses (refining odometry backbone)
  for (size_t i = 1; i < total_poses && num_constraints < SLAM_MAX_CONSTRAINTS; i++) {
    size_t j = i - 1;
    swarm_pose_id_t ref_id = {.drone_id = swarm_id, .node_id = (uint16_t)j};
    swarm_pose_id_t cur_id = {.drone_id = swarm_id, .node_id = (uint16_t)i};

    ref_scan.pose = ref_id;
    cur_scan.pose = cur_id;

    if (swarm_graph_has_scan(ref_id) && swarm_graph_has_scan(cur_id)) {
      swarm_graph_load_scan(&ref_scan);
      swarm_graph_load_scan(&cur_scan);

      scan_extract_points(&ref_scan, poses[j][0], poses[j][1], poses[j][2], ref_cloud);
      scan_extract_points(&cur_scan, poses[i][0], poses[i][1], poses[i][2], cur_cloud);

      constraint cons;
      if (comp_constr(ref_cloud, cur_cloud, poses[j], poses[i],
                      (int16_t)j, (int16_t)i, ICP_MAX_ROT_RAD, ICP_MAX_TRANS_M, &cons)) {
        constraints[num_constraints++] = cons;
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }

  // 2. Loop closure constraints for non-adjacent poses (revisiting earlier
  //    locations). One constraint per pose, against its closest earlier pose.
  //    Accepting every nearby pair (previous behaviour) let dozens of
  //    20x-weighted constraints from a small hover area overpower odometry.
  for (size_t i = LOOP_MIN_NODE_GAP; i < total_poses && num_constraints < SLAM_MAX_CONSTRAINTS; i++) {
    int best_j = -1;
    float best_dist = LOOP_MAX_DIST_M;
    for (size_t j = 0; j + LOOP_MIN_NODE_GAP <= i; j++) {
      float dist = hypotf(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1]);
      float dyaw = fabsf(wrap_pi(poses[i][2] - poses[j][2]));
      if (dist < best_dist && dyaw < LOOP_MAX_DYAW_RAD) {
        best_dist = dist;
        best_j = (int)j;
      }
    }
    if (best_j < 0) continue;

    size_t j = (size_t)best_j;
    swarm_pose_id_t ref_id = {.drone_id = swarm_id, .node_id = (uint16_t)j};
    swarm_pose_id_t cur_id = {.drone_id = swarm_id, .node_id = (uint16_t)i};

    ref_scan.pose = ref_id;
    cur_scan.pose = cur_id;

    if (swarm_graph_has_scan(ref_id) && swarm_graph_has_scan(cur_id)) {
      swarm_graph_load_scan(&ref_scan);
      swarm_graph_load_scan(&cur_scan);

      scan_extract_points(&ref_scan, poses[j][0], poses[j][1], poses[j][2], ref_cloud);
      scan_extract_points(&cur_scan, poses[i][0], poses[i][1], poses[i][2], cur_cloud);

      constraint cons;
      if (comp_constr(ref_cloud, cur_cloud, poses[j], poses[i],
                      (int16_t)j, (int16_t)i, ICP_MAX_ROT_RAD, ICP_MAX_TRANS_M, &cons)) {
        constraints[num_constraints++] = cons;
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(1));

  ESP_LOGI(TAG, "Running Graph LS-SLAM on %d poses with %d constraints...",
           (int)total_poses, (int)num_constraints);
  log_slam_memory("pre_slam");

  ls_slam((float *)poses, skip, constraints, (int16_t)total_poses,
          (int16_t)num_constraints);
  log_slam_memory("post_slam");

  uint64_t t_elapsed_ms = (esp_timer_get_time() - t_start_us) / 1000;
  ESP_LOGI(TAG, "LS-SLAM Optimization completed in %u ms",
           (unsigned int)t_elapsed_ms);

  // Save optimized poses back into graph and broadcast
  for (size_t i = 0; i < total_poses; i++) {
    p.id.drone_id = swarm_id;
    p.id.node_id = (uint16_t)i;
    if (swarm_graph_load_pose(&p)) {
      p.x = poses[i][0];
      p.y = poses[i][1];
      p.yaw = poses[i][2];
      swarm_graph_store_pose(&p);
      swarm_comm_broadcast_pose(p.id);
    }
  }

  is_optimizing = false;
}

static void slam_background_task(void *pvParameters) {
  run_slam();
  vTaskDelete(NULL);
}

void slam_trigger_optimization(void) {
  if (is_optimizing) {
    ESP_LOGW(TAG, "SLAM optimization already running");
    return;
  }
  // Launch on Core 0 with 40KB stack — SLAM puts ~30KB on the stack
  // Pinned to Core 0 to avoid starving nav_task/mission_task on Core 1
  xTaskCreatePinnedToCore(slam_background_task, "slam_opt", 40960, NULL, 1,
                          NULL, 0);
}

bool slam_is_optimizing(void) { return is_optimizing; }

void slam_init(void) {
  swarm_init();
  swarm_graph_init();
  swarm_comm_init();
  ESP_LOGI(TAG, "SLAM subsystem initialized");
}
