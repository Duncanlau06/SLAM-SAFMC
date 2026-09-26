/**
 * @file slam_bridge.c
 * @brief Implementation of the bridge between ESP32 tasks and ETH Zurich SLAM.
 *
 * Frames
 *   odom : PX4 local NED with estimator resets removed (drone_state_t.slam_x/y).
 *   map  : SLAM frame. Starts equal to odom at takeoff; every committed
 *          optimisation updates the correction D so that map = D ⊕ odom.
 *
 * New poses are stored in the map frame (D ⊕ raw odom), so the graph stays
 * continuous across corrections. Navigation reads D through
 * slam_bridge_odom_to_map / slam_bridge_map_to_odom (used by odom.c).
 */

#include "slam_bridge.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "app_config.h"
#include "mavlink_task.h"
#include "occ_grid.h"
#include "slam.h"
#include "swarm.h"
#include "swarm_graph.h"
#include "scan.h"

static const char *TAG = "slam_bridge";

/* Held for a whole pose record (incl. the 400 ms scan) and for commits, so a
 * commit never interleaves with a half-recorded pose. */
static SemaphoreHandle_t s_bridge_mutex = NULL;
static StaticSemaphore_t s_bridge_mutex_buffer;
static uint16_t s_node_idx = 0;
/* Odometry (jump-free) poses as recorded, exported so the plotter can show
 * what SLAM changed, and used to re-anchor poses on every commit. */
static float s_raw_poses[GRAPH_MAX_POSES][3];

/* Optimiser output; only one optimisation runs at a time (s_opt_running) */
static float s_opt_poses[GRAPH_MAX_POSES][3];
static volatile bool s_opt_running = false;

/* ---------------------------------------------------------------------------
 * SE(2) helpers — x = north, y = east, yaw clockwise (NED), same rotation
 * convention as scan_extract_points().
 * --------------------------------------------------------------------------- */
typedef struct { float x, y, yaw; } se2_t;

static se2_t se2_compose(se2_t a, se2_t b) {
    float c = cosf(a.yaw), s = sinf(a.yaw);
    return (se2_t){a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
}

static se2_t se2_inverse(se2_t a) {
    float c = cosf(a.yaw), s = sinf(a.yaw);
    return (se2_t){-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw};
}

/* map = s_corr ⊕ odom. Read at 20 Hz by navigation, so guarded by a short
 * critical section rather than s_bridge_mutex (held for 400 ms scans). */
static se2_t s_corr = {0.0f, 0.0f, 0.0f};
static portMUX_TYPE s_corr_lock = portMUX_INITIALIZER_UNLOCKED;

static se2_t get_corr(void) {
    taskENTER_CRITICAL(&s_corr_lock);
    se2_t c = s_corr;
    taskEXIT_CRITICAL(&s_corr_lock);
    return c;
}

void slam_bridge_odom_to_map(float odom_x, float odom_y, float *map_x, float *map_y) {
    se2_t m = se2_compose(get_corr(), (se2_t){odom_x, odom_y, 0.0f});
    *map_x = m.x;
    *map_y = m.y;
}

void slam_bridge_map_to_odom(float map_x, float map_y, float *odom_x, float *odom_y) {
    se2_t o = se2_compose(se2_inverse(get_corr()), (se2_t){map_x, map_y, 0.0f});
    *odom_x = o.x;
    *odom_y = o.y;
}

/* ---------------------------------------------------------------------------
 * Commit
 * --------------------------------------------------------------------------- */

/* Applies s_opt_poses[0..n-1] to the graph and moves the correction so that the
 * newest optimised pose becomes the anchor. Poses recorded while the optimiser
 * ran (n..s_node_idx-1) are re-anchored from their raw odometry. Rejects the
 * result if the newest pose would move more than max_shift_m.
 * Caller holds s_bridge_mutex. */
static bool commit_locked(size_t n, float max_shift_m) {
    swarm_pose_t last = {.id = {.drone_id = swarm_id, .node_id = (uint16_t)(n - 1)}};
    if (!swarm_graph_load_pose(&last)) return false;

    float shift = hypotf(s_opt_poses[n - 1][0] - last.x, s_opt_poses[n - 1][1] - last.y);
    if (shift > max_shift_m) {
        ESP_LOGW(TAG, "SLAM result rejected: would move current estimate %.2f m (max %.2f)",
                 shift, max_shift_m);
        return false;
    }

    slam_commit_poses(s_opt_poses, n);

    se2_t anchor_map = {s_opt_poses[n - 1][0], s_opt_poses[n - 1][1], s_opt_poses[n - 1][2]};
    se2_t anchor_odom = {s_raw_poses[n - 1][0], s_raw_poses[n - 1][1], s_raw_poses[n - 1][2]};
    se2_t corr = se2_compose(anchor_map, se2_inverse(anchor_odom));
    taskENTER_CRITICAL(&s_corr_lock);
    s_corr = corr;
    taskEXIT_CRITICAL(&s_corr_lock);

    for (uint16_t i = (uint16_t)n; i < s_node_idx; i++) {
        swarm_pose_t p = {.id = {.drone_id = swarm_id, .node_id = i}};
        if (!swarm_graph_load_pose(&p)) continue;
        se2_t m = se2_compose(corr, (se2_t){s_raw_poses[i][0], s_raw_poses[i][1], s_raw_poses[i][2]});
        p.x = m.x;
        p.y = m.y;
        p.yaw = m.yaw;
        swarm_graph_store_pose(&p);
    }

    ESP_LOGI(TAG, "SLAM committed %u poses (+%u re-anchored): current estimate moved %.2f m, "
                  "correction now N%+.2f E%+.2f yaw%+.3f",
             (unsigned)n, (unsigned)(s_node_idx - n), shift, corr.x, corr.y, corr.yaw);
    return true;
}

/* ---------------------------------------------------------------------------
 * In-flight background optimisation
 * --------------------------------------------------------------------------- */

static void slam_opt_task(void *arg) {
    uint64_t t0 = esp_timer_get_time();
    size_t n = slam_optimize(s_opt_poses);
    if (n >= 2) {
        xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);
        commit_locked(n, SLAM_NAV_MAX_SHIFT_M);
        xSemaphoreGive(s_bridge_mutex);
    }
    ESP_LOGI(TAG, "In-flight SLAM finished in %llu ms",
             (unsigned long long)((esp_timer_get_time() - t0) / 1000));
    s_opt_running = false;
    vTaskDelete(NULL);
}

static void maybe_trigger_inflight(void) {
#if SLAM_INFLIGHT_EVERY_N > 0
    if (s_node_idx % SLAM_INFLIGHT_EVERY_N != 0 || s_opt_running) return;
    s_opt_running = true;
    /* Core 0, priority 1: below mavlink (5) and ToF (4), so sensing and the
     * flight link always win. Core 1 (nav, mission) is untouched. */
    if (xTaskCreatePinnedToCore(slam_opt_task, "slam_opt", 40960, NULL, 1, NULL, 0) != pdPASS) {
        s_opt_running = false;
        ESP_LOGW(TAG, "Could not start in-flight SLAM task (free internal RAM %u)",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
#endif
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

void slam_bridge_init(void) {
    if (s_bridge_mutex == NULL) {
        s_bridge_mutex = xSemaphoreCreateMutexStatic(&s_bridge_mutex_buffer);
        s_node_idx = 0;
        swarm_init();
        swarm_graph_init();
        slam_init();
        ESP_LOGI(TAG, "SLAM Bridge initialized for Swarm ID %d (Max %d poses in RAM)",
                 swarm_id, GRAPH_MAX_POSES);
    }
}

bool slam_bridge_record_pose(float x_m, float y_m, float yaw_rad) {
    if (s_bridge_mutex == NULL) {
        slam_bridge_init();
    }

    xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);

    if (s_node_idx >= GRAPH_MAX_POSES) {
        ESP_LOGW(TAG, "SLAM pose graph full (%d poses)", GRAPH_MAX_POSES);
        xSemaphoreGive(s_bridge_mutex);
        return false;
    }

    /* Store in the map frame so the graph stays continuous with poses that
     * earlier optimisations already corrected. */
    se2_t m = se2_compose(get_corr(), (se2_t){x_m, y_m, yaw_rad});
    swarm_pose_t p = {
        .id = {
            .drone_id = swarm_id,
            .node_id = s_node_idx++,
        },
        .x = m.x,
        .y = m.y,
        .yaw = m.yaw,
        .num_edges = 0,
    };

    swarm_graph_store_pose(&p);
    s_raw_poses[p.id.node_id][0] = x_m;
    s_raw_poses[p.id.node_id][1] = y_m;
    s_raw_poses[p.id.node_id][2] = yaw_rad;

    scan_t sc;
    sc.pose = p.id;
    scan_acquire(&sc);

    ESP_LOGI(TAG, "Stored SLAM pose [%d:%d] -> x=%.2f, y=%.2f, yaw=%.2f rad",
             p.id.drone_id, p.id.node_id, p.x, p.y, p.yaw);

    maybe_trigger_inflight();

    xSemaphoreGive(s_bridge_mutex);
    return true;
}

bool slam_bridge_record_step(void) {
    drone_state_t st = mavlink_get_state();
    /* Jump-free odometry: PX4 estimator resets once shifted whole segments of
     * the map by ~10 m. See POS_JUMP_MIN_M in mavlink_task.c. */
    return slam_bridge_record_pose(st.slam_x, st.slam_y, st.heading);
}

void slam_bridge_run_optimization(void) {
    /* Let an in-flight optimisation finish first; they share s_opt_poses */
    while (s_opt_running) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_opt_running = true;

    ESP_LOGI(TAG, "Starting post-flight Graph LS-SLAM optimization...");
    uint64_t t_start_us = esp_timer_get_time();

    size_t n = slam_optimize(s_opt_poses);
    if (n >= 2) {
        xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);
        commit_locked(n, INFINITY); /* on the ground: no navigation to protect */
        xSemaphoreGive(s_bridge_mutex);
    }

    uint64_t elapsed_ms = (esp_timer_get_time() - t_start_us) / 1000;
    ESP_LOGI(TAG, "Graph LS-SLAM finished in %llu ms", (unsigned long long)elapsed_ms);
    s_opt_running = false;
}

void slam_bridge_export_json(void) {
    ESP_LOGI(TAG, "Exporting SLAM Map JSON to console...");

    /* Printed before the map JSON: the live plotter plots on SLAM_MAP_JSON_END */
    printf("\n=== SLAM_RAW_POSES_START ===\n[");
    for (uint16_t i = 0; i < s_node_idx; i++) {
        printf("%s[%.3f,%.3f,%.3f]", i ? "," : "",
               s_raw_poses[i][0], s_raw_poses[i][1], s_raw_poses[i][2]);
    }
    printf("]\n=== SLAM_RAW_POSES_END ===\n");
    fflush(stdout);

    swarm_graph_debug_print();

    /* Last block sent: the laptop plots when SLAM_GRID_END arrives */
    occ_grid_build_and_export();
}

size_t slam_bridge_get_pose_count(void) {
    return swarm_graph_get_pose_count();
}

bool slam_bridge_is_optimizing(void) {
    return s_opt_running;
}
