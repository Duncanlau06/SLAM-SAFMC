/**
 * @file occ_grid.c
 * @brief Onboard occupancy-grid map (see occ_grid.h).
 *
 * Raycasting follows the approach of mark_laser_bresenham() / get_occ_map2()
 * in laptop/mapping.c (Polonelli et al., IWASI 2023), but counts distinct
 * scans per hit cell so one noisy scan cannot create a wall on its own.
 */

#include "occ_grid.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "icp/icp.h"
#include "scan.h"
#include "swarm.h"
#include "swarm_graph.h"

static const char *TAG = "occ_grid";

enum { CELL_UNKNOWN = 0, CELL_FREE = 1, CELL_OCC = 2 };

typedef struct {
  float res, x0, y0; /* x0 = min north, y0 = min east (cell corner) */
  int rows, cols;    /* row = north index, col = east index */
  uint8_t *hits;     /* distinct scans ending in the cell (saturating) */
  uint8_t *misses;   /* beams passing through the cell (saturating) */
  uint8_t *last;     /* last scan (index + 1) that hit the cell; reused for classes */
} grid_t;

/* Scratch cloud for one scan; static to keep it off the mission-task stack.
 * (POINTS_STATIC_ALLOC contains a statement, so it can't be used at file scope.) */
static uint8_t s_cloud_buf[POINTS_SIZEOF(POINTS_IN_SCAN)] __attribute__((aligned(4)));
static icp_points_t *const s_cloud = (icp_points_t *)s_cloud_buf;

static bool load_scan_points(size_t node, float pose[3]) {
  swarm_pose_t p = {.id = {.drone_id = swarm_id, .node_id = (uint16_t)node}};
  if (!swarm_graph_load_pose(&p)) return false;
  static scan_t sc;
  sc.pose = p.id;
  if (!swarm_graph_has_scan(p.id) || !swarm_graph_load_scan(&sc)) return false;
  pose[0] = p.x;
  pose[1] = p.y;
  pose[2] = p.yaw;
  scan_extract_points(&sc, p.x, p.y, p.yaw, s_cloud);
  return true;
}

static inline void sat_inc(uint8_t *v) {
  if (*v < UINT8_MAX) (*v)++;
}

static inline int cell_of(const grid_t *g, float n, float e, int *r, int *c) {
  *r = (int)floorf((n - g->x0) / g->res);
  *c = (int)floorf((e - g->y0) / g->res);
  return *r >= 0 && *r < g->rows && *c >= 0 && *c < g->cols;
}

/* Marks every cell from (r0,c0) up to, but not including, (r1,c1) as a miss */
static void raycast_free(grid_t *g, int r0, int c0, int r1, int c1) {
  int dr = abs(r1 - r0), dc = abs(c1 - c0);
  int sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
  int err = dr - dc;
  while (r0 != r1 || c0 != c1) {
    if (r0 >= 0 && r0 < g->rows && c0 >= 0 && c0 < g->cols) {
      sat_inc(&g->misses[r0 * g->cols + c0]);
    }
    int e2 = 2 * err;
    if (e2 > -dc) { err -= dc; r0 += sr; }
    if (e2 < dr)  { err += dr; c0 += sc; }
  }
}

static bool compute_bounds(grid_t *g, size_t num_poses) {
  float min_n = INFINITY, min_e = INFINITY, max_n = -INFINITY, max_e = -INFINITY;
  float pose[3];
  for (size_t i = 0; i < num_poses; i++) {
    if (!load_scan_points(i, pose)) continue;
    min_n = fminf(min_n, pose[0]); max_n = fmaxf(max_n, pose[0]);
    min_e = fminf(min_e, pose[1]); max_e = fmaxf(max_e, pose[1]);
    for (size_t k = 0; k < s_cloud->num; k++) {
      min_n = fminf(min_n, s_cloud->items[k].x); max_n = fmaxf(max_n, s_cloud->items[k].x);
      min_e = fminf(min_e, s_cloud->items[k].y); max_e = fmaxf(max_e, s_cloud->items[k].y);
    }
  }
  if (!isfinite(min_n)) return false;

  g->x0 = min_n - OCC_MARGIN_M;
  g->y0 = min_e - OCC_MARGIN_M;
  float span_n = (max_n + OCC_MARGIN_M) - g->x0;
  float span_e = (max_e + OCC_MARGIN_M) - g->y0;
  g->res = OCC_RES_M;
  while (span_n / g->res > OCC_MAX_DIM || span_e / g->res > OCC_MAX_DIM) {
    g->res *= 1.25f;
  }
  g->rows = (int)ceilf(span_n / g->res);
  g->cols = (int)ceilf(span_e / g->res);
  return true;
}

static void raycast_all(grid_t *g, size_t num_poses) {
  float pose[3];
  for (size_t i = 0; i < num_poses; i++) {
    if (!load_scan_points(i, pose)) continue;
    int pr, pc;
    cell_of(g, pose[0], pose[1], &pr, &pc);
    uint8_t tag = (uint8_t)(i + 1);
    for (size_t k = 0; k < s_cloud->num; k++) {
      int hr, hc;
      int inside = cell_of(g, s_cloud->items[k].x, s_cloud->items[k].y, &hr, &hc);
      raycast_free(g, pr, pc, hr, hc);
      if (!inside) continue;
      size_t idx = (size_t)hr * g->cols + hc;
      if (g->last[idx] != tag) { /* count each scan once per cell */
        g->last[idx] = tag;
        sat_inc(&g->hits[idx]);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* Writes the cell classes into g->last (no longer needed after raycasting) and
 * returns the per-class counts. */
static void classify(grid_t *g, int counts[3]) {
  uint8_t *cls = g->last;
  size_t n = (size_t)g->rows * g->cols;
  for (size_t i = 0; i < n; i++) {
    unsigned h = g->hits[i], m = g->misses[i];
    if (h >= OCC_MIN_HITS && h * 100 >= OCC_MIN_RATIO_PCT * (h + m)) cls[i] = CELL_OCC;
    else if (m > 0 || h > 0) cls[i] = CELL_FREE;
    else cls[i] = CELL_UNKNOWN;
  }

  /* Drop isolated occupied cells. In place is safe: an isolated cell has no
   * occupied neighbours, so removing it cannot isolate anything else. */
  for (int r = 0; r < g->rows; r++) {
    for (int c = 0; c < g->cols; c++) {
      if (cls[r * g->cols + c] != CELL_OCC) continue;
      bool has_neighbour = false;
      for (int dr = -1; dr <= 1 && !has_neighbour; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
          int rr = r + dr, cc = c + dc;
          if ((dr || dc) && rr >= 0 && rr < g->rows && cc >= 0 && cc < g->cols &&
              cls[rr * g->cols + cc] == CELL_OCC) {
            has_neighbour = true;
            break;
          }
        }
      }
      if (!has_neighbour) cls[r * g->cols + c] = CELL_FREE;
    }
  }

  counts[0] = counts[1] = counts[2] = 0;
  for (size_t i = 0; i < n; i++) counts[cls[i]]++;
}

static void export_rle(const grid_t *g) {
  static const char sym[3] = {'u', 'f', 'o'};
  static char line[OCC_MAX_DIM * 5 + 2]; /* worst case "400u" x alternating runs */
  const uint8_t *cls = g->last;

  printf("\n=== SLAM_GRID_START ===\n");
  printf("{\"res\":%.4f,\"x0\":%.3f,\"y0\":%.3f,\"rows\":%d,\"cols\":%d}\n",
         g->res, g->x0, g->y0, g->rows, g->cols);
  for (int r = 0; r < g->rows; r++) {
    int len = 0;
    const uint8_t *row = &cls[r * g->cols];
    for (int c = 0; c < g->cols;) {
      int run = 1;
      while (c + run < g->cols && row[c + run] == row[c]) run++;
      len += snprintf(line + len, sizeof(line) - len, "%d%c", run, sym[row[c]]);
      c += run;
    }
    printf("%s\n", line);
    if ((r & 15) == 15) {
      fflush(stdout);
      vTaskDelay(pdMS_TO_TICKS(2)); /* let the console drain; avoid task watchdog */
    }
  }
  printf("=== SLAM_GRID_END ===\n");
  fflush(stdout);
}

void occ_grid_build_and_export(void) {
  uint64_t t0 = esp_timer_get_time();
  size_t num_poses = swarm_graph_get_pose_count();

  grid_t g = {0};
  if (!compute_bounds(&g, num_poses)) {
    ESP_LOGW(TAG, "No scans in graph, skipping occupancy grid");
    return;
  }

  size_t n = (size_t)g.rows * g.cols;
  g.hits = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
  g.misses = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
  g.last = heap_caps_calloc(n, 1, MALLOC_CAP_SPIRAM);
  if (!g.hits || !g.misses || !g.last) {
    ESP_LOGE(TAG, "Failed to allocate %u-cell grid in PSRAM", (unsigned)n);
    goto out;
  }

  raycast_all(&g, num_poses);
  int counts[3];
  classify(&g, counts);

  ESP_LOGI(TAG, "occ_grid built in %u ms: %dx%d @ %.3f m (occ %d, free %d, unknown %d), free PSRAM %u",
           (unsigned)((esp_timer_get_time() - t0) / 1000), g.rows, g.cols, g.res,
           counts[CELL_OCC], counts[CELL_FREE], counts[CELL_UNKNOWN],
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  export_rle(&g);

out:
  heap_caps_free(g.hits);
  heap_caps_free(g.misses);
  heap_caps_free(g.last);
}
