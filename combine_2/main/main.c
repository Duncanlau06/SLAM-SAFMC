#include "app_config.h"
#include "slam_bridge.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mavlink_task.h"
#include "nav_task.h"
#include "nvs_flash.h"
#include "odom.h"
#include "tof_task.h"

#include <math.h>

static const char *TAG = "mission";

/* ---------------------------------------------------------------------------
 * Test parameters — adjust before flight
 * ---------------------------------------------------------------------------
 */
#define CRUISE_ALT_M 0.5f     /* target altitude above takeoff (m)        */
#define GOAL_OFFSET_X_M 2.0f  /* goal North offset from takeoff (m)       */
#define GOAL_OFFSET_Y_M 1.6f  /* goal East  offset from takeoff (m)       */
#define TAKEOFF_TIMEOUT_S 10  /* max seconds to wait for altitude (s)     */
#define LAND_TIMEOUT_S 20     /* max seconds to wait for disarm (s)       */
#define ALT_TOLERANCE_M 0.15f /* altitude band considered "at altitude"   */

void log_memory_stats(const char *tag) {
  size_t free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  ESP_LOGI("MEM", "[%s] Free SRAM: %u bytes (Min Lifetime Free: %u bytes)",
           tag, (unsigned int)free_sram, (unsigned int)min_free_sram);
}

/* ---------------------------------------------------------------------------
 * Mission task — runs once, handles full autonomous flight sequence & SLAM
 *
 * Sequencing contract with nav_task:
 *   1. nav_task is idle (no goal) — mission_task owns the MAVLink setpoint
 *   2. nav_set_goal_map() is called — nav_task takes over setpoints
 *   3. nav_cancel() restores ownership to mission_task for landing
 * ---------------------------------------------------------------------------
 */
static void mission_task(void *arg) {
  float target_z = -(CRUISE_ALT_M); /* NED: negative = above ground */
  float takeoff_x = 0.0f;
  float takeoff_y = 0.0f;

  /* ------------------------------------------------------------------ */
  /* Phase 1: Wait for valid telemetry                                   */
  /* ------------------------------------------------------------------ */
  ESP_LOGI(TAG, "Waiting for telemetry...");
  while (1) {
    drone_state_t st = mavlink_get_state();
    if (mavlink_position_valid() && st.last_hb_ms != 0)
      break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI(TAG, "Telemetry valid");

  /* ------------------------------------------------------------------ */
  /* Phase 2: Pre-stream hold setpoint                                   */
  /* PX4 requires setpoints to already be streaming before it will       */
  /* accept an OFFBOARD mode switch.  Stream for 2 s.                    */
  /* ------------------------------------------------------------------ */
  mavlink_set_hold();
  ESP_LOGI(TAG, "Pre-streaming hold setpoint for 2 s...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* ------------------------------------------------------------------ */
  /* Pre-arm safety: require all ToF sensors to be initialised.          */
  /* If any sensor failed, refuse to arm — flying without full obstacle  */
  /* coverage is too dangerous.  Log every 2 s so the issue is obvious.  */
  /* ------------------------------------------------------------------ */
  {
    int tof_ok = tof_sensors_ok_count();
    while (tof_ok < TOF_SENSOR_COUNT) {
      ESP_LOGE(TAG, "TOF CHECK FAILED: %d / %d sensors OK — refusing to arm",
               tof_ok, TOF_SENSOR_COUNT);
      vTaskDelay(pdMS_TO_TICKS(2000));
      tof_ok = tof_sensors_ok_count();
    }
    ESP_LOGI(TAG, "All %d ToF sensors OK — proceeding to arm", tof_ok);
  }

  /* ------------------------------------------------------------------ */
  /* Phase 3a: Switch to OFFBOARD mode                                   */
  /* Retry every 500 ms until PX4 confirms via heartbeat custom_main_mode */
  /* ------------------------------------------------------------------ */
  ESP_LOGI(TAG, "Requesting OFFBOARD mode...");
  while (1) {
    mavlink_set_offboard_mode();
    vTaskDelay(pdMS_TO_TICKS(500));
    drone_state_t cur_st = mavlink_get_state();
    if (cur_st.offboard_ack ||
        cur_st.custom_main_mode == PX4_MAIN_MODE_OFFBOARD ||
        cur_st.custom_main_mode == 10 ||
        ((cur_st.raw_custom_mode >> 8) & 0xFF) == PX4_MAIN_MODE_OFFBOARD) {
      ESP_LOGI(TAG, "OFFBOARD mode confirmed (ack=%d mode=%u raw=0x%08lX)",
               cur_st.offboard_ack, cur_st.custom_main_mode, (unsigned long)cur_st.raw_custom_mode);
      break;
    }
    ESP_LOGW(TAG, "OFFBOARD not set yet (mode=%u sub=%u armed=%d pos=[%.2f, %.2f, %.2f]) — retrying",
             cur_st.custom_main_mode, cur_st.custom_sub_mode, cur_st.armed,
             cur_st.x, cur_st.y, cur_st.z);
  }

  /* ------------------------------------------------------------------ */
  /* Phase 3b: Arm                                                        */
  /* Retry every 500 ms until PX4 confirms armed via heartbeat           */
  /* ------------------------------------------------------------------ */
  ESP_LOGI(TAG, "Arming...");
  while (1) {
    mavlink_arm(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    drone_state_t cur_st = mavlink_get_state();
    if (cur_st.armed) {
      ESP_LOGI(TAG, "Armed confirmed");
      break;
    }
    ESP_LOGW(TAG, "Not armed yet (mode=%u armed=%d) — retrying",
             cur_st.custom_main_mode, cur_st.armed);
  }

  /* ------------------------------------------------------------------ */
  /* Phase 4: Take off to cruise altitude                                */
  /* nav_task is still IDLE — mission_task owns the setpoint.            */
  /* ------------------------------------------------------------------ */
  {
    drone_state_t st = mavlink_get_state();
    takeoff_x = st.x;
    takeoff_y = st.y;
    mavlink_set_position_ned(takeoff_x, takeoff_y, target_z, st.heading);
  }
  ESP_LOGI(TAG, "Taking off to %.1f m AGL (NED z=%.2f)...", CRUISE_ALT_M,
           target_z);

  for (int i = 0; i < TAKEOFF_TIMEOUT_S * 10; i++) {
    if (fabsf(mavlink_get_state().z - target_z) < ALT_TOLERANCE_M)
      break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI(TAG, "Altitude reached: NED z=%.2f (target=%.2f)",
           mavlink_get_state().z, target_z);

  vTaskDelay(pdMS_TO_TICKS(1000));

  /* Record initial takeoff pose node via SLAM bridge */
  slam_bridge_record_step();

  /* ------------------------------------------------------------------ */
  /* Phase 5: Autonomous Multi-Waypoint Exploration & SLAM Mapping       */
  /* ------------------------------------------------------------------ */
  ESP_LOGI(TAG, "Autonomous patrol mode (WiFi and AprilTag disabled)...");

  /* Multi-waypoint loop patrol to collect rich 360 point cloud SLAM data.
   * Goals are in the map frame: takeoff_x/y are raw PX4 (used for the takeoff
   * setpoint), which differs from the map once SLAM corrections or PX4
   * position resets have been applied. */
  float home_x, home_y;
  odom_to_map(takeoff_x, takeoff_y, &home_x, &home_y);
  float waypoints[4][2] = {
      {home_x + GOAL_OFFSET_X_M, home_y + GOAL_OFFSET_Y_M},
      {home_x + GOAL_OFFSET_X_M, home_y},
      {home_x, home_y + GOAL_OFFSET_Y_M},
      {home_x, home_y}
  };
  int current_wp_idx = 0;

  ESP_LOGI(TAG, "Navigating to waypoint 0 (%.2f, %.2f, %.2f)...",
           waypoints[0][0], waypoints[0][1], target_z);
  nav_set_goal_map(waypoints[0][0], waypoints[0][1], target_z);

  uint32_t last_scan_ms = 0;
  uint32_t flight_start_ms = (uint32_t)(esp_timer_get_time() / 1000);

  while (1) {
    /* Periodic SLAM pose & scan recording */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (now_ms - last_scan_ms > SLAM_RECORD_INTERVAL_MS) {
      last_scan_ms = now_ms;
      slam_bridge_record_step();
    }

    nav_status_t ns = nav_get_status();
    if (ns.state == NAV_ARRIVED) {
      /* Advance to next patrol waypoint to keep moving and mapping */
      current_wp_idx = (current_wp_idx + 1) % 4;
      ESP_LOGI(TAG, "Reached waypoint -> moving to waypoint %d (%.2f, %.2f)",
               current_wp_idx, waypoints[current_wp_idx][0], waypoints[current_wp_idx][1]);
      nav_set_goal_map(waypoints[current_wp_idx][0], waypoints[current_wp_idx][1], target_z);
    }

    /* Check if autonomous flight duration elapsed */
    if ((now_ms - flight_start_ms) > (AUTONOMOUS_FLIGHT_DURATION_S * 1000)) {
      ESP_LOGI(TAG, "Autonomous flight duration (%ds) complete — initiating landing",
               AUTONOMOUS_FLIGHT_DURATION_S);
      nav_cancel();
      goto do_land;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }

  /* ------------------------------------------------------------------ */
  /* Phase 6: Land & Run Post-Flight SLAM Optimization                   */
  /* ------------------------------------------------------------------ */
do_land:
  mavlink_send_land_command();
  ESP_LOGI(TAG, "Land command sent");

  for (int i = 0; i < LAND_TIMEOUT_S * 10; i++) {
    if (!mavlink_get_state().armed) {
      ESP_LOGI(TAG, "Disarmed — mission complete");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  /* If still armed after landing timeout, force disarm */
  if (mavlink_get_state().armed) {
    ESP_LOGW(TAG, "Still armed after landing timeout — forcing DISARM");
    mavlink_arm(false);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  /* Execute full graph SLAM optimization and JSON export via bridge */
  ESP_LOGI(TAG, "Triggering final post-flight Graph LS-SLAM optimization...");
  slam_bridge_run_optimization();
  slam_bridge_export_json();
  log_memory_stats("post_slam");

  vTaskDelete(NULL);
}

/* ---------------------------------------------------------------------------
 * app_main — init and spawn all active tasks
 * ---------------------------------------------------------------------------
 */
void app_main(void) {
  ESP_LOGI(TAG, "ESP starting with Integrated Swarm SLAM (Combine 2 - Standalone Mode)");

  /* NVS init */
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  /* Init all active modules before spawning tasks */
  mavlink_task_init();
  tof_task_init();
  nav_task_init();
  odom_init();
  slam_bridge_init();

  log_memory_stats("app_main_after_init");

  /* Core 0: hardware-facing tasks (MAVLink UART, ToF I2C MUX) */
  xTaskCreatePinnedToCore(mavlink_task, "mav", MAV_TASK_STACK, NULL,
                          MAV_TASK_PRIORITY, NULL, MAV_TASK_CORE);
  xTaskCreatePinnedToCore(tof_task, "tof", TOF_TASK_STACK, NULL,
                          TOF_TASK_PRIORITY, NULL, TOF_TASK_CORE);

  /* Core 1: navigator (Pri 3) + autonomous mission & SLAM (Pri 2) */
  xTaskCreatePinnedToCore(nav_task, "nav", NAV_TASK_STACK, NULL,
                          NAV_TASK_PRIORITY, NULL, NAV_TASK_CORE);
  xTaskCreatePinnedToCore(mission_task, "mission", 40960, NULL, 2, NULL, 1);

  ESP_LOGI(TAG, "All active tasks spawned (AprilTag and WiFi disabled)");
}
