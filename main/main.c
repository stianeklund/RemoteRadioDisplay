#include "cat_parser.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "gfx/lcd_init.h"
#include "gfx/lvgl_init.h"
#include "gfx/touch_init.h"
#include "ntp_client.h"
#include "nvs_flash.h"
#include "uart.h"
#include "ui.h"
#include "wifi_init.h"
#include <stdio.h>


static const char *TAG = "MAIN";
void monitoring_task(void *pvParameters);

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "Initializing WiFi");
  wifi_init();

  ESP_LOGI(TAG, "Initializing LCD and UI");
  esp_lcd_panel_handle_t panel_handle = NULL;
  ret = lcd_init(&panel_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init LCD: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "LCD initialized");

  // Initialize touch
  esp_lcd_touch_handle_t tp = NULL;
  ret = touch_init(&tp);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize touch: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "Touch initialized");

  // Initialize LVGL
  ret = lvgl_init(panel_handle, tp);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize LVGL: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "LVGL initialized");

  // Initialize UI
  ui_init();
  ESP_LOGI(TAG, "UI initialized");

  init_ntp_client();
  init_uart();

  xTaskCreate(monitoring_task, "monitor", 4096, NULL, 1, NULL);

  // Main application loop
  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    read_uart();
    }
}

void monitoring_task(void *pvParameters) {
  while (1) {
    ESP_LOGI(TAG, "--- System Status ---");

    // Log heap usage
    ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Minimum free heap since boot: %d bytes",
             esp_get_minimum_free_heap_size());

    // Log uptime
    ESP_LOGI(TAG, "Uptime: %lld s", esp_timer_get_time() / 1000000);

    vTaskDelay(pdMS_TO_TICKS(5000)); // Run every 3 seconds
  }
}