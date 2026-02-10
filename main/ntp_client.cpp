#include "ntp_client.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include <sys/time.h>
#include <time.h>

// LVGL and UI includes
#include "lvgl.h"
#include "ui/ui.h"
#include "radio/radio_subjects.h"
#include "radio/radio_subject_updater.h"


static const char *TAG = "NTP_CLIENT";

// Static buffer to hold the formatted time string for LVGL.
// It needs to be static or global because lv_async_call is asynchronous.
static char ntp_time_buffer_for_lvgl[10]; // "HH:MM:SS\0"
static volatile bool s_ntp_synced = false; // Set true in sync callback

/**
 * @brief LVGL task to update the UTC time label.
 * This function is called by lv_async_call from the LVGL task context.
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
void _lvgl_update_utc_time_label_task(void *data) { // Made non-static
    if (ui_UtcTime) {
        // Ensure the UI element is initialized
        // ui_UtcTime is now an lv_label, so use lv_label_set_text
        lv_label_set_text(ui_UtcTime, (const char *) data);
    } else {
        ESP_LOGW(TAG, "ui_UtcTime not initialized when trying to update time.");
    }
}

#define NTP_SERVER "pool.ntp.org"
#define RETRY_COUNT 5
#define RETRY_DELAY_MS 1500

esp_err_t init_ntp_client(void) {
    ESP_LOGI(TAG, "Initializing NTP client");

    // Ensure timezone is UTC for consistent UI display
    setenv("TZ", "UTC", 1);
    tzset();

    // Deinit any previous SNTP instance (idempotent safety)
    esp_netif_sntp_deinit();

    // Configure SNTP
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(NTP_SERVER);
    config.sync_cb = ntp_sync_callback;

    esp_err_t ret = esp_netif_sntp_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SNTP init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Some IDF versions require explicit start
    ret = esp_netif_sntp_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // INVALID_STATE means already started
        ESP_LOGE(TAG, "SNTP start failed: %s", esp_err_to_name(ret));
        esp_netif_sntp_deinit();
        return ret;
    }

    // Observe initial sync briefly but treat service start as success
    s_ntp_synced = false;
    int retry = 0;
    while (++retry <= RETRY_COUNT) {
        // If callback already fired, exit early
        if (s_ntp_synced) {
            ESP_LOGI(TAG, "Initial SNTP sync observed via callback");
            return ESP_OK;
        }
        // Alternatively, consider time valid if year is sane
        time_t now; struct tm ti;
        time(&now);
        localtime_r(&now, &ti);
        if (ti.tm_year >= (2016 - 1900)) {
            ESP_LOGI(TAG, "System time appears valid (year=%d)", ti.tm_year + 1900);
            return ESP_OK;
        }
        ESP_LOGI(TAG, "SNTP syncing... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    // No sync observed yet; keep SNTP running in background for later update
    ESP_LOGW(TAG, "SNTP started; initial sync not yet observed");
    return ESP_OK;
}

void deinit_ntp_client(void) {
    ESP_LOGI(TAG, "Stopping and deinitializing NTP client");
    esp_netif_sntp_deinit();
}

esp_err_t ntp_get_time(struct tm *timeinfo) {
    time_t now;
    time(&now);
    localtime_r(&now, timeinfo);

    // Check if the year is valid (e.g., after 2016, as in original code)
    if (timeinfo->tm_year < (2016 - 1900)) {
        ESP_LOGW(TAG, "Time might not be accurately set yet (year: %d)", 1900 + timeinfo->tm_year);
        // Depending on strictness, you might return ESP_FAIL or allow it.
        // For now, let's stick to the original logic.
        return ESP_FAIL;
    }

    return ESP_OK;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void ntp_sync_callback(struct timeval *tv) {
    s_ntp_synced = true;
    struct tm timeinfo;

    // Use the timeval parameter provided by the NTP callback
    // instead of calling time() again
    if (tv != NULL) {
        // Convert the timeval to tm structure
        localtime_r(&tv->tv_sec, &timeinfo);

        // Format the time string specifically for the ui_UtcTime label ("HH:MM:SS")
        strftime(ntp_time_buffer_for_lvgl, sizeof(ntp_time_buffer_for_lvgl),
                 "%H:%M:%S", &timeinfo);

        ESP_LOGI(TAG, "NTP Time synchronized. Updating display to: %s",
                 ntp_time_buffer_for_lvgl);

        // Notify UI via LVGL 9 native observer
        radio_subject_set_int_async(&radio_time_subject, (int32_t)tv->tv_sec);
    } else {
        ESP_LOGW(TAG, "NTP callback received NULL timeval pointer");
    }
}
