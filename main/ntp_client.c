#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_newlib.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "esp_check.h"
#include "ntp_client.h"

static const char *TAG = "NTP_CLIENT";

#define NTP_SERVER "pool.ntp.org"
#define RETRY_COUNT 5
#define RETRY_DELAY_MS 1500

esp_err_t init_ntp_client(void)
{
    ESP_LOGI(TAG, "Initializing NTP client");

    esp_err_t ret = ESP_OK;

    // Stop SNTP operation if it was running before
    esp_sntp_stop();

    // Use UTC time
    setenv("TZ", "UTC", 1);
    tzset();

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(NTP_SERVER);
    config.sync_cb = ntp_sync_callback;

    // Initialize SNTP
    ret = esp_netif_sntp_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SNTP: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for time to be set
    int retry = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < RETRY_COUNT) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    if (retry == RETRY_COUNT) {
        ESP_LOGE(TAG, "Failed to set system time");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void deinit_ntp_client(void)
{
    ESP_LOGI(TAG, "Stopping and deinitializing NTP client");
    esp_netif_sntp_deinit();
}

// Optional: Callback function for when NTP sync occurs
void ntp_sync_callback(struct timeval *tv)
{
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);
    localtime_r(&now, &timeinfo);
    // Print the current time
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "NTP Time synchronized. Time is: %s", strftime_buf);
}