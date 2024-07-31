#include <time.h>
#include <sys/time.h>
#include "ntp_client.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "esp_sntp.h"

#define TAG "NTP_CLIENT"

static time_t local_time;

void init_ntp_client() {
    esp_sntp_stop();
    ESP_LOGI(TAG, "Initializing NTP client");

    time(&local_time);  // Initialize with current system time

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}

void deinit_ntp_client() {
    ESP_LOGI(TAG, "Stopping and deinitializing ntp client");
    esp_sntp_stop();
    esp_netif_sntp_deinit();
}