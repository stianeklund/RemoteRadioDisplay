#ifndef NTP_CLIENT_H
#define NTP_CLIENT_H
#include "esp_err.h"
#include <time.h>

esp_err_t init_ntp_client(void);
void deinit_ntp_client(void);
esp_err_t ntp_get_time(struct tm *timeinfo);
/**
 * @brief Callback function that is triggered when NTP synchronization occurs
 *
 * This function formats the current time and updates the UI element
 * through LVGL's async mechanism to ensure thread safety.
 *
 * @param tv Pointer to the timeval structure containing the synchronized time
 */
void ntp_sync_callback(struct timeval *tv);

/**
 * @brief LVGL task to update the UTC time label.
 * To be called via lv_async_call.
 * @param data Pointer to a character string (e.g., "HH:MM:SS") to display.
 *             The string must persist until the task executes.
 */
void _lvgl_update_utc_time_label_task(void *data);

#endif // NTP_CLIENT_H
