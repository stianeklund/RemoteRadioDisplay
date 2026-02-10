#ifndef WIFI_INIT_H
#define WIFI_INIT_H

#include "esp_err.h"
#include <stdbool.h>

// Start WiFi connection (non-blocking). Returns immediately after starting connection attempt.
// Use wifi_is_connected() to check status, or wifi_wait_connected() to block until connected.
esp_err_t wifi_init_start(void);

// Wait for WiFi connection with timeout. Returns ESP_OK if connected, ESP_FAIL if failed,
// ESP_ERR_TIMEOUT if timeout expired.
esp_err_t wifi_wait_connected(uint32_t timeout_ms);

// Check if WiFi is currently connected.
bool wifi_is_connected(void);

// Legacy blocking init (calls wifi_init_start + wifi_wait_connected with 30s timeout)
esp_err_t wifi_init(void);

#endif // WIFI_INIT_H
