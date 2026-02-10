#include "wifi_init.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"  // For esp_timer_get_time() used in timeout monitoring
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "ntp_client.h"  // Start NTP once we have network
#include "websocket_client.h"  // Start WebSocket client once we have network
#include "mdns.h"  // For mDNS cleanup
#include "settings_storage.h"  // For antenna switch setting

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi_init";
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;
static bool s_wifi_started = false;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

esp_err_t wifi_init_start(void)
{
    if (s_wifi_started) {
        ESP_LOGW(TAG, "WiFi already started");
        return ESP_OK;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .threshold = {
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_started = true;
    ESP_LOGI(TAG, "WiFi started (non-blocking), connecting to SSID:%s", WIFI_SSID);

    return ESP_OK;
}

esp_err_t wifi_wait_connected(uint32_t timeout_ms)
{
    if (!s_wifi_started || s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "WiFi not started, call wifi_init_start() first");
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t wait_start_time = esp_timer_get_time() / 1000;

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

    uint32_t wait_duration = (esp_timer_get_time() / 1000) - wait_start_time;

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s (%lums)", WIFI_SSID, wait_duration);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s (%lums)", WIFI_SSID, wait_duration);
        return ESP_FAIL;
    } else {
        ESP_LOGW(TAG, "WiFi connection still pending after %lums", wait_duration);
        return ESP_ERR_TIMEOUT;
    }
}

bool wifi_is_connected(void)
{
    if (!s_wifi_started || s_wifi_event_group == NULL) {
        return false;
    }
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t wifi_init(void)
{
    esp_err_t ret = wifi_init_start();
    if (ret != ESP_OK) {
        return ret;
    }
    return wifi_wait_connected(30000);  // 30 second timeout for legacy behavior
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");

        // Network went down; stop/deinit SNTP to avoid stale state
        deinit_ntp_client();
        ESP_LOGI(TAG, "NTP client deinitialized due to WiFi disconnect");
        
        // Stop WebSocket client as well
        websocket_client_stop();
        ESP_LOGI(TAG, "WebSocket client stopped due to WiFi disconnect");
        
        // Clean up mDNS if it was initialized
        mdns_free();
        ESP_LOGI(TAG, "mDNS deinitialized due to WiFi disconnect");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Initialize and start NTP as soon as we have an IP address
        esp_err_t ntp_ret = init_ntp_client();
        if (ntp_ret != ESP_OK) {
            ESP_LOGW(TAG, "NTP client init on IP event failed: %s", esp_err_to_name(ntp_ret));
        } else {
            ESP_LOGI(TAG, "NTP client started after IP acquisition");
        }
        
        // Initialize and start WebSocket client for antenna control (if enabled in settings)
        user_settings_t settings;
        esp_err_t set_ret = settings_load(&settings);

        if (set_ret == ESP_OK && settings.antenna_switch_enabled) {
            ESP_LOGI(TAG, "Antenna switch enabled in settings - initializing WebSocket client");
            esp_err_t ws_ret = websocket_client_init();
            if (ws_ret == ESP_OK) {
                ws_ret = websocket_client_start();
                if (ws_ret != ESP_OK) {
                    ESP_LOGW(TAG, "WebSocket client start failed: %s", esp_err_to_name(ws_ret));
                } else {
                    ESP_LOGI(TAG, "WebSocket client started after IP acquisition");
                }
            } else {
                ESP_LOGW(TAG, "WebSocket client init failed: %s", esp_err_to_name(ws_ret));
            }
        } else {
            if (set_ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to load settings for antenna switch check: %s", esp_err_to_name(set_ret));
            } else {
                ESP_LOGI(TAG, "Antenna switch disabled in settings - WebSocket client NOT initialized");
            }
        }
    }
}
