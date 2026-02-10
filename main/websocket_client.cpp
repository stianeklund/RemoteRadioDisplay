#include "websocket_client.h"
#include "antenna_control.h"
#include "cat_parser.h"
#include "memory_monitor.h"
#include "radio/radio_subjects.h"
#include "radio/radio_subject_updater.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"  // For memory monitoring in error handler
#include "mdns.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lvgl.h"
#include "ui.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "WS_CLIENT";

static esp_websocket_client_handle_t g_ws_client = NULL;
static ws_client_state_t g_client_state = WS_CLIENT_STATE_DISCONNECTED;
static char g_server_uri[128] = {0};

// Cache to avoid redundant status requests
static uint32_t g_last_status_request_id = 0;
static TickType_t g_last_status_request_time = 0;
#define STATUS_REQUEST_COOLDOWN_MS 200  // Minimum 200ms between status requests for responsive UI

// Optional application-layer heartbeat (status poll) to supplement WS ping/pong
static esp_timer_handle_t g_antenna_poll_timer = NULL;
#ifdef CONFIG_ANTENNA_APP_HEARTBEAT_ENABLE
#define ANTENNA_POLL_INTERVAL_MS ((CONFIG_ANTENNA_APP_HEARTBEAT_INTERVAL) * 1000)
#else
#define ANTENNA_POLL_INTERVAL_MS 30000
#endif

// Forward declarations
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static esp_err_t build_websocket_uri(char *uri_buf, size_t uri_buf_len);
static esp_err_t send_websocket_message(const char *message);
static esp_err_t send_cjson_message(cJSON *message);
static void process_websocket_message(const char *message, size_t len);
static uint32_t generate_request_id(void);
static void antenna_poll_timer_callback(void* arg);
static esp_err_t start_antenna_polling(void);
static esp_err_t stop_antenna_polling(void);
static const char *ws_error_type_str(int error_type);


esp_err_t websocket_client_init(void)
{
    if (g_ws_client != NULL) {
        ESP_LOGI(TAG, "WebSocket client already initialized");
        return ESP_OK;
    }

#ifdef CONFIG_ANTENNA_ENABLE_MDNS_LOOKUP
    // Initialize mDNS for hostname resolution
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize mDNS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "mDNS initialized for hostname resolution");
    }
#endif

    // Build WebSocket URI
    ret = build_websocket_uri(g_server_uri, sizeof(g_server_uri));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build WebSocket URI: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WebSocket client will connect to: %s", g_server_uri);
    ESP_LOGI(TAG, "WebSocket client initialized");
    return ESP_OK;
}

esp_err_t websocket_client_start(void)
{
    if (g_ws_client != NULL) {
        ESP_LOGI(TAG, "WebSocket client already started");
        return ESP_OK;
    }

    esp_websocket_client_config_t websocket_cfg = {
        .uri = g_server_uri,
        .disable_auto_reconnect = false,
        .task_prio = 5,
        .task_stack = 6144,                 // Increase from 4KB to 6KB for stability
        .buffer_size = 8192,                // Match server WS_MAX_MESSAGE_SIZE
        .user_agent = "ESP32-AntennaController/1.0",
        .pingpong_timeout_sec = 40,         // Wait 40s for pong (server timeout buffer)
        .disable_pingpong_discon = false,   // Enable disconnection on pong timeout
        .reconnect_timeout_ms = CONFIG_ANTENNA_RECONNECT_INTERVAL * 1000,
        .network_timeout_ms = 10000,        // 10s network timeout
        .ping_interval_sec = 25,            // Send ping every 25s (before server's 30s±5s)
    };

    g_ws_client = esp_websocket_client_init(&websocket_cfg);
    if (g_ws_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize WebSocket client");
        return ESP_FAIL;
    }

    esp_websocket_register_events(g_ws_client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);

    g_client_state = WS_CLIENT_STATE_CONNECTING;
    esp_err_t ret = esp_websocket_client_start(g_ws_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WebSocket client: %s", esp_err_to_name(ret));
        g_client_state = WS_CLIENT_STATE_ERROR;
        return ret;
    }

    ESP_LOGI(TAG, "WebSocket client started, connecting to %s", g_server_uri);
    return ESP_OK;
}

esp_err_t websocket_client_stop(void)
{
    // Stop periodic polling first
    stop_antenna_polling();
    
    if (g_ws_client) {
        esp_websocket_client_stop(g_ws_client);
        esp_websocket_client_destroy(g_ws_client);
        g_ws_client = NULL;
    }

    g_client_state = WS_CLIENT_STATE_DISCONNECTED;
    ESP_LOGI(TAG, "WebSocket client stopped");
    return ESP_OK;
}

esp_err_t websocket_client_cleanup(void)
{
    // Stop client and tasks first
    websocket_client_stop();

#ifdef CONFIG_ANTENNA_ENABLE_MDNS_LOOKUP
    // Clean up mDNS (if initialized)
    mdns_free();
#endif

    ESP_LOGI(TAG, "WebSocket client cleanup completed");
    return ESP_OK;
}

ws_client_state_t websocket_client_get_state(void)
{
    return g_client_state;
}

esp_err_t websocket_client_set_relay(uint8_t relay_id, bool state)
{
    if (!websocket_client_is_connected()) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot set relay");
        return ESP_ERR_INVALID_STATE;
    }

    cJSON *message = cJSON_CreateObject();
    cJSON *data = cJSON_CreateObject();

    char request_id[16];
    snprintf(request_id, sizeof(request_id), "req-%lu", generate_request_id());

    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "relay_control");
    cJSON_AddNumberToObject(data, "relay", relay_id);
    cJSON_AddBoolToObject(data, "state", state);
    cJSON_AddItemToObject(message, "data", data);

    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

esp_err_t websocket_client_select_antenna(const char *radio, uint8_t antenna_id)
{
    if (!websocket_client_is_connected()) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot select antenna");
        return ESP_ERR_INVALID_STATE;
    }

    if (!radio || (strcmp(radio, "A") != 0 && strcmp(radio, "B") != 0)) {
        ESP_LOGE(TAG, "Invalid radio parameter: %s (must be 'A' or 'B')", radio ? radio : "NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Selecting antenna %d for radio %s via antenna_switch", antenna_id, radio);

    cJSON *message = cJSON_CreateObject();
    cJSON *data = cJSON_CreateObject();

    char request_id[16];
    snprintf(request_id, sizeof(request_id), "req-%lu", generate_request_id());

    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "antenna_switch");
    cJSON_AddStringToObject(data, "radio", radio);
    cJSON_AddNumberToObject(data, "antenna", antenna_id);
    cJSON_AddItemToObject(message, "data", data);

    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

esp_err_t websocket_client_get_status(void)
{
    if (!websocket_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Rate limiting: avoid excessive status requests
    TickType_t current_time = xTaskGetTickCount();
    if ((current_time - g_last_status_request_time) < pdMS_TO_TICKS(STATUS_REQUEST_COOLDOWN_MS)) {
        ESP_LOGD(TAG, "Status request rate limited - too frequent");
        return ESP_OK; // Not an error, just rate limited
    }

    g_last_status_request_time = current_time;

    cJSON *message = cJSON_CreateObject();

    char request_id[16];
    g_last_status_request_id = generate_request_id();
    snprintf(request_id, sizeof(request_id), "req-%lu", g_last_status_request_id);

    cJSON *data = cJSON_CreateObject();
    
    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "status");
    cJSON_AddItemToObject(message, "data", data);

    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

esp_err_t websocket_client_get_relay_names(void)
{
    if (!websocket_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    cJSON *message = cJSON_CreateObject();

    char request_id[16];
    snprintf(request_id, sizeof(request_id), "req-%lu", generate_request_id());

    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "relay_names");

    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

esp_err_t websocket_client_subscribe_events(void)
{
    if (!websocket_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    cJSON *message = cJSON_CreateObject();
    cJSON *data = cJSON_CreateObject();
    cJSON *events = cJSON_CreateArray();

    char request_id[16];
    snprintf(request_id, sizeof(request_id), "req-%lu", generate_request_id());

    // Subscribe to relevant events
    cJSON_AddItemToArray(events, cJSON_CreateString("status_updates"));
    cJSON_AddItemToArray(events, cJSON_CreateString("relay_state_changes"));
    cJSON_AddItemToArray(events, cJSON_CreateString("frequency_changes"));

    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "subscribe");
    cJSON_AddItemToObject(data, "events", events);
    cJSON_AddItemToObject(message, "data", data);

    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

esp_err_t websocket_client_antenna_switch_next(const char *radio)
{
    if (!websocket_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!radio || (strcmp(radio, "A") != 0 && strcmp(radio, "B") != 0)) {
        ESP_LOGE(TAG, "Invalid radio parameter: %s (must be 'A' or 'B')", radio ? radio : "NULL");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *message = cJSON_CreateObject();
    cJSON *data = cJSON_CreateObject();

    char request_id[16];
    snprintf(request_id, sizeof(request_id), "req-%lu", generate_request_id());

    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "antenna_switch");
    cJSON_AddStringToObject(data, "radio", radio);
    cJSON_AddStringToObject(data, "action", "next");
    cJSON_AddItemToObject(message, "data", data);

    ESP_LOGI(TAG, "Sending antenna switch next command for radio %s", radio);
    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

esp_err_t websocket_client_antenna_switch_prev(const char *radio)
{
    if (!websocket_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!radio || (strcmp(radio, "A") != 0 && strcmp(radio, "B") != 0)) {
        ESP_LOGE(TAG, "Invalid radio parameter: %s (must be 'A' or 'B')", radio ? radio : "NULL");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *message = cJSON_CreateObject();
    cJSON *data = cJSON_CreateObject();

    char request_id[16];
    snprintf(request_id, sizeof(request_id), "req-%lu", generate_request_id());

    cJSON_AddStringToObject(message, "id", request_id);
    cJSON_AddStringToObject(message, "type", "request");
    cJSON_AddStringToObject(message, "action", "antenna_switch");
    cJSON_AddStringToObject(data, "radio", radio);
    cJSON_AddStringToObject(data, "action", "prev");
    cJSON_AddItemToObject(message, "data", data);

    ESP_LOGI(TAG, "Sending antenna switch previous command for radio %s", radio);
    esp_err_t ret = send_cjson_message(message);

    cJSON_Delete(message);
    return ret;
}

// Helper functions
static esp_err_t build_websocket_uri(char *uri_buf, size_t uri_buf_len)
{
    // WebSocket client task is not monitored by watchdog due to long network operations
    // IMPORTANT: Do NOT resolve hostname here to prevent memory corruption during reconnection
    // Let the ESP WebSocket client handle hostname resolution internally

    const char *server_address = CONFIG_ANTENNA_SERVER_HOSTNAME;

    // Build URI directly with hostname - ESP WebSocket client will handle resolution
    int ret = snprintf(uri_buf, uri_buf_len, "ws://%s:%d%s",
                       server_address, CONFIG_ANTENNA_SERVER_PORT, CONFIG_ANTENNA_WS_URI);

    if (ret >= uri_buf_len) {
        ESP_LOGE(TAG, "WebSocket URI too long for buffer");
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}


static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket connected to antenna controller");
            ESP_LOGI(TAG, "Keepalive: Client pings every 25s, server pings every 30s±5s, pong timeout 40s");
            g_client_state = WS_CLIENT_STATE_CONNECTED;
            
            // Automatically request initial status and subscribe to events upon connection
            ESP_LOGD(TAG, "Auto-requesting antenna status on connection");
            
            // Send all requests immediately without delay for fastest response
            websocket_client_subscribe_events();
            websocket_client_get_status(); 
            websocket_client_get_relay_names();
            
            // Optional: Start application heartbeat if enabled
#ifdef CONFIG_ANTENNA_APP_HEARTBEAT_ENABLE
            start_antenna_polling();
#else
            // Rely on server status_update events only
            // start_antenna_polling();
#endif
            
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "WebSocket disconnected from antenna controller (cause=%s, sock_errno=%d, tls_err=0x%x, hs_code=%d)",
                    ws_error_type_str(data->error_handle.error_type),
                    data->error_handle.esp_transport_sock_errno,
                    data->error_handle.esp_tls_last_esp_err,
                    data->error_handle.esp_ws_handshake_status_code);
            g_client_state = WS_CLIENT_STATE_DISCONNECTED;
            
            // Stop periodic polling when disconnected
            stop_antenna_polling();
            
            // The component will attempt to reconnect automatically.
            break;

        case WEBSOCKET_EVENT_DATA:
            if (data->op_code == 0x01) { // Text frame
                ESP_LOGI(TAG, "Received WebSocket message (%zu bytes): %.*s",
                         data->data_len, (int)data->data_len, (char*)data->data_ptr);
                process_websocket_message((char*)data->data_ptr, data->data_len);
            } else if (data->op_code == 0x09 || data->op_code == 0x0a) { // Ping/Pong frames
                // DIAG: Visible marker for jitter correlation - runs every 25s
                ESP_LOGW(TAG, "[DIAG] WebSocket %s (25s keepalive)",
                         data->op_code == 0x09 ? "PING" : "PONG");
            } else {
                ESP_LOGW(TAG, "Received non-text WebSocket frame (op_code=0x%02x)", data->op_code);
            }
            break;

        case WEBSOCKET_EVENT_CLOSED:
            ESP_LOGW(TAG, "WebSocket closed cleanly by server (cause=%s)",
                    ws_error_type_str(data->error_handle.error_type));
            // No state change here; DISCONNECTED already handled transition/reconnect
            break;

        case WEBSOCKET_EVENT_ERROR: {
            ESP_LOGE(TAG, "WebSocket error: type=%s, sock_errno=%d, esp_tls_last_esp_err=0x%x, hs_code=%d",
                     ws_error_type_str(data->error_handle.error_type),
                     data->error_handle.esp_transport_sock_errno,
                     data->error_handle.esp_tls_last_esp_err,
                     data->error_handle.esp_ws_handshake_status_code);

            // Log memory status when WebSocket errors occur - often memory-related
            size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            size_t spiram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            ESP_LOGE(TAG, "Memory at WebSocket error: Internal=%u bytes, SPIRAM=%u bytes",
                     (unsigned)internal_free, (unsigned)spiram_free);

            if (internal_free < 15000) { // Less than 15KB
                ESP_LOGE(TAG, "WebSocket error likely caused by internal SRAM exhaustion!");
            }

            g_client_state = WS_CLIENT_STATE_ERROR;
            // The component will attempt to reconnect automatically.
            break;
        }

        default:
            ESP_LOGD(TAG, "WebSocket event: %ld", event_id);
            break;
    }
}

// Fast-path parser for simple relay state events to avoid cJSON overhead
static bool try_parse_relay_event_fast(const char *message, size_t len, uint8_t *relay_id, bool *state)
{
    // Look for pattern: {"type":"event","event":"relay_state_changed","data":{"relay":N,"state":true/false}}
    // This is a performance optimization for the most common message type

    const char *relay_pos = strstr(message, "\"relay\":");
    const char *state_pos = strstr(message, "\"state\":");
    const char *event_pos = strstr(message, "\"relay_state_changed\"");

    if (!relay_pos || !state_pos || !event_pos) {
        return false; // Not a simple relay state event
    }

    // Parse relay number (single digit expected: 1-8)
    relay_pos += 8; // Skip "\"relay\":"
    while (*relay_pos == ' ') relay_pos++; // Skip whitespace
    if (*relay_pos >= '1' && *relay_pos <= '8') {
        *relay_id = *relay_pos - '0';
    } else {
        return false; // Invalid format
    }

    // Parse state (true/false)
    state_pos += 8; // Skip "\"state\":"
    while (*state_pos == ' ') state_pos++; // Skip whitespace
    if (strncmp(state_pos, "true", 4) == 0) {
        *state = true;
    } else if (strncmp(state_pos, "false", 5) == 0) {
        *state = false;
    } else {
        return false; // Invalid format
    }

    return true; // Successfully parsed
}

static void process_websocket_message(const char *message, size_t len)
{
    if (len == 0 || !message) {
        ESP_LOGW(TAG, "Received empty WebSocket message");
        return;
    }

    // Fast-path optimization: Try to parse common relay events without cJSON
    uint8_t relay_id;
    bool state;
    if (try_parse_relay_event_fast(message, len, &relay_id, &state)) {
        ESP_LOGI(TAG, "Relay %d state changed to %s via WebSocket event (fast path)",
                 relay_id, state ? "ON" : "OFF");

        // Update antenna state via antenna_control (emits observer notification)
        antenna_update_relay_from_server(relay_id, state);
        return; // Skip expensive cJSON parsing
    }

    // Fall back to full JSON parsing for complex messages
    cJSON *json = cJSON_ParseWithLength(message, len);
    if (!json) {
        ESP_LOGW(TAG, "Failed to parse WebSocket JSON message (%zu bytes): %.*s",
                 len, (int)len, message);
        return;
    }

    cJSON *type_item = cJSON_GetObjectItem(json, "type");
    if (!type_item || !cJSON_IsString(type_item)) {
        ESP_LOGW(TAG, "Missing or invalid message type in JSON (%zu bytes): %.*s",
                 len, (int)len, message);
        cJSON_Delete(json);
        return;
    }

    const char *msg_type = type_item->valuestring;

    if (strcmp(msg_type, "response") == 0) {
        // Handle response messages (status queries, etc.)
        cJSON *data_item = cJSON_GetObjectItem(json, "data");

        if (data_item) {
            // Check if this is a status response with antenna information
            cJSON *antenna_item = cJSON_GetObjectItem(data_item, "antenna");
            cJSON *available_antennas_item = cJSON_GetObjectItem(data_item, "available_antennas");

            if (antenna_item && available_antennas_item) {
                // Parse current antenna (format: "Antenna 1" or "Antenna 3")
                int current_antenna = 0;
                if (cJSON_IsString(antenna_item)) {
                    const char *antenna_str = antenna_item->valuestring;
                    if (strncmp(antenna_str, "Antenna ", 8) == 0) {
                        current_antenna = atoi(antenna_str + 8);
                    }
                }

                // Parse available antennas array
                int available_count = 0;
                uint8_t available_antennas[8] = {0};

                if (cJSON_IsArray(available_antennas_item)) {
                    cJSON *antenna_id = NULL;
                    cJSON_ArrayForEach(antenna_id, available_antennas_item) {
                        if (cJSON_IsNumber(antenna_id) && available_count < 8) {
                            int id = antenna_id->valueint;
                            if (id >= 1 && id <= 8) {
                                available_antennas[available_count++] = (uint8_t)id;
                            }
                        }
                    }
                }

                ESP_LOGI(TAG, "Antenna status: current=%d, available_count=%d", current_antenna, available_count);

                // Debug: Print available antennas array
                ESP_LOGD(TAG, "Available antennas: [%d, %d, %d, %d, %d, %d, %d, %d]",
                         available_antennas[0], available_antennas[1], available_antennas[2], available_antennas[3],
                         available_antennas[4], available_antennas[5], available_antennas[6], available_antennas[7]);

                // Update antenna state via antenna_control (emits observer notification)
                antenna_update_status_from_server((uint8_t)current_antenna, available_antennas, available_count);
            }

            // Check if this is a relay names response
            cJSON *relay_names_item = cJSON_GetObjectItem(data_item, "relay_names");
            if (relay_names_item && cJSON_IsObject(relay_names_item)) {
                ESP_LOGI(TAG, "Received relay names response from API");

                // Parse relay names from response
                // Expected format: {"relay_names": {"1": "Triband", "2": "Inverted V", ...}}
                bool names_updated = false;

                for (int i = 1; i <= MAX_ANTENNA_RELAYS; i++) {
                    char relay_key[4];
                    snprintf(relay_key, sizeof(relay_key), "%d", i);
                    cJSON *name_item = cJSON_GetObjectItem(relay_names_item, relay_key);

                    if (name_item && cJSON_IsString(name_item)) {
                        const char *name = name_item->valuestring;
                        if (name && strlen(name) > 0) {
                            // Update antenna name
                            if (antenna_set_name(i, name) == ESP_OK) {
                                ESP_LOGD(TAG, "Updated relay %d name to: %s", i, name);
                                names_updated = true;
                            }
                        }
                    }
                }

                if (names_updated) {
                    // Save the updated names to NVS for persistence
                    antenna_save_names_to_nvs();

                    // Notify UI via LVGL 9 native observer
                    radio_subject_notify_async(&radio_antenna_names_subject);

                    ESP_LOGI(TAG, "Antenna names updated and cached to NVS");
                }
            }
        }

    } else if (strcmp(msg_type, "event") == 0) {
        // Handle real-time events
        cJSON *event_item = cJSON_GetObjectItem(json, "event");
        cJSON *data_item = cJSON_GetObjectItem(json, "data");

        if (event_item && cJSON_IsString(event_item)) {
            const char *event_name = event_item->valuestring;

            if (strcmp(event_name, "relay_state_changed") == 0 && data_item) {
                // Handle individual relay state changes
                cJSON *relay_item = cJSON_GetObjectItem(data_item, "relay");
                cJSON *state_item = cJSON_GetObjectItem(data_item, "state");

                if (relay_item && cJSON_IsNumber(relay_item) && state_item && cJSON_IsBool(state_item)) {
                    uint8_t relay_id = (uint8_t)relay_item->valueint;
                    bool state = cJSON_IsTrue(state_item);

                    ESP_LOGI(TAG, "Relay %d state changed to %s via WebSocket event",
                             relay_id, state ? "ON" : "OFF");

                    // Update antenna state via antenna_control (emits observer notification)
                    // This updates local state from server - no command is sent back
                    antenna_update_relay_from_server((uint8_t)relay_id, state);
                }
            } else if (strcmp(event_name, "status_update") == 0 && data_item) {
                // Handle full status updates (same as response handling above)
                cJSON *antenna_item = cJSON_GetObjectItem(data_item, "antenna");
                cJSON *available_antennas_item = cJSON_GetObjectItem(data_item, "available_antennas");

                if (antenna_item && available_antennas_item) {
                    int current_antenna = 0;
                    if (cJSON_IsString(antenna_item)) {
                        const char *antenna_str = antenna_item->valuestring;
                        if (strncmp(antenna_str, "Antenna ", 8) == 0) {
                            current_antenna = atoi(antenna_str + 8);
                        }
                    }

                    int available_count = 0;
                    uint8_t available_antennas[8] = {0};

                    if (cJSON_IsArray(available_antennas_item)) {
                        cJSON *antenna_id = NULL;
                        cJSON_ArrayForEach(antenna_id, available_antennas_item) {
                            if (cJSON_IsNumber(antenna_id) && available_count < 8) {
                                int id = antenna_id->valueint;
                                if (id >= 1 && id <= 8) {
                                    available_antennas[available_count++] = (uint8_t)id;
                                }
                            }
                        }
                    }

                    ESP_LOGI(TAG, "Status update event: current=%d, available_count=%d", current_antenna, available_count);

                    // Update antenna state via antenna_control (emits observer notification)
                    antenna_update_status_from_server((uint8_t)current_antenna, available_antennas, available_count);
                }
            }
        }

    } else if (strcmp(msg_type, "error") == 0) {
        cJSON *data_item = cJSON_GetObjectItem(json, "data");
        if (data_item) {
            cJSON *message_item = cJSON_GetObjectItem(data_item, "message");
            if (message_item && cJSON_IsString(message_item)) {
                ESP_LOGW(TAG, "WebSocket error: %s", message_item->valuestring);
            }
        }
    }

    cJSON_Delete(json);
}

static esp_err_t send_websocket_message(const char *message)
{
    if (!g_ws_client || g_client_state != WS_CLIENT_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Cannot send message - WebSocket not connected");
        return ESP_ERR_INVALID_STATE;
    }

    // **CRITICAL FIX**: Replace portMAX_DELAY with 5-second timeout to prevent display freezes
    // portMAX_DELAY was causing infinite blocking when antenna controller unreachable
    const TickType_t ws_send_timeout = pdMS_TO_TICKS(5000); // 5 second timeout
    uint32_t send_start_time = esp_timer_get_time() / 1000;
    
    int ret = esp_websocket_client_send_text(g_ws_client, message, strlen(message), ws_send_timeout);
    
    uint32_t send_duration = (esp_timer_get_time() / 1000) - send_start_time;
    
    if (ret < 0) {
        if (send_duration >= 4500) { // Close to 5s timeout
            ESP_LOGE(TAG, "WebSocket send TIMEOUT after %lums - this was preventing display freezes", send_duration);
            // Report timeout to hardware health monitoring
            hardware_health_report_ws_timeout();
        } else {
            ESP_LOGE(TAG, "Failed to send WebSocket message after %lums", send_duration);
        }
        return ESP_FAIL;
    }

    if (send_duration > 1000) { // Log slow operations > 1s
        ESP_LOGW(TAG, "WebSocket send took %lums (slow but successful)", send_duration);
    }
    
    ESP_LOGD(TAG, "Sent WebSocket message (%lums): %s", send_duration, message);
    return ESP_OK;
}

static esp_err_t send_cjson_message(cJSON *message)
{
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }

    char stack_buffer[256];
    if (cJSON_PrintPreallocated(message, stack_buffer, sizeof(stack_buffer), false)) {
        return send_websocket_message(stack_buffer);
    }

    char *heap_buffer = cJSON_PrintUnformatted(message);
    if (!heap_buffer) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = send_websocket_message(heap_buffer);
    free(heap_buffer);
    return ret;
}


static uint32_t generate_request_id(void)
{
    static uint32_t counter = 0;
    return ++counter;
}

bool websocket_client_is_connected(void)
{
    return g_client_state == WS_CLIENT_STATE_CONNECTED && g_ws_client != NULL;
}

esp_err_t websocket_client_trigger_reconnection(void)
{
    if (g_ws_client == NULL) {
        ESP_LOGW(TAG, "Cannot trigger reconnection - WebSocket client not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_client_state == WS_CLIENT_STATE_CONNECTED) {
        ESP_LOGD(TAG, "Already connected - no reconnection needed");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Triggering immediate WebSocket reconnection...");
    
    // Stop and restart the client to trigger immediate reconnection
    esp_err_t ret = esp_websocket_client_stop(g_ws_client);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop WebSocket client for reconnection: %s", esp_err_to_name(ret));
    }
    
    g_client_state = WS_CLIENT_STATE_CONNECTING;
    ret = esp_websocket_client_start(g_ws_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart WebSocket client: %s", esp_err_to_name(ret));
        g_client_state = WS_CLIENT_STATE_ERROR;
        return ret;
    }
    
    ESP_LOGI(TAG, "WebSocket reconnection attempt initiated");
    return ESP_OK;
}

// Periodic antenna status polling timer callback
static void antenna_poll_timer_callback(void* arg)
{
    if (!websocket_client_is_connected()) {
        ESP_LOGD(TAG, "Skipping antenna status poll - WebSocket not connected");
        return;
    }

    // DIAG: Visible marker for jitter correlation - runs every 30s
    ESP_LOGW(TAG, "[DIAG] >>> Antenna poll START (30s timer)");

    // Request status as fallback in case events aren't working
    esp_err_t ret = websocket_client_get_status();
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "[DIAG] <<< Antenna poll END (success)");
    } else {
        ESP_LOGW(TAG, "[DIAG] <<< Antenna poll END (failed: %s)", esp_err_to_name(ret));
    }
}

// Start periodic antenna status polling
static esp_err_t start_antenna_polling(void)
{
    if (g_antenna_poll_timer != NULL) {
        ESP_LOGD(TAG, "Antenna polling timer already started");
        return ESP_OK;
    }
    
    esp_timer_create_args_t timer_args = {
        .callback = antenna_poll_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "antenna_poll"
    };
    
    esp_err_t ret = esp_timer_create(&timer_args, &g_antenna_poll_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create antenna polling timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_timer_start_periodic(g_antenna_poll_timer, ANTENNA_POLL_INTERVAL_MS * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start antenna polling timer: %s", esp_err_to_name(ret));
        esp_timer_delete(g_antenna_poll_timer);
        g_antenna_poll_timer = NULL;
        return ret;
    }
    
    ESP_LOGD(TAG, "Antenna polling timer started (interval: %d ms)", ANTENNA_POLL_INTERVAL_MS);
    return ESP_OK;
}

// Stop periodic antenna status polling
static esp_err_t stop_antenna_polling(void)
{
    if (g_antenna_poll_timer == NULL) {
        return ESP_OK;
    }
    
    esp_err_t ret = esp_timer_stop(g_antenna_poll_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop antenna polling timer: %s", esp_err_to_name(ret));
    }
    
    ret = esp_timer_delete(g_antenna_poll_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to delete antenna polling timer: %s", esp_err_to_name(ret));
    }
    
    g_antenna_poll_timer = NULL;
    ESP_LOGI(TAG, "Antenna polling timer stopped");
    return ESP_OK;
}

// Convenience functions that default to radio "A" for backward compatibility
esp_err_t websocket_client_select_antenna_default(uint8_t antenna_id)
{
    return websocket_client_select_antenna("A", antenna_id);
}

esp_err_t websocket_client_antenna_switch_next_default(void)
{
    return websocket_client_antenna_switch_next("A");
}

esp_err_t websocket_client_antenna_switch_prev_default(void)
{
    return websocket_client_antenna_switch_prev("A");
}

// Map error type to readable string for logging clarity
static const char *ws_error_type_str(int error_type)
{
    switch (error_type) {
        case WEBSOCKET_ERROR_TYPE_NONE: return "NONE";
        case WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT: return "TCP_TRANSPORT";
        case WEBSOCKET_ERROR_TYPE_PONG_TIMEOUT: return "PONG_TIMEOUT";
        case WEBSOCKET_ERROR_TYPE_HANDSHAKE: return "HANDSHAKE";
        case WEBSOCKET_ERROR_TYPE_SERVER_CLOSE: return "SERVER_CLOSE";
        default: return "UNKNOWN";
    }
}
