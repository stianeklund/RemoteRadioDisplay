#include "antenna_control.h"
#include "websocket_client.h"
#include "esp_log.h"
#include "lvgl.h"
#include "radio/radio_subjects.h"
#include "radio/radio_subject_updater.h"
#include "ui.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ANTENNA_CTRL";
static const char *NVS_ANTENNA_NAMESPACE = "antenna";
static const char *NVS_ANTENNA_NAMES_KEY = "names";

static antenna_system_state_t g_antenna_state = {0};
static bool g_initialized = false;

// Default antenna names
static const char* default_antenna_names[MAX_ANTENNA_RELAYS] = {
    "Triband",
    "Inverted V", 
    "Fan Dipole",
    "Vertical",
    "DXC",
    "Inverted L",
    "Relay 7",
    "Relay 8"
};

// Note: GPIO control removed - antenna switching now done via WebSocket API
// to remote antenna controller server

esp_err_t antenna_control_init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing antenna control system (WebSocket client mode)");

    // Initialize antenna info with defaults
    for (int i = 0; i < MAX_ANTENNA_RELAYS; i++) {
        g_antenna_state.antennas[i].relay_id = i + 1;
        g_antenna_state.antennas[i].state = false;
        g_antenna_state.antennas[i].available = true;
        strncpy(g_antenna_state.antennas[i].name, default_antenna_names[i],
                ANTENNA_NAME_MAX_LENGTH - 1);
        g_antenna_state.antennas[i].name[ANTENNA_NAME_MAX_LENGTH - 1] = '\0';
    }

    g_antenna_state.active_antenna = 0; // No antenna selected initially
    g_antenna_state.transmitting = false;
    g_initialized = true;

    // Note: NVS load deferred to after LVGL init to avoid blocking during critical startup
    // WebSocket will request antenna names from API and cache them for next boot
    ESP_LOGI(TAG, "Antenna control system initialized (WebSocket client mode, NVS load deferred)");
    return ESP_OK;
}

esp_err_t antenna_set_relay(uint8_t relay_id, bool state)
{
    if (!g_initialized) {
        ESP_LOGE(TAG, "Antenna control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (relay_id < 1 || relay_id > MAX_ANTENNA_RELAYS) {
        ESP_LOGE(TAG, "Invalid relay ID: %d", relay_id);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t index = relay_id - 1;
    if (g_antenna_state.antennas[index].state == state) {
        ESP_LOGD(TAG, "Relay %d already %s, skipping command", relay_id, state ? "ON" : "OFF");
        return ESP_OK;
    }

    // Send relay control command via WebSocket
    esp_err_t ret = websocket_client_set_relay(relay_id, state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send relay command via WebSocket: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update local state (will be confirmed by WebSocket event)
    g_antenna_state.antennas[index].state = state;
    
    ESP_LOGI(TAG, "Relay %d (%s) command sent: %s", relay_id, 
             g_antenna_state.antennas[index].name, state ? "ON" : "OFF");

    // Notify UI via LVGL 9 native observer
    radio_subject_set_pointer_async(&radio_antenna_state_subject, &g_antenna_state, sizeof(g_antenna_state));

    return ESP_OK;
}

esp_err_t antenna_get_relay_state(uint8_t relay_id, bool *state)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (relay_id < 1 || relay_id > MAX_ANTENNA_RELAYS || !state) {
        return ESP_ERR_INVALID_ARG;
    }

    *state = g_antenna_state.antennas[relay_id - 1].state;
    return ESP_OK;
}

esp_err_t antenna_select(uint8_t antenna_id)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (antenna_id > MAX_ANTENNA_RELAYS) {
        ESP_LOGE(TAG, "Invalid antenna ID: %d", antenna_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (antenna_id == g_antenna_state.active_antenna) {
        ESP_LOGD(TAG, "Antenna %d already active", antenna_id);
        return ESP_OK;
    }

    if (antenna_id == 0) {
        uint8_t previous = g_antenna_state.active_antenna;
        if (previous >= 1 && previous <= MAX_ANTENNA_RELAYS) {
            esp_err_t ret = antenna_set_relay(previous, false);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to deactivate relay %d when clearing selection", previous);
                return ret;
            }
        }

        g_antenna_state.active_antenna = 0;
        for (int i = 0; i < MAX_ANTENNA_RELAYS; ++i) {
            g_antenna_state.antennas[i].state = false;
        }
        radio_subject_set_pointer_async(&radio_antenna_state_subject, &g_antenna_state, sizeof(g_antenna_state));
        ESP_LOGI(TAG, "No antenna selected (all relays OFF)");
        return ESP_OK;
    }

    esp_err_t ret = websocket_client_select_antenna_default(antenna_id);
    if (ret == ESP_OK) {
        for (int i = 0; i < MAX_ANTENNA_RELAYS; ++i) {
            g_antenna_state.antennas[i].state = ((i + 1) == antenna_id);
        }
        g_antenna_state.active_antenna = antenna_id;
        radio_subject_set_pointer_async(&radio_antenna_state_subject, &g_antenna_state, sizeof(g_antenna_state));
        ESP_LOGI(TAG, "Antenna %d (%s) selected via aggregate command", antenna_id,
                 g_antenna_state.antennas[antenna_id - 1].name);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Aggregate antenna select failed (%s), falling back to individual relays",
             esp_err_to_name(ret));

    esp_err_t fallback_status = ESP_OK;
    for (int i = 0; i < MAX_ANTENNA_RELAYS; ++i) {
        bool desired_state = ((i + 1) == antenna_id);
        if (g_antenna_state.antennas[i].state == desired_state) {
            continue;
        }
        esp_err_t relay_ret = antenna_set_relay(i + 1, desired_state);
        if (fallback_status == ESP_OK && relay_ret != ESP_OK) {
            fallback_status = relay_ret;
        }
    }

    g_antenna_state.active_antenna = antenna_id;
    radio_subject_set_pointer_async(&radio_antenna_state_subject, &g_antenna_state, sizeof(g_antenna_state));

    if (fallback_status == ESP_OK) {
        ESP_LOGI(TAG, "Antenna %d (%s) selected via fallback", antenna_id,
                 g_antenna_state.antennas[antenna_id - 1].name);
    }

    return fallback_status;
}

uint8_t antenna_get_active(void)
{
    return g_antenna_state.active_antenna;
}

esp_err_t antenna_get_info(uint8_t antenna_id, antenna_info_t *info)
{
    if (!g_initialized || !info) {
        return ESP_ERR_INVALID_STATE;
    }

    if (antenna_id < 1 || antenna_id > MAX_ANTENNA_RELAYS) {
        return ESP_ERR_INVALID_ARG;
    }

    *info = g_antenna_state.antennas[antenna_id - 1];
    return ESP_OK;
}

esp_err_t antenna_set_name(uint8_t antenna_id, const char *name)
{
    if (!g_initialized || !name) {
        return ESP_ERR_INVALID_ARG;
    }

    if (antenna_id < 1 || antenna_id > MAX_ANTENNA_RELAYS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t index = antenna_id - 1;
    strncpy(g_antenna_state.antennas[index].name, name, ANTENNA_NAME_MAX_LENGTH - 1);
    g_antenna_state.antennas[index].name[ANTENNA_NAME_MAX_LENGTH - 1] = '\0';

    ESP_LOGI(TAG, "Antenna %d renamed to: %s", antenna_id, name);
    return ESP_OK;
}

esp_err_t antenna_get_system_state(antenna_system_state_t *state)
{
    if (!g_initialized || !state) {
        return ESP_ERR_INVALID_STATE;
    }

    *state = g_antenna_state;
    return ESP_OK;
}

esp_err_t antenna_next(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t next_antenna = g_antenna_state.active_antenna + 1;
    if (next_antenna > MAX_ANTENNA_RELAYS) {
        next_antenna = 1; // Wrap to first antenna
    }

    return antenna_select(next_antenna);
}

esp_err_t antenna_previous(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t prev_antenna = g_antenna_state.active_antenna - 1;
    if (prev_antenna < 1) {
        prev_antenna = MAX_ANTENNA_RELAYS; // Wrap to last antenna
    }

    return antenna_select(prev_antenna);
}

const char* antenna_get_name(uint8_t antenna_id)
{
    if (!g_initialized || antenna_id < 1 || antenna_id > MAX_ANTENNA_RELAYS) {
        return "Invalid";
    }

    return g_antenna_state.antennas[antenna_id - 1].name;
}

// NVS Storage Functions for antenna names caching
// Format: Serialize all 8 antenna names (32 bytes each = 256 bytes total)
// This allows fallback when API is unavailable

esp_err_t antenna_save_names_to_nvs(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_ANTENNA_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for antenna names: %s", esp_err_to_name(err));
        return err;
    }

    // Create a buffer containing all antenna names
    uint8_t names_buffer[MAX_ANTENNA_RELAYS * ANTENNA_NAME_MAX_LENGTH];
    for (int i = 0; i < MAX_ANTENNA_RELAYS; i++) {
        memcpy(&names_buffer[i * ANTENNA_NAME_MAX_LENGTH],
               g_antenna_state.antennas[i].name,
               ANTENNA_NAME_MAX_LENGTH);
    }

    err = nvs_set_blob(nvs_handle, NVS_ANTENNA_NAMES_KEY, names_buffer, sizeof(names_buffer));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write antenna names to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Antenna names saved to NVS successfully");
    } else {
        ESP_LOGW(TAG, "Failed to commit antenna names to NVS: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t antenna_load_names_from_nvs(void)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_ANTENNA_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "NVS antenna namespace not found (first boot?): %s", esp_err_to_name(err));
        return err;
    }

    uint8_t names_buffer[MAX_ANTENNA_RELAYS * ANTENNA_NAME_MAX_LENGTH];
    size_t required_size = sizeof(names_buffer);

    err = nvs_get_blob(nvs_handle, NVS_ANTENNA_NAMES_KEY, names_buffer, &required_size);
    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        // Successfully loaded from NVS, restore antenna names
        for (int i = 0; i < MAX_ANTENNA_RELAYS; i++) {
            memcpy(g_antenna_state.antennas[i].name,
                   &names_buffer[i * ANTENNA_NAME_MAX_LENGTH],
                   ANTENNA_NAME_MAX_LENGTH);
            // Ensure null termination
            g_antenna_state.antennas[i].name[ANTENNA_NAME_MAX_LENGTH - 1] = '\0';
        }
        ESP_LOGI(TAG, "Antenna names restored from NVS cache");
        return ESP_OK;
    }

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "Antenna names not found in NVS (first boot?)");
        return ESP_ERR_NVS_NOT_FOUND;
    }

    ESP_LOGW(TAG, "Failed to read antenna names from NVS: %s", esp_err_to_name(err));
    return err;
}

// Functions for receiving state updates from WebSocket server
// These update local state and emit subject notifications without sending commands

esp_err_t antenna_update_relay_from_server(uint8_t relay_id, bool state)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (relay_id < 1 || relay_id > MAX_ANTENNA_RELAYS) {
        ESP_LOGE(TAG, "Invalid relay ID from server: %d", relay_id);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t index = relay_id - 1;
    if (g_antenna_state.antennas[index].state == state) {
        return ESP_OK; // No change
    }

    g_antenna_state.antennas[index].state = state;

    // Update active_antenna based on relay states
    if (state) {
        g_antenna_state.active_antenna = relay_id;
    } else if (g_antenna_state.active_antenna == relay_id) {
        g_antenna_state.active_antenna = 0;
    }

    ESP_LOGD(TAG, "Relay %d state updated from server: %s", relay_id, state ? "ON" : "OFF");

    // Notify UI via observer
    radio_subject_set_pointer_async(&radio_antenna_state_subject, &g_antenna_state, sizeof(g_antenna_state));

    return ESP_OK;
}

esp_err_t antenna_update_status_from_server(uint8_t current_antenna, const uint8_t *available, int count)
{
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update active antenna
    if (current_antenna <= MAX_ANTENNA_RELAYS) {
        g_antenna_state.active_antenna = current_antenna;
    }

    // Update relay states based on active antenna
    for (int i = 0; i < MAX_ANTENNA_RELAYS; i++) {
        g_antenna_state.antennas[i].state = ((i + 1) == current_antenna);
    }

    // Update availability based on available array
    if (available && count > 0) {
        // First mark all as unavailable
        for (int i = 0; i < MAX_ANTENNA_RELAYS; i++) {
            g_antenna_state.antennas[i].available = false;
        }
        // Then mark available ones
        for (int i = 0; i < count && i < MAX_ANTENNA_RELAYS; i++) {
            uint8_t id = available[i];
            if (id >= 1 && id <= MAX_ANTENNA_RELAYS) {
                g_antenna_state.antennas[id - 1].available = true;
            }
        }
    }

    ESP_LOGD(TAG, "Status updated from server: active=%d, available_count=%d", current_antenna, count);

    // Notify UI via observer
    radio_subject_set_pointer_async(&radio_antenna_state_subject, &g_antenna_state, sizeof(g_antenna_state));

    return ESP_OK;
}
