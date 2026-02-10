#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_ANTENNA_RELAYS 8
#define ANTENNA_NAME_MAX_LENGTH 32

typedef struct {
    uint8_t relay_id;        // 1-8
    bool state;              // true = ON, false = OFF
    char name[ANTENNA_NAME_MAX_LENGTH];
    bool available;          // Whether this antenna is configured/available
} antenna_info_t;

typedef struct {
    antenna_info_t antennas[MAX_ANTENNA_RELAYS];
    uint8_t active_antenna;  // Currently selected antenna (1-8, 0 = none)
    bool transmitting;       // Current TX state
} antenna_system_state_t;

esp_err_t antenna_control_init(void);

esp_err_t antenna_set_relay(uint8_t relay_id, bool state);

esp_err_t antenna_get_relay_state(uint8_t relay_id, bool *state);

esp_err_t antenna_select(uint8_t antenna_id);

uint8_t antenna_get_active(void);

esp_err_t antenna_get_info(uint8_t antenna_id, antenna_info_t *info);

esp_err_t antenna_set_name(uint8_t antenna_id, const char *name);

esp_err_t antenna_get_system_state(antenna_system_state_t *state);

esp_err_t antenna_next(void);

esp_err_t antenna_previous(void);

const char* antenna_get_name(uint8_t antenna_id);

// NVS caching functions for antenna names persistence
esp_err_t antenna_save_names_to_nvs(void);

esp_err_t antenna_load_names_from_nvs(void);

// Functions for receiving state updates from WebSocket server
// These update local state and emit subject notifications without sending commands
esp_err_t antenna_update_relay_from_server(uint8_t relay_id, bool state);

esp_err_t antenna_update_status_from_server(uint8_t current_antenna, const uint8_t *available, int count);

#ifdef __cplusplus
}
#endif