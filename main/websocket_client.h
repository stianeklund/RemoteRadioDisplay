#pragma once

#include "esp_err.h"
#include "esp_websocket_client.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_CLIENT_STATE_DISCONNECTED,
    WS_CLIENT_STATE_CONNECTING,
    WS_CLIENT_STATE_CONNECTED,
    WS_CLIENT_STATE_ERROR
} ws_client_state_t;

typedef struct {
    uint8_t relay_id;        // 1-8
    bool state;              // true = ON, false = OFF
} antenna_relay_cmd_t;

typedef struct {
    char antenna_name[32];
    bool active;
    bool available;
} antenna_status_t;

esp_err_t websocket_client_init(void);

esp_err_t websocket_client_start(void);

esp_err_t websocket_client_stop(void);

esp_err_t websocket_client_cleanup(void);

ws_client_state_t websocket_client_get_state(void);

esp_err_t websocket_client_set_relay(uint8_t relay_id, bool state);

esp_err_t websocket_client_select_antenna(const char *radio, uint8_t antenna_id);

esp_err_t websocket_client_get_status(void);

esp_err_t websocket_client_get_relay_names(void);

esp_err_t websocket_client_subscribe_events(void);

esp_err_t websocket_client_antenna_switch_next(const char *radio);

esp_err_t websocket_client_antenna_switch_prev(const char *radio);

// Convenience functions that default to radio "A"
esp_err_t websocket_client_select_antenna_default(uint8_t antenna_id);
esp_err_t websocket_client_antenna_switch_next_default(void);
esp_err_t websocket_client_antenna_switch_prev_default(void);

bool websocket_client_is_connected(void);

esp_err_t websocket_client_trigger_reconnection(void);

#ifdef __cplusplus
}
#endif