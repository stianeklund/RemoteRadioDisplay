#ifndef CAT_POLLING_H
#define CAT_POLLING_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// AI mode values
typedef enum {
    AI_MODE_UNKNOWN = -1,
    AI_MODE_OFF = 0,     // AI0 - No auto info, need polling
    AI_MODE_ON = 2,      // AI2 - Auto info on (no backup)  
    AI_MODE_ON_BACKUP = 4 // AI4 - Auto info on (with backup)
} cat_ai_mode_t;

// VFO selection values
typedef enum {
    VFO_A = 0,
    VFO_B = 1,
    VFO_MEM = 2,
    VFO_UNKNOWN = -1
} cat_vfo_t;

// Polling manager state
typedef struct {
    bool polling_enabled;           // Manual polling enable/disable
    bool user_polling_override;     // User override for polling
    cat_ai_mode_t ai_mode;         // Current AI mode
    cat_ai_mode_t expected_ai_mode; // Expected AI mode for persistence monitoring
    cat_vfo_t rx_vfo;              // Current RX VFO (from FR command)
    cat_vfo_t tx_vfo;              // Current TX VFO (from FT command)
    bool cat_connection_active;    // CAT connection is active
    uint32_t last_cat_activity;   // Timestamp of last CAT activity
} cat_polling_state_t;

// Function declarations

/**
 * Initialize the CAT polling manager
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t cat_polling_init(void);

/**
 * Start CAT polling (called after first CAT response received)
 * @return ESP_OK on success, ESP_FAIL on error  
 */
esp_err_t cat_polling_start(void);

/**
 * Stop CAT polling
 */
void cat_polling_stop(void);

/**
 * Enable or disable manual polling override
 * @param enabled true to enable polling, false to disable
 */
void cat_polling_set_user_override(bool enabled);

/**
 * Get current polling enabled state
 * @return true if polling is active, false otherwise
 */
bool cat_polling_is_enabled(void);

/**
 * Get current user override state
 * @return true if user override is enabled, false otherwise
 */
bool cat_polling_get_user_override(void);

/**
 * Set AI mode and adjust polling accordingly
 * @param mode AI mode to set on radio (AI_MODE_ON or AI_MODE_ON_BACKUP)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t cat_polling_set_ai_mode(cat_ai_mode_t mode);

/**
 * Get current AI mode
 * @return Current AI mode
 */
cat_ai_mode_t cat_polling_get_ai_mode(void);

/**
 * Update AI mode from parsed AI command response
 * @param mode Parsed AI mode value
 */
void cat_polling_update_ai_mode(cat_ai_mode_t mode);

/**
 * Initialize expected AI mode from settings (for monitoring)
 * @param mode Expected AI mode from saved settings
 */
void cat_polling_set_expected_ai_mode(cat_ai_mode_t mode);

/**
 * Update RX VFO from FR command response
 * @param vfo RX VFO selection
 */
void cat_polling_update_rx_vfo(cat_vfo_t vfo);

/**
 * Update TX VFO from FT command response  
 * @param vfo TX VFO selection
 */
void cat_polling_update_tx_vfo(cat_vfo_t vfo);

/**
 * Get current RX VFO
 * @return Current RX VFO
 */
cat_vfo_t cat_polling_get_rx_vfo(void);

/**
 * Get current TX VFO
 * @return Current TX VFO  
 */
cat_vfo_t cat_polling_get_tx_vfo(void);


/**
 * Mark CAT connection as active (called on successful CAT response)
 */
void cat_polling_mark_activity(void);

/**
 * Check if CAT connection is active
 * @return true if CAT connection is active, false otherwise
 */
bool cat_polling_is_cat_active(void);

/**
 * Get timestamp of last CAT activity (for screensaver/inactivity detection)
 * @return Timestamp in milliseconds (lv_tick_get() format)
 */
uint32_t cat_polling_get_last_activity_time(void);

/**
 * Get polling state for debugging
 * @return Pointer to polling state structure
 */
const cat_polling_state_t* cat_polling_get_state(void);

// Boot/initialization helpers
// Called by parser when a PS answer is received. If ps_on=true and not yet booted,
// sends a short boot sequence of CAT queries to populate internal state.
void cat_polling_handle_ps_status(bool ps_on);
void cat_polling_handle_agc_response(void);

/**
 * Request memory channel data via MR command
 * @param channel Memory channel number (0-999)
 */
void cat_polling_request_memory_channel(uint16_t channel);

/**
 * Get the last requested memory channel
 * @return Last requested memory channel number
 */
uint16_t cat_polling_get_last_memory_channel(void);

#endif // CAT_POLLING_H
