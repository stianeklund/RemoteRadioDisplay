#pragma once

#include "esp_err.h"
#include "cat_polling.h"

#ifdef __cplusplus
extern "C" {
#endif

// Settings storage keys
#define NVS_NAMESPACE "rrd_settings"
#define KEY_XVTR_OFFSET_MIX "xvtr_mix"
#define KEY_CAT_POLLING "cat_polling"
#define KEY_AI_MODE "ai_mode"
#define KEY_PEAK_HOLD "peak_hold"
#define KEY_PEAK_HOLD_DURATION "peak_duration"
#define KEY_PEP_ENABLED "pep_enabled"
#define KEY_SMETER_AVERAGING "smeter_avg"
#define KEY_ANTENNA_SWITCH "ant_switch"

// Settings structure for easy access
typedef struct {
    bool xvtr_offset_mix_enabled;
    bool cat_polling_enabled;
    cat_ai_mode_t ai_mode;
    bool peak_hold_enabled;
    uint32_t peak_hold_duration_ms;
    bool pep_enabled;
    bool smeter_averaging_enabled;
    bool antenna_switch_enabled;
} user_settings_t;

// Default settings values
#define DEFAULT_XVTR_OFFSET_MIX false
#define DEFAULT_CAT_POLLING false
#define DEFAULT_AI_MODE AI_MODE_UNKNOWN
#define DEFAULT_PEAK_HOLD false
#define DEFAULT_PEAK_HOLD_DURATION 100
#define DEFAULT_PEP_ENABLED true
#define DEFAULT_SMETER_AVERAGING true
#define DEFAULT_ANTENNA_SWITCH false

/**
 * Initialize settings storage
 * Must be called after nvs_flash_init()
 * @return ESP_OK on success
 */
esp_err_t settings_storage_init(void);

/**
 * Load all settings from NVS
 * @param settings Pointer to settings structure to populate
 * @return ESP_OK on success
 */
esp_err_t settings_load(user_settings_t *settings);

/**
 * Save all settings to NVS
 * @param settings Pointer to settings structure to save
 * @return ESP_OK on success
 */
esp_err_t settings_save(const user_settings_t *settings);

/**
 * Individual setting accessors (for convenience)
 */
esp_err_t settings_save_xvtr_offset_mix(bool enabled);
esp_err_t settings_save_cat_polling(bool enabled);
esp_err_t settings_save_ai_mode(cat_ai_mode_t mode);
esp_err_t settings_save_peak_hold(bool enabled);
esp_err_t settings_save_peak_hold_duration(uint32_t duration_ms);
esp_err_t settings_save_pep_enabled(bool enabled);
esp_err_t settings_save_smeter_averaging(bool enabled);
esp_err_t settings_save_antenna_switch(bool enabled);

/**
 * Get current settings snapshot (combines stored + runtime values)
 * @param settings Pointer to settings structure to populate
 * @return ESP_OK on success
 */
esp_err_t settings_get_current(user_settings_t *settings);

#ifdef __cplusplus
}
#endif