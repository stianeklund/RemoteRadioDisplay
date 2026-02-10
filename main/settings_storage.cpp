#include "settings_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "SETTINGS";
static nvs_handle_t settings_nvs_handle = 0;

esp_err_t settings_storage_init(void) {
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &settings_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Settings storage initialized");
    return ESP_OK;
}

esp_err_t settings_load(user_settings_t *settings) {
    if (!settings) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (settings_nvs_handle == 0) {
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err;
    uint8_t value_u8;
    int32_t value_i32;
    size_t required_size;
    
    // Load XVTR offset mix setting
    required_size = sizeof(value_u8);
    err = nvs_get_blob(settings_nvs_handle, KEY_XVTR_OFFSET_MIX, &value_u8, &required_size);
    if (err == ESP_OK) {
        settings->xvtr_offset_mix_enabled = (bool)value_u8;
        ESP_LOGD(TAG, "Loaded XVTR offset mix: %s", settings->xvtr_offset_mix_enabled ? "ON" : "OFF");
    } else {
        settings->xvtr_offset_mix_enabled = DEFAULT_XVTR_OFFSET_MIX;
        ESP_LOGD(TAG, "XVTR offset mix not found, using default: %s", settings->xvtr_offset_mix_enabled ? "ON" : "OFF");
    }
    
    // Load CAT polling setting
    required_size = sizeof(value_u8);
    err = nvs_get_blob(settings_nvs_handle, KEY_CAT_POLLING, &value_u8, &required_size);
    if (err == ESP_OK) {
        settings->cat_polling_enabled = (bool)value_u8;
        ESP_LOGD(TAG, "Loaded CAT polling: %s", settings->cat_polling_enabled ? "ON" : "OFF");
    } else {
        settings->cat_polling_enabled = DEFAULT_CAT_POLLING;
        ESP_LOGD(TAG, "CAT polling not found, using default: %s", settings->cat_polling_enabled ? "ON" : "OFF");
    }
    
    // Load AI mode setting
    required_size = sizeof(value_i32);
    err = nvs_get_blob(settings_nvs_handle, KEY_AI_MODE, &value_i32, &required_size);
    if (err == ESP_OK && (value_i32 >= AI_MODE_UNKNOWN && value_i32 <= AI_MODE_ON_BACKUP)) {
        settings->ai_mode = (cat_ai_mode_t)value_i32;
        ESP_LOGD(TAG, "Loaded AI mode: %d", settings->ai_mode);
    } else {
        settings->ai_mode = DEFAULT_AI_MODE;
        ESP_LOGD(TAG, "AI mode not found or invalid, using default: %d", settings->ai_mode);
    }
    
    // Load peak hold setting
    required_size = sizeof(value_u8);
    err = nvs_get_blob(settings_nvs_handle, KEY_PEAK_HOLD, &value_u8, &required_size);
    if (err == ESP_OK) {
        settings->peak_hold_enabled = (bool)value_u8;
        ESP_LOGD(TAG, "Loaded peak hold: %s", settings->peak_hold_enabled ? "ON" : "OFF");
    } else {
        settings->peak_hold_enabled = DEFAULT_PEAK_HOLD;
        ESP_LOGD(TAG, "Peak hold not found, using default: %s", settings->peak_hold_enabled ? "ON" : "OFF");
    }
    
    // Load peak hold duration setting
    uint32_t value_u32;
    required_size = sizeof(value_u32);
    err = nvs_get_blob(settings_nvs_handle, KEY_PEAK_HOLD_DURATION, &value_u32, &required_size);
    if (err == ESP_OK && value_u32 >= 10 && value_u32 <= 1000) {
        settings->peak_hold_duration_ms = value_u32;
        ESP_LOGD(TAG, "Loaded peak hold duration: %lu ms", settings->peak_hold_duration_ms);
    } else {
        settings->peak_hold_duration_ms = DEFAULT_PEAK_HOLD_DURATION;
        ESP_LOGD(TAG, "Peak hold duration not found or invalid, using default: %d ms", DEFAULT_PEAK_HOLD_DURATION);
    }
    
    // Load PEP enabled setting
    required_size = sizeof(value_u8);
    err = nvs_get_blob(settings_nvs_handle, KEY_PEP_ENABLED, &value_u8, &required_size);
    if (err == ESP_OK) {
        settings->pep_enabled = (bool)value_u8;
        ESP_LOGD(TAG, "Loaded PEP enabled: %s", settings->pep_enabled ? "ON" : "OFF");
    } else {
        settings->pep_enabled = DEFAULT_PEP_ENABLED;
        ESP_LOGD(TAG, "PEP enabled not found, using default: %s", settings->pep_enabled ? "ON" : "OFF");
    }
    
    // Load S-meter averaging setting
    required_size = sizeof(value_u8);
    err = nvs_get_blob(settings_nvs_handle, KEY_SMETER_AVERAGING, &value_u8, &required_size);
    if (err == ESP_OK) {
        settings->smeter_averaging_enabled = (bool)value_u8;
        ESP_LOGD(TAG, "Loaded S-meter averaging: %s", settings->smeter_averaging_enabled ? "ON" : "OFF");
    } else {
        settings->smeter_averaging_enabled = DEFAULT_SMETER_AVERAGING;
        ESP_LOGD(TAG, "S-meter averaging not found, using default: %s", settings->smeter_averaging_enabled ? "ON" : "OFF");
    }

    // Load antenna switch setting
    required_size = sizeof(value_u8);
    err = nvs_get_blob(settings_nvs_handle, KEY_ANTENNA_SWITCH, &value_u8, &required_size);
    if (err == ESP_OK) {
        settings->antenna_switch_enabled = (bool)value_u8;
        ESP_LOGD(TAG, "Loaded antenna switch: %s", settings->antenna_switch_enabled ? "ON" : "OFF");
    } else {
        settings->antenna_switch_enabled = DEFAULT_ANTENNA_SWITCH;
        ESP_LOGD(TAG, "Antenna switch not found, using default: %s", settings->antenna_switch_enabled ? "ON" : "OFF");
    }

    ESP_LOGI(TAG, "Settings loaded: XVTR=%s, CAT=%s, AI=%d, Peak=%s, Duration=%lums, PEP=%s, SMeter_Avg=%s, AntSwitch=%s",
             settings->xvtr_offset_mix_enabled ? "ON" : "OFF",
             settings->cat_polling_enabled ? "ON" : "OFF",
             settings->ai_mode,
             settings->peak_hold_enabled ? "ON" : "OFF",
             settings->peak_hold_duration_ms,
             settings->pep_enabled ? "ON" : "OFF",
             settings->smeter_averaging_enabled ? "ON" : "OFF",
             settings->antenna_switch_enabled ? "ON" : "OFF");
    
    return ESP_OK;
}

esp_err_t settings_save(const user_settings_t *settings) {
    if (!settings) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (settings_nvs_handle == 0) {
        ESP_LOGE(TAG, "NVS handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err;
    uint8_t value_u8;
    int32_t value_i32;
    
    // Save XVTR offset mix
    value_u8 = (uint8_t)settings->xvtr_offset_mix_enabled;
    err = nvs_set_blob(settings_nvs_handle, KEY_XVTR_OFFSET_MIX, &value_u8, sizeof(value_u8));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save XVTR offset mix: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save CAT polling
    value_u8 = (uint8_t)settings->cat_polling_enabled;
    err = nvs_set_blob(settings_nvs_handle, KEY_CAT_POLLING, &value_u8, sizeof(value_u8));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save CAT polling: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save AI mode
    value_i32 = (int32_t)settings->ai_mode;
    err = nvs_set_blob(settings_nvs_handle, KEY_AI_MODE, &value_i32, sizeof(value_i32));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save AI mode: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save peak hold
    value_u8 = (uint8_t)settings->peak_hold_enabled;
    err = nvs_set_blob(settings_nvs_handle, KEY_PEAK_HOLD, &value_u8, sizeof(value_u8));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save peak hold: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save peak hold duration
    uint32_t value_u32 = settings->peak_hold_duration_ms;
    err = nvs_set_blob(settings_nvs_handle, KEY_PEAK_HOLD_DURATION, &value_u32, sizeof(value_u32));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save peak hold duration: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save PEP enabled
    value_u8 = (uint8_t)settings->pep_enabled;
    err = nvs_set_blob(settings_nvs_handle, KEY_PEP_ENABLED, &value_u8, sizeof(value_u8));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save PEP enabled: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save S-meter averaging
    value_u8 = (uint8_t)settings->smeter_averaging_enabled;
    err = nvs_set_blob(settings_nvs_handle, KEY_SMETER_AVERAGING, &value_u8, sizeof(value_u8));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save S-meter averaging: %s", esp_err_to_name(err));
        return err;
    }

    // Save antenna switch
    value_u8 = (uint8_t)settings->antenna_switch_enabled;
    err = nvs_set_blob(settings_nvs_handle, KEY_ANTENNA_SWITCH, &value_u8, sizeof(value_u8));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save antenna switch: %s", esp_err_to_name(err));
        return err;
    }

    // Commit changes to flash
    err = nvs_commit(settings_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit settings: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Settings saved: XVTR=%s, CAT=%s, AI=%d, Peak=%s, Duration=%lums, PEP=%s, SMeter_Avg=%s, AntSwitch=%s",
             settings->xvtr_offset_mix_enabled ? "ON" : "OFF",
             settings->cat_polling_enabled ? "ON" : "OFF",
             settings->ai_mode,
             settings->peak_hold_enabled ? "ON" : "OFF",
             settings->peak_hold_duration_ms,
             settings->pep_enabled ? "ON" : "OFF",
             settings->smeter_averaging_enabled ? "ON" : "OFF",
             settings->antenna_switch_enabled ? "ON" : "OFF");
    
    return ESP_OK;
}

esp_err_t settings_save_xvtr_offset_mix(bool enabled) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    uint8_t value = (uint8_t)enabled;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_XVTR_OFFSET_MIX, &value, sizeof(value));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "XVTR offset mix saved: %s", enabled ? "ON" : "OFF");
    }
    return err;
}

esp_err_t settings_save_cat_polling(bool enabled) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    uint8_t value = (uint8_t)enabled;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_CAT_POLLING, &value, sizeof(value));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "CAT polling saved: %s", enabled ? "ON" : "OFF");
    }
    return err;
}

esp_err_t settings_save_ai_mode(cat_ai_mode_t mode) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    int32_t value = (int32_t)mode;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_AI_MODE, &value, sizeof(value));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "AI mode saved: %d", mode);
    }
    return err;
}

esp_err_t settings_save_peak_hold(bool enabled) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    uint8_t value = (uint8_t)enabled;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_PEAK_HOLD, &value, sizeof(value));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "Peak hold saved: %s", enabled ? "ON" : "OFF");
    }
    return err;
}

esp_err_t settings_save_peak_hold_duration(uint32_t duration_ms) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    // Validate range (10ms to 1000ms)
    if (duration_ms < 10 || duration_ms > 1000) {
        ESP_LOGE(TAG, "Invalid peak hold duration: %lu ms (valid range: 10-1000)", duration_ms);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_PEAK_HOLD_DURATION, &duration_ms, sizeof(duration_ms));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "Peak hold duration saved: %lu ms", duration_ms);
    }
    return err;
}

esp_err_t settings_save_pep_enabled(bool enabled) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;
    
    uint8_t value_u8 = (uint8_t)enabled;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_PEP_ENABLED, &value_u8, sizeof(value_u8));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "PEP enabled saved: %s", enabled ? "ON" : "OFF");
    }
    return err;
}

esp_err_t settings_save_smeter_averaging(bool enabled) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;

    uint8_t value_u8 = (uint8_t)enabled;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_SMETER_AVERAGING, &value_u8, sizeof(value_u8));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "S-meter averaging saved: %s", enabled ? "ON" : "OFF");
    }
    return err;
}

esp_err_t settings_save_antenna_switch(bool enabled) {
    if (settings_nvs_handle == 0) return ESP_ERR_INVALID_STATE;

    uint8_t value_u8 = (uint8_t)enabled;
    esp_err_t err = nvs_set_blob(settings_nvs_handle, KEY_ANTENNA_SWITCH, &value_u8, sizeof(value_u8));
    if (err == ESP_OK) {
        err = nvs_commit(settings_nvs_handle);
        ESP_LOGD(TAG, "Antenna switch saved: %s", enabled ? "ON" : "OFF");
    }
    return err;
}

esp_err_t settings_get_current(user_settings_t *settings) {
    if (!settings) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Load from NVS first
    esp_err_t ret = settings_load(settings);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Override with current runtime values from the system
    // This ensures we return the actual current state, not just stored values
    
    // Note: We don't override XVTR and peak hold here since they're UI-only state
    // that should match the stored values. CAT polling and AI mode can be overridden
    // by runtime system state if needed in the future.
    
    return ESP_OK;
}