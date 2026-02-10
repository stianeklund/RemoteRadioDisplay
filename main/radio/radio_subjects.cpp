/**
 * @file radio_subjects.cpp
 * @brief LVGL 9 Subject definitions and initialization for radio state
 *
 * All subjects are statically allocated to avoid heap fragmentation.
 * POINTER-type subjects use static buffers for their payloads.
 */

#include "radio_subjects.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "RADIO_SUBJECTS";

static bool s_initialized = false;

// ============================================================================
// Static Buffers for POINTER Subjects
// ============================================================================

static vfo_update_t s_vfo_buffer = {0};
static kenwood_if_data_t s_if_data_buffer = {0};
static kenwood_xi_data_t s_xi_data_buffer = {0};
static transverter_state_t s_transverter_state_buffer = {0};
static transverter_xo_data_t s_xo_buffer = {0};
static transverter_ex056_data_t s_ex056_buffer = {0};
static antenna_system_state_t s_antenna_state_buffer = {0};
static pep_data_t s_pep_buffer = {0};
static memory_channel_data_t s_memory_channel_buffer = {0};

// ============================================================================
// Subject Definitions - Metering
// ============================================================================

lv_subject_t radio_s_meter_subject;
lv_subject_t radio_swr_subject;
lv_subject_t radio_alc_subject;
lv_subject_t radio_comp_subject;
lv_subject_t radio_power_subject;
lv_subject_t radio_ri_subject;

// ============================================================================
// Subject Definitions - TX State
// ============================================================================

lv_subject_t radio_tx_status_subject;
lv_subject_t radio_split_subject;
lv_subject_t radio_rit_status_subject;
lv_subject_t radio_xit_status_subject;
lv_subject_t radio_rit_freq_subject;
lv_subject_t radio_xit_freq_subject;

// ============================================================================
// Subject Definitions - Radio Configuration
// ============================================================================

lv_subject_t radio_mode_subject;
lv_subject_t radio_data_mode_subject;
lv_subject_t radio_agc_subject;
lv_subject_t radio_preamp_subject;
lv_subject_t radio_att_subject;
lv_subject_t radio_nb_subject;
lv_subject_t radio_nr_subject;
lv_subject_t radio_bc_subject;
lv_subject_t radio_notch_subject;
lv_subject_t radio_notch_freq_subject;
lv_subject_t radio_proc_subject;
lv_subject_t radio_at_status_subject;
lv_subject_t radio_vfo_function_subject;
lv_subject_t radio_af_gain_subject;
lv_subject_t radio_rf_gain_subject;
lv_subject_t radio_ps_status_subject;
lv_subject_t radio_filter_subject;

// ============================================================================
// Subject Definitions - VFO/Frequency
// ============================================================================

lv_subject_t radio_vfo_consolidated_subject;
lv_subject_t radio_freq_a_subject;
lv_subject_t radio_freq_b_subject;

// ============================================================================
// Subject Definitions - IF/XI Data
// ============================================================================

lv_subject_t radio_if_data_subject;
lv_subject_t radio_xi_data_subject;
lv_subject_t radio_memory_channel_subject;

// ============================================================================
// Subject Definitions - Filter
// ============================================================================

lv_subject_t radio_sh_filter_subject;
lv_subject_t radio_sl_filter_subject;
lv_subject_t radio_cw_bandwidth_subject;
lv_subject_t radio_ssb_filter_mode_subject;
lv_subject_t radio_ssb_data_filter_mode_subject;

// ============================================================================
// Subject Definitions - CW Menu
// ============================================================================

lv_subject_t radio_cw_sidetone_volume_subject;
lv_subject_t radio_cw_pitch_hz_subject;
lv_subject_t radio_fm_mic_gain_subject;
lv_subject_t radio_usb_input_level_subject;
lv_subject_t radio_usb_output_level_subject;
lv_subject_t radio_acc2_input_level_subject;
lv_subject_t radio_acc2_output_level_subject;

// ============================================================================
// Subject Definitions - Transverter
// ============================================================================

lv_subject_t radio_transverter_state_subject;
lv_subject_t radio_transverter_xo_subject;
lv_subject_t radio_transverter_ex056_subject;

// ============================================================================
// Subject Definitions - Antenna
// ============================================================================

lv_subject_t radio_antenna_state_subject;
lv_subject_t radio_antenna_select_subject;

// ============================================================================
// Subject Definitions - UI Settings
// ============================================================================

lv_subject_t radio_peak_hold_enabled_subject;
lv_subject_t radio_peak_hold_duration_subject;
lv_subject_t radio_smeter_averaging_subject;
lv_subject_t radio_backlight_subject;
lv_subject_t radio_xvtr_offset_mix_subject;
lv_subject_t radio_transverter_enabled_subject;

// ============================================================================
// Subject Definitions - Notifications
// ============================================================================

lv_subject_t radio_force_refresh_subject;
lv_subject_t radio_antenna_names_subject;
lv_subject_t radio_cat_polling_state_subject;

// ============================================================================
// Subject Definitions - PEP
// ============================================================================

lv_subject_t radio_pep_subject;

// ============================================================================
// Subject Definitions - Time
// ============================================================================

lv_subject_t radio_time_subject;

// ============================================================================
// Subject Definitions - Float (conditional)
// ============================================================================

#if LV_USE_FLOAT
lv_subject_t radio_actual_swr_float_subject;
#endif

// ============================================================================
// Initialization
// ============================================================================

esp_err_t radio_subjects_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Radio subjects already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing radio subjects");

    // Metering subjects (INT, 0-30 range)
    lv_subject_init_int(&radio_s_meter_subject, 0);
    lv_subject_init_int(&radio_swr_subject, 0);
    lv_subject_init_int(&radio_alc_subject, 0);
    lv_subject_init_int(&radio_comp_subject, 0);
    lv_subject_init_int(&radio_power_subject, 0);
    lv_subject_init_int(&radio_ri_subject, 0);

    // TX state subjects (INT as bool)
    lv_subject_init_int(&radio_tx_status_subject, 0);
    lv_subject_init_int(&radio_split_subject, 0);
    lv_subject_init_int(&radio_rit_status_subject, 0);
    lv_subject_init_int(&radio_xit_status_subject, 0);
    lv_subject_init_int(&radio_rit_freq_subject, 0);
    lv_subject_init_int(&radio_xit_freq_subject, 0);

    // Radio configuration subjects
    lv_subject_init_int(&radio_mode_subject, 0);
    lv_subject_init_int(&radio_data_mode_subject, 0);
    lv_subject_init_int(&radio_agc_subject, 0);
    lv_subject_init_int(&radio_preamp_subject, 0);
    lv_subject_init_int(&radio_att_subject, 0);
    lv_subject_init_int(&radio_nb_subject, 0);
    lv_subject_init_int(&radio_nr_subject, 0);
    lv_subject_init_int(&radio_bc_subject, 0);
    lv_subject_init_int(&radio_notch_subject, 0);
    lv_subject_init_int(&radio_notch_freq_subject, 0);
    lv_subject_init_int(&radio_proc_subject, 0);
    lv_subject_init_int(&radio_at_status_subject, 0);
    lv_subject_init_int(&radio_vfo_function_subject, 0);
    lv_subject_init_int(&radio_af_gain_subject, 128);  // Mid-range default
    lv_subject_init_int(&radio_rf_gain_subject, 128);  // Mid-range default
    lv_subject_init_int(&radio_ps_status_subject, 0);
    lv_subject_init_int(&radio_filter_subject, 0);

    // VFO/Frequency subjects
    // Note: uint32_t fits in int32_t for frequencies up to ~2.1 GHz
    lv_subject_init_pointer(&radio_vfo_consolidated_subject, &s_vfo_buffer);
    lv_subject_init_int(&radio_freq_a_subject, 14200000);  // Default 20m
    lv_subject_init_int(&radio_freq_b_subject, 14200000);

    // IF/XI data subjects (POINTER)
    lv_subject_init_pointer(&radio_if_data_subject, &s_if_data_buffer);
    lv_subject_init_pointer(&radio_xi_data_subject, &s_xi_data_buffer);
    lv_subject_init_pointer(&radio_memory_channel_subject, &s_memory_channel_buffer);

    // Filter subjects
    lv_subject_init_int(&radio_sh_filter_subject, 0);
    lv_subject_init_int(&radio_sl_filter_subject, 0);
    lv_subject_init_int(&radio_cw_bandwidth_subject, 500);  // Default 500Hz
    lv_subject_init_int(&radio_ssb_filter_mode_subject, 0);
    lv_subject_init_int(&radio_ssb_data_filter_mode_subject, 0);

    // CW menu subjects (EX006/EX040)
    lv_subject_init_int(&radio_cw_sidetone_volume_subject, 5);  // Mid-level default
    lv_subject_init_int(&radio_cw_pitch_hz_subject, 700);  // Common pitch default
    lv_subject_init_int(&radio_fm_mic_gain_subject, 2);  // Mid-level default
    lv_subject_init_int(&radio_usb_input_level_subject, 5);
    lv_subject_init_int(&radio_usb_output_level_subject, 5);
    lv_subject_init_int(&radio_acc2_input_level_subject, 5);
    lv_subject_init_int(&radio_acc2_output_level_subject, 5);

    // Transverter subjects (POINTER)
    lv_subject_init_pointer(&radio_transverter_state_subject, &s_transverter_state_buffer);
    lv_subject_init_pointer(&radio_transverter_xo_subject, &s_xo_buffer);
    lv_subject_init_pointer(&radio_transverter_ex056_subject, &s_ex056_buffer);

    // Antenna subjects
    lv_subject_init_pointer(&radio_antenna_state_subject, &s_antenna_state_buffer);
    lv_subject_init_int(&radio_antenna_select_subject, 0);

    // UI settings subjects
    lv_subject_init_int(&radio_peak_hold_enabled_subject, 0);
    lv_subject_init_int(&radio_peak_hold_duration_subject, 100);  // 100ms default (matches DEFAULT_PEAK_HOLD_DURATION)
    lv_subject_init_int(&radio_smeter_averaging_subject, 0);
    lv_subject_init_int(&radio_backlight_subject, 255);  // Full brightness
    lv_subject_init_int(&radio_xvtr_offset_mix_subject, 0);  // OFF by default
    lv_subject_init_int(&radio_transverter_enabled_subject, 0);  // OFF by default

    // Notification subjects (counter-based)
    lv_subject_init_int(&radio_force_refresh_subject, 0);
    lv_subject_init_int(&radio_antenna_names_subject, 0);
    lv_subject_init_int(&radio_cat_polling_state_subject, 0);

    // PEP subject (POINTER)
    lv_subject_init_pointer(&radio_pep_subject, &s_pep_buffer);

    // Time subject
    lv_subject_init_int(&radio_time_subject, 0);

#if LV_USE_FLOAT
    // Float subjects
    lv_subject_init_float(&radio_actual_swr_float_subject, 1.0f);
#endif

    s_initialized = true;
    ESP_LOGI(TAG, "Radio subjects initialized successfully");
    return ESP_OK;
}

bool radio_subjects_initialized(void)
{
    return s_initialized;
}

// ============================================================================
// Static Buffer Accessors
// ============================================================================

vfo_update_t* radio_get_vfo_buffer(void)
{
    return &s_vfo_buffer;
}

kenwood_if_data_t* radio_get_if_data_buffer(void)
{
    return &s_if_data_buffer;
}

kenwood_xi_data_t* radio_get_xi_data_buffer(void)
{
    return &s_xi_data_buffer;
}

transverter_state_t* radio_get_transverter_state_buffer(void)
{
    return &s_transverter_state_buffer;
}

antenna_system_state_t* radio_get_antenna_state_buffer(void)
{
    return &s_antenna_state_buffer;
}

pep_data_t* radio_get_pep_buffer(void)
{
    return &s_pep_buffer;
}

memory_channel_data_t* radio_get_memory_channel_buffer(void)
{
    return &s_memory_channel_buffer;
}
