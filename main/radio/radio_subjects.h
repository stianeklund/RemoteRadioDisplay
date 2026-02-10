/**
 * @file radio_subjects.h
 * @brief LVGL 9 Subject declarations for radio state management
 *
 * This module provides statically allocated lv_subject_t instances for all
 * radio parameters. Subjects provide reactive data binding - UI observers
 * are automatically notified when values change.
 *
 * Thread Safety:
 *   Subjects must only be modified from the LVGL task context.
 *   Use radio_subject_updater.h functions for cross-task updates.
 */
#ifndef RADIO_SUBJECTS_H
#define RADIO_SUBJECTS_H

#include "lvgl.h"
#include "cat_parser.h"
#include "cat_shared_types.h"
#include "antenna_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Metering Subjects (high-frequency updates, INT type)
// ============================================================================

/** S-meter reading (0-30 raw value) */
extern lv_subject_t radio_s_meter_subject;

/** SWR meter reading (0-30 raw value) */
extern lv_subject_t radio_swr_subject;

/** ALC meter reading (0-30 raw value) */
extern lv_subject_t radio_alc_subject;

/** COMP (compression) meter reading (0-30 raw value) */
extern lv_subject_t radio_comp_subject;

/** Power meter reading (0-30 raw value) */
extern lv_subject_t radio_power_subject;

/** RI meter reading (0-30 raw value) */
extern lv_subject_t radio_ri_subject;

// ============================================================================
// TX State Subjects (boolean as INT, 0/1)
// ============================================================================

/** Transmit status (0=RX, 1=TX) */
extern lv_subject_t radio_tx_status_subject;

/** Split mode status (0=OFF, 1=ON) */
extern lv_subject_t radio_split_subject;

/** RIT status (0=OFF, 1=ON) */
extern lv_subject_t radio_rit_status_subject;

/** XIT status (0=OFF, 1=ON) */
extern lv_subject_t radio_xit_status_subject;

/** RIT frequency offset (Hz, signed) */
extern lv_subject_t radio_rit_freq_subject;

/** XIT frequency offset (Hz, signed) */
extern lv_subject_t radio_xit_freq_subject;

// ============================================================================
// Radio Configuration Subjects (INT type)
// ============================================================================

/** Operating mode (0-9, maps to mode enum) */
extern lv_subject_t radio_mode_subject;

/** Data mode status (0=OFF, 1=ON) */
extern lv_subject_t radio_data_mode_subject;

/** AGC mode (0=OFF, 1=SLOW, 2=MED, 3=FAST) */
extern lv_subject_t radio_agc_subject;

/** Preamp level (0=OFF, 1=ON) */
extern lv_subject_t radio_preamp_subject;

/** Attenuator level (0=OFF, 1=6dB, 2=12dB, 3=18dB) */
extern lv_subject_t radio_att_subject;

/** NB mode (0=OFF, 1=NB1, 2=NB2) */
extern lv_subject_t radio_nb_subject;

/** NR mode (0=OFF, 1=NR1, 2=NR2) */
extern lv_subject_t radio_nr_subject;

/** Beat cancel status (0=OFF, 1=ON) */
extern lv_subject_t radio_bc_subject;

/** Notch mode (0=OFF, 1=AUTO, 2=MANUAL, 3=MANUAL_WIDE) */
extern lv_subject_t radio_notch_subject;

/** Notch frequency (Hz) */
extern lv_subject_t radio_notch_freq_subject;

/** Processor status (0=OFF, 1=ON) */
extern lv_subject_t radio_proc_subject;

/** Auto tuner status (0=OFF, 1=ON) */
extern lv_subject_t radio_at_status_subject;

/** VFO function (0=VFO A, 1=VFO B, 2=MEM) */
extern lv_subject_t radio_vfo_function_subject;

/** AF gain level (0-255) */
extern lv_subject_t radio_af_gain_subject;

/** RF gain level (0-255) */
extern lv_subject_t radio_rf_gain_subject;

/** Power status (0=OFF, 1=RX, 2=TX) */
extern lv_subject_t radio_ps_status_subject;

/** Filter selection */
extern lv_subject_t radio_filter_subject;

// ============================================================================
// VFO/Frequency Subjects (POINTER to static buffers)
// ============================================================================

/** Consolidated VFO update (pointer to vfo_update_t) */
extern lv_subject_t radio_vfo_consolidated_subject;

/** VFO A frequency (uint32_t direct, fits in INT) */
extern lv_subject_t radio_freq_a_subject;

/** VFO B frequency (uint32_t direct, fits in INT) */
extern lv_subject_t radio_freq_b_subject;

// ============================================================================
// IF/XI Data Subjects (POINTER to static buffers)
// ============================================================================

/** IF command data (pointer to kenwood_if_data_t) */
extern lv_subject_t radio_if_data_subject;

/** XI command data (pointer to kenwood_xi_data_t) */
extern lv_subject_t radio_xi_data_subject;

/** Memory channel data (pointer to memory_channel_data_t) */
extern lv_subject_t radio_memory_channel_subject;

// ============================================================================
// Filter Subjects
// ============================================================================

/** SH (High cut) filter index */
extern lv_subject_t radio_sh_filter_subject;

/** SL (Low cut) filter index */
extern lv_subject_t radio_sl_filter_subject;

/** CW bandwidth (Hz) */
extern lv_subject_t radio_cw_bandwidth_subject;

/** SSB filter mode (0=Hi/Lo, 1=Width/Shift) */
extern lv_subject_t radio_ssb_filter_mode_subject;

/** SSB-DATA filter mode (0=Hi/Lo, 1=Width/Shift) */
extern lv_subject_t radio_ssb_data_filter_mode_subject;

// ============================================================================
// CW Menu Subjects
// ============================================================================

/** CW sidetone volume (0=OFF, 1-9) */
extern lv_subject_t radio_cw_sidetone_volume_subject;

/** CW TX pitch / sidetone frequency (Hz) */
extern lv_subject_t radio_cw_pitch_hz_subject;

/** FM mic gain (1-3) */
extern lv_subject_t radio_fm_mic_gain_subject;

/** USB audio input level (0-9) */
extern lv_subject_t radio_usb_input_level_subject;

/** USB audio output level (0-9) */
extern lv_subject_t radio_usb_output_level_subject;

/** ACC2 AF input level (0-9) */
extern lv_subject_t radio_acc2_input_level_subject;

/** ACC2 AF output level (0-9) */
extern lv_subject_t radio_acc2_output_level_subject;

// ============================================================================
// Transverter Subjects (POINTER to static buffers)
// ============================================================================

/** Transverter state (pointer to transverter_state_t) */
extern lv_subject_t radio_transverter_state_subject;

/** Transverter XO data (pointer to transverter_xo_data_t) */
extern lv_subject_t radio_transverter_xo_subject;

/** Transverter EX056 data (pointer to transverter_ex056_data_t) */
extern lv_subject_t radio_transverter_ex056_subject;

// ============================================================================
// Antenna Subjects (POINTER to static buffers)
// ============================================================================

/** Antenna system state (pointer to antenna_system_state_t) */
extern lv_subject_t radio_antenna_state_subject;

/** Antenna selection (1-8, 0=none) */
extern lv_subject_t radio_antenna_select_subject;

// ============================================================================
// UI Settings Subjects
// ============================================================================

/** Peak hold enabled (0=OFF, 1=ON) */
extern lv_subject_t radio_peak_hold_enabled_subject;

/** Peak hold duration (ms) */
extern lv_subject_t radio_peak_hold_duration_subject;

/** S-meter averaging enabled (0=OFF, 1=ON) */
extern lv_subject_t radio_smeter_averaging_subject;

/** Backlight level (0-255) */
extern lv_subject_t radio_backlight_subject;

/** XVTR Offset Mix toggle (0=OFF, 1=ON) */
extern lv_subject_t radio_xvtr_offset_mix_subject;

/** Transverter enabled (UIXD) toggle (0=OFF, 1=ON) */
extern lv_subject_t radio_transverter_enabled_subject;

// ============================================================================
// Notification Subjects (increment counter to notify)
// ============================================================================

/** Force display refresh (increment to trigger) */
extern lv_subject_t radio_force_refresh_subject;

/** Antenna names updated (increment to trigger) */
extern lv_subject_t radio_antenna_names_subject;

/** CAT polling state changed (increment to trigger) */
extern lv_subject_t radio_cat_polling_state_subject;

// ============================================================================
// PEP (Peak Envelope Power) Subject
// ============================================================================

/** PEP data (pointer to pep_data_t) */
extern lv_subject_t radio_pep_subject;

// ============================================================================
// Time Subject
// ============================================================================

/** Time update (time_t cast to int32_t for low 32 bits) */
extern lv_subject_t radio_time_subject;

// ============================================================================
// Actual SWR Float Subject (requires LV_USE_FLOAT)
// ============================================================================

#if LV_USE_FLOAT
/** Actual SWR as float value */
extern lv_subject_t radio_actual_swr_float_subject;
#endif

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize all radio subjects
 *
 * Must be called from LVGL task context after lv_init() and before
 * any UI code that references subjects.
 *
 * @return ESP_OK on success
 */
esp_err_t radio_subjects_init(void);

/**
 * @brief Check if radio subjects are initialized
 *
 * @return true if initialized, false otherwise
 */
bool radio_subjects_initialized(void);

// ============================================================================
// Static Buffer Accessors (for producers that need to update data in-place)
// ============================================================================

/** Get pointer to VFO update buffer (for in-place modification) */
vfo_update_t* radio_get_vfo_buffer(void);

/** Get pointer to IF data buffer (for in-place modification) */
kenwood_if_data_t* radio_get_if_data_buffer(void);

/** Get pointer to XI data buffer (for in-place modification) */
kenwood_xi_data_t* radio_get_xi_data_buffer(void);

/** Get pointer to transverter state buffer (for in-place modification) */
transverter_state_t* radio_get_transverter_state_buffer(void);

/** Get pointer to antenna state buffer (for in-place modification) */
antenna_system_state_t* radio_get_antenna_state_buffer(void);

/** Get pointer to PEP data buffer (for in-place modification) */
pep_data_t* radio_get_pep_buffer(void);

/** Get pointer to memory channel data buffer (for in-place modification) */
memory_channel_data_t* radio_get_memory_channel_buffer(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_SUBJECTS_H
