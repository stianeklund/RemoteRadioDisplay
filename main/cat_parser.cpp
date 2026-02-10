#include "lvgl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cat_parser.h"
#include "cat_shared_types.h" // Include the shared types
#include "esp_log.h"
#include "esp_heap_caps.h" // For heap integrity checking
#include "esp_task_wdt.h" // For watchdog reset
#include "esp_timer.h" // For esp_timer_get_time
#include "freertos/FreeRTOS.h"
#include "gfx/lcd_init.h" // For backlight enable/disable
#include "gfx/lvgl_init.h" // For LVGL display functions
#include "radio/radio_subjects.h" // For LVGL 9 observer subjects
#include "radio/radio_subject_updater.h" // For async subject updates
#include "ui/screens/ui_Screen1.h" // For ui_screen1_update_filter
#include "ui/components/ui_power_popup.h" // For UI meta command handling
#include <errno.h>
#include <ctype.h> // For isdigit
#include "driver/uart.h" // For uart_write_bytes and UART definitions
#include "uart.h"        // For uart_write_raw()
#include "ui/ui.h" // For ui components
#include "cat_polling.h" // For AI mode updates
#include "cat_state.hpp"  // For radio_set_ssb_filter_mode, etc.
#include "screensaver.h"  // For screensaver_update_backlight

// External UI macro functions (defined in ui_Screen2.cpp)
extern void ui_macro_set_cached(uint8_t id, const char *name, const char *cmd);
extern void ui_macro_set_fkey_assignment(uint8_t fkey, uint8_t macro_id);
extern void ui_macro_refresh_list(void);
extern void ui_macro_request_refresh(void);

#ifndef CAT_PARSER_DEBUG
#define CAT_PARSER_DEBUG 0
#endif

// Frequency update performance instrumentation
// Set to 1 to enable stats logging every 10 seconds
#define FREQ_UPDATE_STATS_ENABLED 0

#if FREQ_UPDATE_STATS_ENABLED
static struct {
    uint32_t if_cmd_count;           // IF commands received
    uint32_t fa_cmd_count;           // FA commands received
    uint32_t fb_cmd_count;           // FB commands received
    uint32_t freq_change_count;      // Actual frequency changes detected
    uint32_t ui_queue_count;         // Updates sent to UI queue
    uint32_t throttled_count;        // Updates throttled (dedup or rate limit)
    uint32_t last_freq_hz;           // Last frequency for change detection
    int64_t last_stats_time_us;      // Last stats report time
    int64_t min_interval_us;         // Min interval between freq changes
    int64_t max_interval_us;         // Max interval between freq changes
    int64_t last_freq_change_time_us; // Time of last frequency change
    int64_t total_interval_us;       // Sum of intervals for averaging
} g_freq_stats = {0};

static const char *STATS_TAG = "FREQ_STATS";
#endif

static const char *TAG = "CAT_PARSER";

static volatile bool g_cat_data_activity_flag = false;

static bool g_is_transmitting = false; // Global variable to store transmit status
static bool g_tfset_active = false;    // True when TF-Set is engaged (emulates TX view in split mode)
static uint8_t g_current_highcut_index = 0; // Default high-cut index
static uint8_t g_current_lowcut_index = 0; // Default low-cut index

// Cache the last raw (radio) VFO frequencies we parsed from FA/FB/IF.
// Used for immediate UI refresh when transverter state or toggle changes.
static uint32_t g_last_raw_vfo_a_hz = 0;
static uint32_t g_last_raw_vfo_b_hz = 0;
static uint32_t g_last_raw_memory_hz = 0;  // Memory channel frequency from IF command

// Track current RX/TX VFO and split state for active VFO display
static int g_current_rx_vfo_function = 0; // 0=VFO A, 1=VFO B, 2=MEM
static int g_current_tx_vfo_function = 0; // 0=VFO A, 1=VFO B, 2=MEM (for split mode)
static bool g_current_split_on = false;
// Track current antenna state for handling AN commands with '9' (no change)
static int g_current_antenna_select = 0; // 0=ANT1, 1=ANT2
static bool g_current_rx_ant_used = false;
static bool g_current_drive_out_on = false;

// Memory channel data storage (MR command)
static memory_channel_data_t g_memory_channel_data = {0};

// Consolidated meter logging - only logs on significant changes or TX/RX transitions
#define METER_LOG_INTERVAL_MS 1000  // Minimum interval between logs
#define METER_LOG_ENABLED 0  // Disabled - set to 1 to enable meter debug logging

#if METER_LOG_ENABLED
static struct {
    // Current values
    int s_meter;      // S-meter (RX) or Power (TX), 0-30
    int alc;          // ALC meter, 0-30
    int swr;          // SWR meter segments, 0-30
    float swr_actual; // Calculated SWR value
    bool is_tx;       // Currently transmitting
    // Previous logged values (for change detection)
    int prev_s_meter;
    int prev_alc;
    int prev_swr;
    bool prev_is_tx;
    int64_t last_log_time_us;
} g_meter_state = {0};

static void meter_log_update(void) {
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_ms = (now_us - g_meter_state.last_log_time_us) / 1000;

    // Check if TX/RX state changed
    bool tx_rx_changed = (g_meter_state.is_tx != g_meter_state.prev_is_tx);

    // Check if values changed significantly
    bool values_changed = false;
    if (g_meter_state.is_tx) {
        // During TX: log if power changes by 3+, or ALC/SWR changes at all
        values_changed = (abs(g_meter_state.s_meter - g_meter_state.prev_s_meter) >= 3) ||
                         (g_meter_state.alc != g_meter_state.prev_alc) ||
                         (g_meter_state.swr != g_meter_state.prev_swr);
    } else {
        // During RX: log if S-meter changes by 2+ segments
        values_changed = (abs(g_meter_state.s_meter - g_meter_state.prev_s_meter) >= 2);
    }

    // Log on TX/RX transition immediately, or on significant change after interval
    if (tx_rx_changed || (values_changed && elapsed_ms >= METER_LOG_INTERVAL_MS)) {
        if (g_meter_state.is_tx) {
            ESP_LOGI("METERS", "TX: PWR=%d ALC=%d SWR=%.1f",
                     g_meter_state.s_meter,
                     g_meter_state.alc,
                     g_meter_state.swr_actual);
        } else {
            ESP_LOGI("METERS", "RX: S=%d", g_meter_state.s_meter);
        }
        // Update previous values
        g_meter_state.prev_s_meter = g_meter_state.s_meter;
        g_meter_state.prev_alc = g_meter_state.alc;
        g_meter_state.prev_swr = g_meter_state.swr;
        g_meter_state.prev_is_tx = g_meter_state.is_tx;
        g_meter_state.last_log_time_us = now_us;
    }
}
#endif

// Forward declarations
static void update_frequency_displays(int rx_vfo_function, int tx_vfo_function, bool split_on);

/* Legacy direct UI update callback declarations removed */

/* Legacy direct sync update declarations removed */

/* Legacy async wrapper declarations removed */

// Global flag to track if ultra-fast mode is initialized
static bool ultra_fast_mode_initialized = false;

// Helper function stubs for filter index management
// TODO: Replace these with actual hardware/DSP control logic
static uint8_t get_current_highcut_index() {
    return g_current_highcut_index;
}

static uint8_t get_current_lowcut_index() {
    return g_current_lowcut_index;
}

static void set_highcut_index(uint8_t idx) {
    // TODO: Implement actual high-cut filter setting
    g_current_highcut_index = idx;
    ESP_LOGI(TAG, "Set high-cut index to: %u", idx);
    radio_subject_set_int_async(&radio_sh_filter_subject, idx);
}

static void set_lowcut_index(uint8_t idx) {
    // TODO: Implement actual low-cut filter setting
    g_current_lowcut_index = idx;
    ESP_LOGI(TAG, "Set low-cut index to: %u", idx);
    radio_subject_set_int_async(&radio_sl_filter_subject, idx);
}

#if FREQ_UPDATE_STATS_ENABLED
// Log frequency update stats every 10 seconds and reset counters
static void freq_stats_log_if_needed(void) {
    int64_t now_us = esp_timer_get_time();

    // Initialize on first call
    if (g_freq_stats.last_stats_time_us == 0) {
        g_freq_stats.last_stats_time_us = now_us;
        g_freq_stats.min_interval_us = INT64_MAX;
        return;
    }

    int64_t elapsed_us = now_us - g_freq_stats.last_stats_time_us;
    if (elapsed_us >= 10000000) { // 10 seconds
        float elapsed_sec = elapsed_us / 1000000.0f;
        float if_rate = g_freq_stats.if_cmd_count / elapsed_sec;
        float fa_rate = g_freq_stats.fa_cmd_count / elapsed_sec;
        float fb_rate = g_freq_stats.fb_cmd_count / elapsed_sec;
        float change_rate = g_freq_stats.freq_change_count / elapsed_sec;
        float queue_rate = g_freq_stats.ui_queue_count / elapsed_sec;

        float avg_interval_ms = 0;
        if (g_freq_stats.freq_change_count > 1) {
            avg_interval_ms = (g_freq_stats.total_interval_us / (g_freq_stats.freq_change_count - 1)) / 1000.0f;
        }

        float min_ms = (g_freq_stats.min_interval_us == INT64_MAX) ? 0 : g_freq_stats.min_interval_us / 1000.0f;
        float max_ms = g_freq_stats.max_interval_us / 1000.0f;

        ESP_LOGI(STATS_TAG, "CAT Cmds: IF:%.1f/s FA:%.1f/s FB:%.1f/s | UI: Queue:%.1f/s Throttled:%lu",
                 if_rate, fa_rate, fb_rate, queue_rate, g_freq_stats.throttled_count);
        ESP_LOGI(STATS_TAG, "Freq Changes: %.1f/s | Interval: avg:%.0fms min:%.0fms max:%.0fms",
                 change_rate, avg_interval_ms, min_ms, max_ms);

        // Reset counters
        g_freq_stats.if_cmd_count = 0;
        g_freq_stats.fa_cmd_count = 0;
        g_freq_stats.fb_cmd_count = 0;
        g_freq_stats.freq_change_count = 0;
        g_freq_stats.ui_queue_count = 0;
        g_freq_stats.throttled_count = 0;
        g_freq_stats.min_interval_us = INT64_MAX;
        g_freq_stats.max_interval_us = 0;
        g_freq_stats.total_interval_us = 0;
        g_freq_stats.last_stats_time_us = now_us;
    }
}

// Track a frequency change event
static void freq_stats_track_change(uint32_t freq_hz) {
    int64_t now_us = esp_timer_get_time();

    if (freq_hz != g_freq_stats.last_freq_hz) {
        g_freq_stats.freq_change_count++;

        if (g_freq_stats.last_freq_change_time_us != 0) {
            int64_t interval = now_us - g_freq_stats.last_freq_change_time_us;
            g_freq_stats.total_interval_us += interval;
            if (interval < g_freq_stats.min_interval_us) g_freq_stats.min_interval_us = interval;
            if (interval > g_freq_stats.max_interval_us) g_freq_stats.max_interval_us = interval;
        }
        g_freq_stats.last_freq_change_time_us = now_us;
        g_freq_stats.last_freq_hz = freq_hz;
    }
}
#endif

// Initialize CAT parser subsystem
esp_err_t cat_parser_init(void) {
    ESP_LOGI(TAG, "Initializing CAT parser");
    ESP_LOGI(TAG, "CAT parser initialized");
    return ESP_OK;
}
// Helper function to convert string digits to integer directly (faster than atoi)
static inline int32_t parse_int(const char *str, int len) {
    int32_t val = 0;
    for (int i = 0; i < len && str[i] >= '0' && str[i] <= '9'; i++) {
        val = val * 10 + (str[i] - '0');
    }
    return val;
}

// Helper function to convert string digits to long long directly (faster than atoll)
static inline int64_t parse_long(const char *str, int len) {
    int64_t val = 0;
    for (int i = 0; i < len && str[i] >= '0' && str[i] <= '9'; i++) {
        val = val * 10 + (str[i] - '0');
    }
    return val;
}

// Calibration table structures and data
typedef struct {
    float raw; // Input value (e.g., raw segment reading, or watts)
    float actual; // Output value (e.g., SWR, or segments)
} cal_point_t;

typedef struct {
    int num_points;
    cal_point_t points[7]; // Max 7 points based on current tables, adjust if larger tables are needed
} cal_table_t;

// Table to convert UI SWR segments (0-30) to actual SWR value (float)
// Based on TS590_SWR_CAL: {0,1.0f}, {6,1.5f}, {12,2.0f}, {15,3.0f}, {30,10.0f}
static const cal_table_t segments_to_swr_cal_table = {
    5, {
        // num_points
        // UI Segments (dots), Actual SWR
        {0.0f, 1.0f},
        {6.0f, 1.5f},
        {12.0f, 2.0f},
        {15.0f, 3.0f}, // Using the 15 dots for 3.0 SWR from the provided calibration
        {30.0f, 10.0f}
    }
};

// Generic linear interpolation function
static float interpolate(float input_value, const cal_table_t *table) {
    if (table == NULL || table->num_points == 0) {
        ESP_LOGE(TAG, "Interpolate: Invalid table or zero points.");
        return input_value; // Or some error/default value
    }

    // Handle out-of-bounds cases: clamp to table limits
    if (input_value <= table->points[0].raw) {
        return table->points[0].actual;
    }
    if (input_value >= table->points[table->num_points - 1].raw) {
        return table->points[table->num_points - 1].actual;
    }

    // Find the two points to interpolate between
    for (int i = 0; i < table->num_points - 1; i++) {
        if (input_value >= table->points[i].raw && input_value <= table->points[i + 1].raw) {
            float raw1 = table->points[i].raw;
            float actual1 = table->points[i].actual;
            float raw2 = table->points[i + 1].raw;
            float actual2 = table->points[i + 1].actual;

            if (raw2 == raw1) {
                // Avoid division by zero if points are identical
                return actual1;
            }

            // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            return actual1 + (input_value - raw1) * (actual2 - actual1) / (raw2 - raw1);
        }
    }
    // Should not be reached if input_value is within table range due to checks above
    ESP_LOGW(TAG, "Interpolate: Value %.2f was out of expected range after initial checks.", input_value);
    return table->points[table->num_points - 1].actual; // Fallback
}

bool cat_get_transmit_status(void) {
    return g_is_transmitting;
}

bool cat_get_split_status(void) {
    // Return authoritative split status from IF/SP commands only.
    // Do NOT derive split from VFO function comparison - the IF command's P12
    // field is the radio's actual split state. VFO function comparison is unreliable
    // during boot sequence when FR/FT responses may arrive out of order.
    return g_current_split_on;
}

int cat_get_rx_vfo_function(void) {
    return g_current_rx_vfo_function;
}

uint32_t parse_fa_frequency(const char *response) {
    uint32_t vfo_frequency = 0;
    // Validate minimum command length (FA + 11 digit frequency)
    size_t len = strlen(response);
    if (len < 13) {
        ESP_LOGW(TAG, "FA command too short: %zu chars", len);
        return 0;
    }

    // Direct conversion without sscanf for better performance
    const char *ptr = response + 2;
    int i = 0;
    while (*ptr >= '0' && *ptr <= '9' && i < 11) {
        vfo_frequency = vfo_frequency * 10 + (*ptr - '0');
        ptr++;
        i++;
    }

    // Validate we parsed exactly 11 digits
    if (i != 11) {
        ESP_LOGW(TAG, "FA frequency invalid length: %d digits", i);
        return 0;
    }

#if FREQ_UPDATE_STATS_ENABLED
    g_freq_stats.fa_cmd_count++;
#endif

    // Cache the last raw VFO-A frequency
    g_last_raw_vfo_a_hz = vfo_frequency;

    // Log FA frequency parsing for debugging
    ESP_LOGV(TAG, "Parsed FA frequency: %lu Hz", vfo_frequency);
    
    // Use active VFO frequency display logic
    update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, cat_get_split_status());
    return vfo_frequency;
}

uint32_t parse_fb_frequency(const char *response) {
    uint32_t freq = 0;

    // Validate minimum command length (FB + 11 digit frequency)
    // Note: Prefix already verified by dispatcher hash
    size_t len = strlen(response);
    if (len < 13) {
        ESP_LOGW(TAG, "FB: Command too short: %zu chars, expected 13+", len);
        return 0;
    }
    
    // Parse exactly 11 digits starting at position 2
    const char *ptr = response + 2;
    for (int i = 0; i < 11; i++) {
        if (ptr[i] < '0' || ptr[i] > '9') {
            ESP_LOGW(TAG, "FB: Invalid digit at position %d: '%c'", i + 2, ptr[i]);
            return 0;
        }
        freq = freq * 10 + (ptr[i] - '0');
    }

    // Cache last raw VFO-B
    g_last_raw_vfo_b_hz = freq;

#if FREQ_UPDATE_STATS_ENABLED
    g_freq_stats.fb_cmd_count++;
#endif

    ESP_LOGV(TAG, "Parsed FB frequency: %lu", freq);

    // Use active VFO frequency display logic
    update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, cat_get_split_status());

    return freq;
}

int parse_mode(const char *response) {
    int mode = -1;
    if (response[2] != '\0' && response[2] >= '0' && response[2] <= '9') {
        mode = response[2] - '0';
        ESP_LOGD(TAG, "Parsed MD command: '%s' -> mode: %d", response, mode);
        radio_subject_set_int_async(&radio_mode_subject, mode);
    }
    return mode;
}

int parse_ps_status(const char *response) {
    // Minimum length check: PS + 1 digit (prefix verified by dispatcher)
    if (response[2] == '\0') {
        ESP_LOGW(TAG, "PS command too short: %s", response);
        return -1;
    }
    {
        char p1_char = response[2];
        bool ps_status = false; // Default to OFF

        if (p1_char == '0' || p1_char == '9') {
            ps_status = false;
        } else if (p1_char == '1') {
            ps_status = true;
        } else {
            ESP_LOGW(TAG, "Invalid PS status character: %c", p1_char);
            return -1; // Indicate error
        }

        ESP_LOGV(TAG, "Parsed PS status: %s", ps_status ? "ON" : "OFF");
        radio_subject_set_int_async(&radio_ps_status_subject, ps_status ? 1 : 0);
        // Notify polling manager to start boot sequence when PS1 observed
        cat_polling_handle_ps_status(ps_status);
        return ps_status ? 1 : 0; // Return 1 for ON, 0 for OFF
    }
}


int parse_filter(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return -1;
    int filter = response[2] - '0';
    ESP_LOGV(TAG, "Parsed filter: %d", filter);
    radio_subject_set_int_async(&radio_filter_subject, filter);

    // Update UI filter labels directly
    ui_screen1_update_filter(filter);
    return filter;
}

bool parse_rit_status(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return false;
    bool rit_status = (response[2] == '1');
    ESP_LOGV(TAG, "Parsed RIT status: %d", rit_status);
    radio_subject_set_int_async(&radio_rit_status_subject, rit_status ? 1 : 0);
    return rit_status;
}

int parse_rit_frequency(const char *response) {
    // RU + 5 digits = 7 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[6] == '\0') return 0;
    int rit_freq = parse_int(response + 2, 5);
    ESP_LOGV(TAG, "Parsed RIT frequency: %d", rit_freq);
    radio_subject_set_int_async(&radio_rit_freq_subject, rit_freq);
    return rit_freq;
}

bool parse_xit_status(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return false;
    bool xit_status = (response[2] == '1');
    ESP_LOGV(TAG, "Parsed XIT status: %d", xit_status);
    radio_subject_set_int_async(&radio_xit_status_subject, xit_status ? 1 : 0);
    return xit_status;
}

int parse_xit_frequency(const char *response) {
    // XU + 5 digits = 7 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[6] == '\0') return 0;
    int xit_freq = parse_int(response + 2, 5);
    ESP_LOGV(TAG, "Parsed XIT frequency: %d", xit_freq);
    radio_subject_set_int_async(&radio_xit_freq_subject, xit_freq);
    return xit_freq;
}

bool parse_split_operation_status(const char *response) {
    // SP command: Split operation frequency setting status (NOT split mode ON/OFF!)
    // Per TS-590SG spec: P1 = 0: No operation/Setting complete, 1: Setting in progress, 2: Cancel
    // This is about the split frequency ADJUSTMENT operation, not split mode itself.
    // Split mode ON/OFF comes exclusively from IF command P12 field.
    if (response[2] == '\0') return false;

    char p1 = response[2];
    bool operation_in_progress = (p1 == '1');

    ESP_LOGD(TAG, "SP command: split frequency operation %s",
             p1 == '0' ? "complete/idle" : (p1 == '1' ? "in progress" : "cancelled"));

    // Do NOT update split mode status here - IF command P12 is the authoritative source
    return operation_in_progress;
}


uint32_t parse_tx_frequency(const char *response) {
    // FB + 11 digits = 13 chars minimum; prefix verified by dispatcher
    size_t len = strlen(response);
    if (len < 13) return 0;

    uint32_t tx_freq = (uint32_t) parse_long(response + 2, 11);
    // Cache last raw VFO-B
    g_last_raw_vfo_b_hz = tx_freq;
    ESP_LOGV(TAG, "Parsed TX frequency: %" PRIu32, tx_freq);
    // Keep TX message as raw radio-side frequency
    // For display, apply transverter offset if enabled
    uint64_t display_frequency = apply_transverter_offset_for_display((uint64_t) tx_freq);
    uint32_t display_freq_u32 = (uint32_t) display_frequency;
    return tx_freq;
}

int parse_agc_mode(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return -1;
    char p1_char = response[2];
    if (p1_char >= '0' && p1_char <= '3') {
        int agc_mode = p1_char - '0';
        ESP_LOGV(TAG, "Parsed AGC mode: %d", agc_mode);
        radio_subject_set_int_async(&radio_agc_subject, agc_mode);
        cat_polling_handle_agc_response();
        return agc_mode;
    }
    ESP_LOGW(TAG, "Invalid AGC mode character: %c", p1_char);
    return -1;
}

int parse_agc_time(const char *response) {
    // GT + 2 digits = 4 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[3] == '\0') return -1;
    int agc_time = parse_int(response + 2, 2);
    ESP_LOGV(TAG, "Parsed AGC time constant: %d", agc_time);
    return agc_time;
}

int parse_power(const char *response) {
    // PC + 3 digits = 5 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[4] == '\0') return -1;
    int power = parse_int(response + 2, 3);
    ESP_LOGV(TAG, "Parsed Power: %d", power);
    return power;
}

int parse_preamp(const char *response) {
    // Command format is PAX0; e.g., PA00 (off), PA10 (on)
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') {
        ESP_LOGW(TAG, "Preamp command too short: %s", response);
        return -1;
    }

    char p1_char = response[2]; // Preamp enable/disable
    int preamp;

    // Validate P1 parameter
    if (p1_char == '0') {
        preamp = 0; // Preamp OFF
    } else if (p1_char == '1') {
        preamp = 1; // Preamp ON
    } else {
        ESP_LOGW(TAG, "Invalid Preamp status character: %c", p1_char);
        return -1;
    }

    // If answer format (P2 exists), validate P2 parameter
    if (response[3] != '\0' && response[3] != ';' && response[3] != '0') {
        ESP_LOGW(TAG, "PA answer format P2 should be '0', got '%c' in: %s", response[3], response);
        // Don't fail completely, just warn - the P1 value is still valid
    }

    ESP_LOGV(TAG, "Parsed Preamp: %d", preamp);
    radio_subject_set_int_async(&radio_preamp_subject, preamp);
    return preamp;
}

int parse_processor(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') {
        ESP_LOGW(TAG, "Processor command too short: %s", response);
        return -1;
    }

    char p1_char = response[2];
    bool proc_status;
    if (p1_char == '1') {
        proc_status = true;
    } else if (p1_char == '0') {
        proc_status = false;
    } else {
        ESP_LOGW(TAG, "Invalid Processor status character: %c", p1_char);
        return -1;
    }

    ESP_LOGV(TAG, "Parsed Processor status: %s", proc_status ? "ON" : "OFF");
    radio_subject_set_int_async(&radio_proc_subject, proc_status ? 1 : 0);
    return proc_status ? 1 : 0;
}

int parse_att(const char *response) {
    // Prefix verified by dispatcher; check length for format detection
    size_t len = strlen(response);

    // Handle both RA command formats:
    // SET format: "RA01" or "RA00" (4 chars)
    // ANSWER format: "RA0100" or "RA0000" (6 chars)
    if (len != 4 && len != 6) {
        ESP_LOGW(TAG, "Invalid RA command length. Expected 4 or 6 chars, got %zu: %s", len, response);
        return -1;
    }

    int att_status;
    // P1 is at response[2] and response[3]
    if (response[2] == '0' && response[3] == '1') {
        att_status = 1; // ATT ON
    } else if (response[2] == '0' && response[3] == '0') {
        att_status = 0; // ATT OFF
    } else {
        ESP_LOGW(TAG, "Invalid ATT P1 parameter: %c%c", response[2], response[3]);
        return -1;
    }

    // For ANSWER format (6 chars), validate P2 should be "00"
    if (len == 6) {
        if (!(response[4] == '0' && response[5] == '0')) {
            ESP_LOGW(TAG, "Invalid ATT P2 parameter: %c%c. Expected '00'. Command: %s",
                     response[4], response[5], response);
            return -1;
        }
    }

    ESP_LOGV(TAG, "Parsed ATT status: %d (format: %s)", att_status, len == 4 ? "SET" : "ANSWER");
    radio_subject_set_int_async(&radio_att_subject, att_status);
    return att_status;
}

bool parse_nb_status(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') {
        ESP_LOGW(TAG, "Invalid NB command format: \"%s\"", response);
        return false;
    }
    int nb_mode = 0;
    char mode_char = response[2];
    if (mode_char >= '0' && mode_char <= '3') {
        nb_mode = mode_char - '0'; // 0=OFF, 1=NB1, 2=NB2, 3=NB3
    }
    ESP_LOGI(TAG, "NB command received: \"%s\" -> mode=%d", response, nb_mode);
    radio_subject_set_int_async(&radio_nb_subject, nb_mode);
    return nb_mode > 0;
}

int parse_nr_status(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return 0;
    int nr_mode = 0;
    char mode_char = response[2];
    if (mode_char >= '0' && mode_char <= '2') {
        nr_mode = mode_char - '0'; // 0=OFF, 1=NR1, 2=NR2
    }
    ESP_LOGV(TAG, "Parsed NR mode: %d", nr_mode);
    radio_subject_set_int_async(&radio_nr_subject, nr_mode);
    return nr_mode;
}

bool parse_bc_status(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return false;
    bool bc_status = (response[2] == '1');
    ESP_LOGV(TAG, "Parsed BC status: %d", bc_status);
    radio_subject_set_int_async(&radio_bc_subject, bc_status ? 1 : 0);
    return bc_status;
}

notch_mode_t parse_notch_status(const char *response) {
    // NT + P1 + P2 = 4 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[3] == '\0') return NOTCH_OFF;

    // NT command format: NTP1P2; where P1=mode (0/1/2), P2=bandwidth (0/1)
    char p1 = response[2];
    char p2 = response[3];
    notch_mode_t notch_mode;

    if (p1 == '0') {
        notch_mode = NOTCH_OFF;
    } else if (p1 == '1') {
        notch_mode = NOTCH_AUTO;
    } else if (p1 == '2') {
        notch_mode = (p2 == '1') ? NOTCH_MANUAL_WIDE : NOTCH_MANUAL_NORMAL;
    } else {
        return NOTCH_OFF;
    }

    ESP_LOGV(TAG, "Parsed Notch mode: %d (P1=%c, P2=%c)", notch_mode, p1, p2);
    radio_subject_set_int_async(&radio_notch_subject, (int)notch_mode);
    return notch_mode;
}

int parse_notch_frequency(const char *response) {
    // BP + 3 digits = 5 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[4] == '\0') return -1;
    int notch_freq = parse_int(response + 2, 3);
    ESP_LOGV(TAG, "Parsed Notch frequency: %d", notch_freq);
    radio_subject_set_int_async(&radio_notch_freq_subject, notch_freq);
    return notch_freq;
}

int parse_cw_speed(const char *response) {
    // KS + 3 digits = 5 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[4] == '\0') return -1;
    int cw_speed = parse_int(response + 2, 3);
    ESP_LOGV(TAG, "Parsed CW Speed: %d", cw_speed);
    return cw_speed;
}

bool parse_vox_status(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return false;
    bool vox_status = (response[2] == '1');
    ESP_LOGV(TAG, "Parsed VOX status: %d", vox_status);
    return vox_status;
}

int parse_vox_gain(const char *response) {
    // VG + 3 digits = 5 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0' || response[4] == '\0') return -1;
    int vox_gain = parse_int(response + 2, 3);
    ESP_LOGV(TAG, "Parsed VOX Gain: %d", vox_gain);
    return vox_gain;
}

void parse_data_command(const char *response) {
    // Prefix verified by dispatcher; need at least DA + 1 digit
    if (response[2] == '\0') return;
    bool data_mode = (response[2] == '1');
    radio_subject_set_int_async(&radio_data_mode_subject, data_mode ? 1 : 0);
}

#define IF_COMMAND_LENGTH 37 // "IF" + 35 payload characters for TS-590SG (P1-P15)
#define FREQ_STR_LENGTH 11
#define RIT_XIT_FREQ_STR_LENGTH 5

static const char *IF_TAG = "IF_COMMAND_PARSER";

esp_err_t parse_frequency(const char *freq_str, uint32_t *frequency) {
    if (strlen(freq_str) != FREQ_STR_LENGTH) {
        ESP_LOGE(IF_TAG, "Invalid frequency string length");
        return ESP_ERR_INVALID_ARG;
    }

    *frequency = strtoul(freq_str, NULL, 10);
    if (*frequency == 0 && errno == EINVAL) {
        ESP_LOGE(TAG, "Invalid frequency format");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


// kenwood_if_data_t struct is now defined in cat_parser.h

// Helper function to update frequency displays based on active VFO logic
// Top display: Always shows currently active VFO frequency
// Bottom display: Always shows non-active VFO frequency
// Active VFO: RX VFO when receiving, TX VFO when transmitting (in split mode)
static void update_frequency_displays(int rx_vfo_function, int tx_vfo_function, bool split_on) {
    // Deduplication state - skip updates when nothing has changed
    static uint32_t s_last_active_freq = 0;
    static uint32_t s_last_inactive_freq = 0;
    static int8_t s_last_active_vfo = -1;

    // Apply transverter offsets to both cached frequencies
    uint64_t display_freq_a = apply_transverter_offset_for_display((uint64_t)g_last_raw_vfo_a_hz);
    uint64_t display_freq_b = apply_transverter_offset_for_display((uint64_t)g_last_raw_vfo_b_hz);
    uint32_t display_freq_a_u32 = (uint32_t)display_freq_a;
    uint32_t display_freq_b_u32 = (uint32_t)display_freq_b;

    // Determine which VFO is currently active
    int active_vfo;
    bool is_transmitting = cat_get_transmit_status();
    bool use_tx_perspective = is_transmitting || (split_on && g_tfset_active);

    if (split_on) {
        // In split mode: active VFO depends on TX state or TF-Set engagement
        active_vfo = use_tx_perspective ? tx_vfo_function : rx_vfo_function;
    } else {
        // In normal mode: active VFO is the RX VFO (same as TX VFO)
        active_vfo = rx_vfo_function;
    }

    // Determine active and inactive frequencies
    uint32_t active_freq, inactive_freq;
    if (active_vfo == 0) {
        // Active VFO is A
        active_freq = display_freq_a_u32;
        inactive_freq = display_freq_b_u32;
    } else if (active_vfo == 1) {
        // Active VFO is B
        active_freq = display_freq_b_u32;
        inactive_freq = display_freq_a_u32;
    } else {
        // Memory VFO - use memory frequency from IF command
        uint64_t display_freq_mem = apply_transverter_offset_for_display((uint64_t)g_last_raw_memory_hz);
        uint32_t display_freq_mem_u32 = (uint32_t)display_freq_mem;
        active_freq = display_freq_mem_u32;
        inactive_freq = display_freq_mem_u32;
    }

    // Deduplication: Skip if nothing has changed
    if (active_freq == s_last_active_freq &&
        inactive_freq == s_last_inactive_freq &&
        active_vfo == s_last_active_vfo) {
#if FREQ_UPDATE_STATS_ENABLED
        g_freq_stats.throttled_count++;
#endif
        return; // No change, skip message send
    }

    // Rate-limit frequency updates to avoid flooding the message queue
    static uint32_t last_freq_update_time = 0;
    const uint32_t MIN_FREQ_UPDATE_INTERVAL_MS = 16; // ~60Hz - aligns with 2x LVGL refresh (30 FPS)
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - last_freq_update_time < MIN_FREQ_UPDATE_INTERVAL_MS) {
#if FREQ_UPDATE_STATS_ENABLED
        g_freq_stats.throttled_count++;
#endif
        return; // Throttle updates
    }
    last_freq_update_time = current_time;

    // Update deduplication state
    s_last_active_freq = active_freq;
    s_last_inactive_freq = inactive_freq;
    s_last_active_vfo = (int8_t)active_vfo;

    // Send consolidated VFO update - single message instead of 5
    static vfo_update_t vfo_update;
    vfo_update.active_freq = active_freq;
    vfo_update.inactive_freq = inactive_freq;
    vfo_update.active_vfo = (int8_t)active_vfo;
    radio_subject_set_pointer_async(&radio_vfo_consolidated_subject, &vfo_update, sizeof(vfo_update));

#if FREQ_UPDATE_STATS_ENABLED
    g_freq_stats.ui_queue_count++;
    freq_stats_track_change(active_freq);
    freq_stats_log_if_needed();
#endif
}

/* Legacy direct/async VFO update functions removed */

void parse_if_command(const char *response) {
    static kenwood_if_data_t if_data_payload; // Use a static struct for the message payload

    // Find the start of the IF command
    const char *if_start = strstr(response, "IF");
    if (if_start == NULL) {
        ESP_LOGE(IF_TAG, "IF command not found in response");
        return;
    }

    // Check if we have enough characters for "IF" + 35 payload characters (total 37)
    const char *cmd_check = if_start;
    int count = 0;
    while (*cmd_check && count < IF_COMMAND_LENGTH) {
        cmd_check++;
        count++;
    }

    if (count < IF_COMMAND_LENGTH) {
        ESP_LOGE(IF_TAG, "Incomplete IF command, received %d chars, expected %d", count, IF_COMMAND_LENGTH);
        return;
    }

#if FREQ_UPDATE_STATS_ENABLED
    g_freq_stats.if_cmd_count++;
#endif

    const char *payload_ptr = if_start + 2; // Pointer to the start of the 35-char payload

    // Initialize all fields to safe defaults
    memset(&if_data_payload, 0, sizeof(if_data_payload));
    if_data_payload.mode = -1; // Invalid mode by default

    // P1: VFO Frequency (11 digits) - positions 0-10
    uint64_t raw_vfo_frequency = 0;
    char freq_str[12];
    memcpy(freq_str, payload_ptr, 11);
    freq_str[11] = '\0';
    
    // Validate frequency string contains only digits
    bool freq_valid = true;
    for (int i = 0; i < 11; i++) {
        if (freq_str[i] < '0' || freq_str[i] > '9') {
            freq_valid = false;
            break;
        }
    }
    
    if (freq_valid) {
        raw_vfo_frequency = strtoull(freq_str, NULL, 10);
        if_data_payload.vfo_frequency = raw_vfo_frequency;
    } else {
        ESP_LOGW(IF_TAG, "IF P1: Invalid frequency format: %.11s", payload_ptr);
        if_data_payload.vfo_frequency = 0;
    }

    // P2: 5 spaces (positions 11-15) - Validate these are spaces
    for (int i = 11; i < 16; i++) {
        if (payload_ptr[i] != ' ') {
            ESP_LOGW(IF_TAG, "IF P2: Expected space at position %d, got '%c'", i, payload_ptr[i]);
        }
    }

    // P3: RIT/XIT Frequency Offset (5 chars, signed) - positions 16-20
    char offset_str[6];
    memcpy(offset_str, payload_ptr + 16, 5);
    offset_str[5] = '\0';
    
    // Parse signed offset: " 1234" (positive) or "-1234" (negative)
    if (offset_str[0] == ' ') {
        // Positive offset
        char pos_str[5];
        memcpy(pos_str, offset_str + 1, 4);
        pos_str[4] = '\0';
        if_data_payload.rit_xit_frequency = atoi(pos_str);
    } else if (offset_str[0] == '-') {
        // Negative offset
        char neg_str[5];
        memcpy(neg_str, offset_str + 1, 4);
        neg_str[4] = '\0';
        if_data_payload.rit_xit_frequency = -atoi(neg_str);
    } else {
        ESP_LOGW(IF_TAG, "IF P3: Invalid RIT/XIT offset format: %.5s", offset_str);
        if_data_payload.rit_xit_frequency = 0;
    }
    
    // Validate offset range ±9990 Hz
    if (abs(if_data_payload.rit_xit_frequency) > 9990) {
        ESP_LOGW(IF_TAG, "IF P3: RIT/XIT offset out of range: %d", if_data_payload.rit_xit_frequency);
        if_data_payload.rit_xit_frequency = 0;
    }

    // P4: RIT Status (1 digit: 0=OFF, 1=ON) - position 21
    char rit_char = payload_ptr[21];
    if (rit_char == '0' || rit_char == '1') {
        if_data_payload.rit_on = (rit_char == '1');
    } else {
        ESP_LOGW(IF_TAG, "IF P4: Invalid RIT status: %c", rit_char);
        if_data_payload.rit_on = false;
    }

    // P5: XIT Status (1 digit: 0=OFF, 1=ON) - position 22
    char xit_char = payload_ptr[22];
    if (xit_char == '0' || xit_char == '1') {
        if_data_payload.xit_on = (xit_char == '1');
    } else {
        ESP_LOGW(IF_TAG, "IF P5: Invalid XIT status: %c", xit_char);
        if_data_payload.xit_on = false;
    }

    // P6 + P7: Memory Channel (P6=hundreds, P7=tens+ones) - positions 23, 24-25
    char hundreds = payload_ptr[23];
    char tens_ones[3];
    memcpy(tens_ones, payload_ptr + 24, 2);
    tens_ones[2] = '\0';
    
    if (hundreds >= '0' && hundreds <= '9' && 
        tens_ones[0] >= '0' && tens_ones[0] <= '9' &&
        tens_ones[1] >= '0' && tens_ones[1] <= '9') {
        if_data_payload.memory_channel = (hundreds - '0') * 100 + atoi(tens_ones);
        // Validate memory channel range (0-999)
        if (if_data_payload.memory_channel > 999) {
            ESP_LOGW(IF_TAG, "IF P6P7: Memory channel out of range: %d", if_data_payload.memory_channel);
            if_data_payload.memory_channel = 0;
        }
    } else {
        ESP_LOGW(IF_TAG, "IF P6P7: Invalid memory channel format: %c%.2s", hundreds, tens_ones);
        if_data_payload.memory_channel = 0;
    }

    // P8: TX/RX Status (1 digit: 0=RX, 1=TX) - position 26
    char tx_rx_char = payload_ptr[26];
    if (tx_rx_char == '0' || tx_rx_char == '1') {
        if_data_payload.tx_rx = (tx_rx_char == '1');
    } else {
        ESP_LOGW(IF_TAG, "IF P8: Invalid TX/RX status: %c", tx_rx_char);
        if_data_payload.tx_rx = false; // Default to RX
    }
    g_is_transmitting = if_data_payload.tx_rx; // Update global transmit status
    ESP_LOGV(TAG, "IF Command: TX/RX status updated - %s", g_is_transmitting ? "TRANSMITTING" : "RECEIVING");

    // Notify UI of TX/RX status change via LVGL 9 native observer
    radio_subject_set_int_async(&radio_tx_status_subject, g_is_transmitting ? 1 : 0);

    // P9: Operating Mode (1 digit/char, see MD command) - position 27
    char mode_char = payload_ptr[27];
    if (mode_char >= '1' && mode_char <= '9') {
        if_data_payload.mode = mode_char - '0';
    } else {
        switch (mode_char) {
            case 'A': if_data_payload.mode = 10; break; // DATA-CW
            case 'B': if_data_payload.mode = 11; break; // DATA-LSB  
            case 'C': if_data_payload.mode = 12; break; // DATA-USB
            default:
                ESP_LOGW(IF_TAG, "IF P9: Invalid operating mode: %c", mode_char);
                if_data_payload.mode = -1; // Error/Unknown
                break;
        }
    }

    // P10: VFO Function (1 digit: 0=VFOA, 1=VFOB, 2=MEM, see FR/FT) - position 28
    char func_char = payload_ptr[28];
    if (func_char >= '0' && func_char <= '2') {
        if_data_payload.function = func_char - '0';
    } else {
        ESP_LOGW(IF_TAG, "IF P10: Invalid VFO function: %c", func_char);
        if_data_payload.function = 0; // Default to VFO A
    }

    // P11: Scan Status (1 digit) - position 29
    char scan_char = payload_ptr[29];
    if (scan_char >= '0' && scan_char <= '9') {
        if_data_payload.scan_on = (scan_char != '0');
    } else {
        ESP_LOGW(IF_TAG, "IF P11: Invalid scan status: %c", scan_char);
        if_data_payload.scan_on = false;
    }

    // P12: Split Status (1 digit: 0=Simplex, 1=Split) - position 30
    char split_char = payload_ptr[30];
    if (split_char == '0' || split_char == '1') {
        if_data_payload.split_on = (split_char == '1');
    } else {
        ESP_LOGW(IF_TAG, "IF P12: Invalid split status: %c", split_char);
        if_data_payload.split_on = false; // Default to Simplex
    }

    // P13: Tone Status (1 digit: 0=OFF, 1=Tone ON, 2=CTCSS ON, 3=Cross Tone ON) - position 31
    char tone_char = payload_ptr[31];
    if (tone_char >= '0' && tone_char <= '3') {
        if_data_payload.tone_on = (tone_char != '0');
    } else {
        ESP_LOGW(IF_TAG, "IF P13: Invalid tone status: %c", tone_char);
        if_data_payload.tone_on = false;
    }

    // P14: Tone Frequency Index (2 digits: 00-42) - positions 32-33
    char tone_freq_str[3];
    memcpy(tone_freq_str, payload_ptr + 32, 2);
    tone_freq_str[2] = '\0';
    
    if (tone_freq_str[0] >= '0' && tone_freq_str[0] <= '9' &&
        tone_freq_str[1] >= '0' && tone_freq_str[1] <= '9') {
        if_data_payload.tone_number = atoi(tone_freq_str);
        // Validate tone frequency index range (0-42)
        if (if_data_payload.tone_number > 42) {
            ESP_LOGW(IF_TAG, "IF P14: Tone frequency index out of range: %d", if_data_payload.tone_number);
            if_data_payload.tone_number = 0;
        }
    } else {
        ESP_LOGW(IF_TAG, "IF P14: Invalid tone frequency index: %.2s", tone_freq_str);
        if_data_payload.tone_number = 0;
    }

    // P15: Always 0 (1 digit) - position 34
    char p15_char = payload_ptr[34];
    if (p15_char == '0') {
        if_data_payload.p15_value = 0;
    } else {
        ESP_LOGW(IF_TAG, "IF P15: Expected '0', got '%c'", p15_char);
        if_data_payload.p15_value = p15_char - '0'; // Store actual value for debugging
    }

    // Initialize shift_status to 0 (not directly mapped from IF parameters)
    if_data_payload.shift_status = 0;

    ESP_LOGV(
        IF_TAG,
        "IF: Freq=%lld, RIT/XIT=%d, RIT=%d, XIT=%d, MemCh=%d, TX=%d, Mode=%d(char:%c), Func=%d(char:%c), Scan=%d, Split=%d(char:%c), Tone=%d, ToneNum=%d, P15=%d",
        if_data_payload.vfo_frequency, if_data_payload.rit_xit_frequency,
        if_data_payload.rit_on, if_data_payload.xit_on, if_data_payload.memory_channel,
        if_data_payload.tx_rx, if_data_payload.mode, mode_char,
        if_data_payload.function, func_char, if_data_payload.scan_on,
        if_data_payload.split_on, split_char, if_data_payload.tone_on, if_data_payload.tone_number,
        if_data_payload.p15_value);

    // Update global RX/TX VFO assignments using TX/RX context from P8
    int previous_rx_vfo = g_current_rx_vfo_function;
    int previous_tx_vfo = g_current_tx_vfo_function;

    if (if_data_payload.tx_rx) {
        // Transmitting: prefer updating the TX VFO, keep RX assignment stable unless simplex
        if (if_data_payload.function >= 0 && if_data_payload.function <= 2) {
            if (!if_data_payload.split_on) {
                g_current_rx_vfo_function = if_data_payload.function;
                g_current_tx_vfo_function = if_data_payload.function;
            } else if (if_data_payload.function != g_current_rx_vfo_function) {
                g_current_tx_vfo_function = if_data_payload.function;
            } else if (g_current_tx_vfo_function < 0 || g_current_tx_vfo_function > 2) {
                g_current_tx_vfo_function = (if_data_payload.function == 0) ? 1 : 0;
            }
        }
    } else {
        // Receiving: IF function mirrors the RX VFO; infer TX when needed
        if (if_data_payload.function >= 0 && if_data_payload.function <= 2) {
            g_current_rx_vfo_function = if_data_payload.function;

            if (!if_data_payload.split_on) {
                g_current_tx_vfo_function = if_data_payload.function;
            } else if (g_current_tx_vfo_function == g_current_rx_vfo_function ||
                       g_current_tx_vfo_function < 0 || g_current_tx_vfo_function > 2) {
                g_current_tx_vfo_function = (if_data_payload.function == 0) ? 1 : 0;
            }
        }
    }

    if (if_data_payload.vfo_frequency >= 30000 && if_data_payload.vfo_frequency <= 300000000) {
        uint32_t freq_u32 = (uint32_t) if_data_payload.vfo_frequency;
        if (if_data_payload.function == 0) {
            g_last_raw_vfo_a_hz = freq_u32;
        } else if (if_data_payload.function == 1) {
            g_last_raw_vfo_b_hz = freq_u32;
        } else if (if_data_payload.function == 2) {
            g_last_raw_memory_hz = freq_u32;  // Store memory channel frequency
        }
    }

    if (previous_rx_vfo != g_current_rx_vfo_function || previous_tx_vfo != g_current_tx_vfo_function) {
        ESP_LOGI(IF_TAG,
                 "IF-derived VFO roles updated: RX=VFO%c -> VFO%c, TX=VFO%c -> VFO%c (split=%s, tx=%s)",
                 previous_rx_vfo == 0 ? 'A' : (previous_rx_vfo == 1 ? 'B' : 'M'),
                 g_current_rx_vfo_function == 0 ? 'A' : (g_current_rx_vfo_function == 1 ? 'B' : 'M'),
                 previous_tx_vfo == 0 ? 'A' : (previous_tx_vfo == 1 ? 'B' : 'M'),
                 g_current_tx_vfo_function == 0 ? 'A' : (g_current_tx_vfo_function == 1 ? 'B' : 'M'),
                 if_data_payload.split_on ? "ON" : "OFF",
                 if_data_payload.tx_rx ? "TX" : "RX");
    }

    if (g_current_split_on != if_data_payload.split_on) {
        ESP_LOGI(IF_TAG, "Split status changed by IF command: %s -> %s",
                 g_current_split_on ? "ON" : "OFF",
                 if_data_payload.split_on ? "ON" : "OFF");
    }
    g_current_split_on = if_data_payload.split_on;

    radio_subject_set_int_async(&radio_split_subject, g_current_split_on ? 1 : 0);
    ESP_LOGD(IF_TAG, "IF parsed: tx=%d, split=%d, updating IF data subject",
             if_data_payload.tx_rx, if_data_payload.split_on);
    radio_subject_set_pointer_async(&radio_if_data_subject, &if_data_payload, sizeof(if_data_payload));

    update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, cat_get_split_status());

    radio_subject_set_int_async(&radio_vfo_function_subject, if_data_payload.function);

    // When in memory mode, check if memory channel changed and request new data
    if (if_data_payload.function == 2 && if_data_payload.memory_channel >= 0 &&
        if_data_payload.memory_channel <= 999) {
        uint16_t last_channel = cat_polling_get_last_memory_channel();
        uint16_t current_channel = (uint16_t)if_data_payload.memory_channel;
        if (last_channel != current_channel) {
            ESP_LOGI(IF_TAG, "Memory channel changed: %u -> %u, requesting MR data",
                     last_channel, current_channel);
            cat_polling_request_memory_channel(current_channel);
        }
    }
}

void parse_ri_command(const char *response) {
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') return;
    int ri_value = response[2] - '0';
    ESP_LOGV(TAG, "Parsed RI command: %d", ri_value);
}

void parse_rm_command(const char *response) {
    // Command format: RMP1P2P2P2P2; per TS-590SG specification
    // RM (2 chars) + P1 (1 char) + P2 (4 chars) = 7 chars minimum
    // Prefix verified by dispatcher
    if (response[2] == '\0' || response[6] == '\0') {
        ESP_LOGW(TAG, "RM command too short: %s", response);
        return;
    }

    char p1_meter_type = response[2];

    // Validate P1 per specification: 0=No selection, 1=SWR, 2=COMP, 3=ALC
    if (p1_meter_type < '0' || p1_meter_type > '3') {
        ESP_LOGW(TAG, "RM: Invalid P1 meter type '%c', expected 0-3", p1_meter_type);
        return;
    }

    int meter_value = parse_int(response + 3, 4); // Parse all 4 digits of P2

    // Clamp value to 0-30 range
    if (meter_value < 0) meter_value = 0;
    if (meter_value > 30) meter_value = 30;

    switch (p1_meter_type) {
        case '1': {
            // SWR - Send the raw segment count for the UI bar meter (unchanged behavior)
            // LVGL 9 observer: Update SWR subject for migration
            radio_subject_set_int_async(&radio_swr_subject, meter_value);

            // Calculate and send the actual SWR float value
            float actual_swr = interpolate((float) meter_value, &segments_to_swr_cal_table);
#if METER_LOG_ENABLED
            g_meter_state.swr = meter_value;
            g_meter_state.swr_actual = actual_swr;
            meter_log_update();
#endif
            break;
        }
        case '2': // COMP (Compression/Speech Processor)
            // LVGL 9 observer: Update COMP subject
            radio_subject_set_int_async(&radio_comp_subject, meter_value);
            ESP_LOGV(TAG, "COMP meter: %d", meter_value);
            break;
        case '3': // ALC
            // LVGL 9 observer: Update ALC subject for migration
            radio_subject_set_int_async(&radio_alc_subject, meter_value);
#if METER_LOG_ENABLED
            g_meter_state.alc = meter_value;
            meter_log_update();
#endif
            break;
        case '0': // No selection
            break;
        default:
            ESP_LOGW(TAG, "Invalid RM command meter type: %c", p1_meter_type);
            break;
    }
}

void parse_sm_command(const char *response) {
    // Command format: SMP1P2P2P2P2; per TS-590SG specification
    // SM (2 chars) + P1 (1 char) + P2 (4 chars) = 7 chars minimum
    // Prefix verified by dispatcher
    if (response[2] == '\0' || response[6] == '\0') return;

    // Validate P1 (always 0 per specification)
    if (response[2] != '0') {
        ESP_LOGW(TAG, "SM: Invalid P1 '%c', expected '0'", response[2]);
        return;
    }

    // Parse P2 (4 digits starting at position 3)
    int sm_value = parse_int(response + 3, 4);

    // Clamp to specification range 0-30
    if (sm_value < 0) sm_value = 0;
    if (sm_value > 30) sm_value = 30;

    // Always send updates - UI layer handles throttling (METER_UPDATE_INTERVAL_MS = 30ms)
    // This ensures we don't drop valid CAT responses at parser level
    bool is_tx = cat_get_transmit_status();

#if METER_LOG_ENABLED
    g_meter_state.s_meter = sm_value;
    g_meter_state.is_tx = is_tx;
    meter_log_update();
#endif

    ESP_LOGV(TAG, "Parsed SM command: P1='%c', P2=%d. Sending update.", response[2], sm_value);
    // LVGL 9 observer: Update S-meter subject for migration
    radio_subject_set_int_async(&radio_s_meter_subject, sm_value);

    // Update PEP system if we're transmitting (SM command shows power during TX)
    if (is_tx) {
        pep_update_power(sm_value);
    }
}

void parse_ts_command(const char *response) {
    // Command format: TS P1; where P1 = 0 (OFF) or 1 (ON)
    if (strncmp(response, "TS", 2) != 0) {
        return;
    }

    size_t len = strlen(response);
    if (len < 3) {
        ESP_LOGW(TAG, "TS command too short: %s", response);
        return;
    }

    char state_char = response[2];

    // Handle query form "TS;" which has '\n' or ';' immediately after command
    if (state_char == ';') {
        ESP_LOGV(TAG, "TS query received without state change: %s", response);
        return;
    }

    if (state_char != '0' && state_char != '1') {
        ESP_LOGW(TAG, "Invalid TS command parameter: %c", state_char);
        return;
    }

    bool tfset_requested = (state_char == '1');

    if (g_tfset_active == tfset_requested) {
        // No change; avoid redundant updates
        ESP_LOGV(TAG, "TS command received with unchanged state (%s)", tfset_requested ? "ON" : "OFF");
        return;
    }

    g_tfset_active = tfset_requested;
    ESP_LOGI(TAG, "TF-Set state changed: %s", g_tfset_active ? "ENABLED" : "DISABLED");

    // Update frequency displays so active/inactive VFO assignments follow TF-Set behavior in split mode
    bool effective_split = cat_get_split_status();
    update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, effective_split);
}

void parse_tx_command(const char *response) {
    // TX (2 chars) + P1 (1 char) = 3 chars minimum; prefix verified by dispatcher
    if (response[2] == '\0') return;
    char p1 = response[2];
    if (p1 == '0' || p1 == '1' || p1 == '2') {
        g_is_transmitting = true;
        radio_subject_set_int_async(&radio_tx_status_subject, 1);

        // Trigger frequency display update to switch active VFO in split mode
        bool effective_split = cat_get_split_status();
        ESP_LOGD(TAG, "TX: Split=%s RX=VFO%c TX=VFO%c",
                 effective_split ? "ON" : "OFF",
                 g_current_rx_vfo_function == 0 ? 'A' : (g_current_rx_vfo_function == 1 ? 'B' : 'M'),
                 g_current_tx_vfo_function == 0 ? 'A' : (g_current_tx_vfo_function == 1 ? 'B' : 'M'));
        update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, effective_split);
    } else {
        ESP_LOGW(TAG, "Invalid TX command parameter: %c", p1);
    }
}

void parse_rx_command(const char *response) {
    // RX command just needs 2 chars; prefix verified by dispatcher
    (void)response; // Prefix already verified
    g_is_transmitting = false;
    radio_subject_set_int_async(&radio_tx_status_subject, 0);

    // Reset PEP data when switching to receive mode
    pep_reset();

    // Trigger frequency display update to switch active VFO back to RX VFO in split mode
    bool effective_split = cat_get_split_status();
    ESP_LOGD(TAG, "RX: Split=%s RX=VFO%c TX=VFO%c",
             effective_split ? "ON" : "OFF",
             g_current_rx_vfo_function == 0 ? 'A' : (g_current_rx_vfo_function == 1 ? 'B' : 'M'),
             g_current_tx_vfo_function == 0 ? 'A' : (g_current_tx_vfo_function == 1 ? 'B' : 'M'));
    update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, effective_split);
}

// ui_at_status_t is now defined in cat_shared_types.h

void parse_ac_command(const char *response) {
    // Command format: AC P1 P2 P3 ;
    // P1: response[2] - 0: RX-AT THRU, 1: RX-AT IN
    // P2: response[3] - 0: TX-AT THRU, 1: TX-AT IN
    // P3: response[4] - 0: Stop tuning / Tuning is stopped, 1: Start tuning / Tuning in progress
    // Example: AC010; (RX THRU, TX IN, Tuning Stopped) or AC010
    // Minimum valid length for "ACP1P2P3" is 5 characters. Semicolon is optional.
    size_t len = strlen(response);
    if (strncmp(response, "AC", 2) == 0 && (len == 5 || (len == 6 && response[5] == ';'))) {
        ui_at_status_t at_status = {0};
        at_status.rx_at_in = (response[2] == '1');
        at_status.tx_at_in = (response[3] == '1');
        at_status.tuning_in_progress = (response[4] == '1');

        ESP_LOGV(TAG, "Parsed AC command: RX_AT_IN=%d, TX_AT_IN=%d, TUNING_IN_PROGRESS=%d",
                 at_status.rx_at_in, at_status.tx_at_in, at_status.tuning_in_progress);
        // AT status bitmask: bit0=rx_at_in, bit1=tx_at_in, bit2=tuning_in_progress
        int32_t at_bitmask = (at_status.rx_at_in ? 0x01 : 0) |
                             (at_status.tx_at_in ? 0x02 : 0) |
                             (at_status.tuning_in_progress ? 0x04 : 0);
        radio_subject_set_int_async(&radio_at_status_subject, at_bitmask);
    } else {
        ESP_LOGW(TAG, "Invalid AC command format or length: %s", response);
    }
}

// antenna
void parse_an_command(const char *response) {
    // Command format: ANP1P2P3; per TS-590SG specification
    // P1: 0 (ANT1), 1 (ANT2) - only these values allowed in answer format
    // P2: 0 (RX ANT not used), 1 (RX ANT used)  
    // P3: 0 (Drive/Antenna Out OFF), 1 (Drive/Antenna Out ON)
    size_t len = strlen(response);
    if (strncmp(response, "AN", 2) == 0 && (len == 5 || (len == 6 && response[5] == ';'))) {
        char p1_char = response[2]; // Antenna select
        char p2_char = response[3]; // RX ANT used  
        char p3_char = response[4]; // Drive/Antenna Out
        
        // Per TS-590SG spec: P1-P3 allow '0', '1', or '9' (no change)
        // In answer format, '9' means "no change" and should be handled gracefully
        bool p1_valid = (p1_char == '0' || p1_char == '1' || p1_char == '9');
        bool p2_valid = (p2_char == '0' || p2_char == '1' || p2_char == '9');
        bool p3_valid = (p3_char == '0' || p3_char == '1' || p3_char == '9');
        
        if (p1_valid && p2_valid && p3_valid) {
            // Handle '9' as "no change" - preserve current values
            int antenna_select = (p1_char == '9') ? g_current_antenna_select : (p1_char - '0');
            bool rx_ant_used = (p2_char == '9') ? g_current_rx_ant_used : (p2_char == '1');
            bool drive_out_on = (p3_char == '9') ? g_current_drive_out_on : (p3_char == '1');
            
            // Update global state tracking
            g_current_antenna_select = antenna_select;
            g_current_rx_ant_used = rx_ant_used;
            g_current_drive_out_on = drive_out_on;
            
            ESP_LOGV(TAG, "Parsed AN: ANT=%d, RX_ANT=%d, DRIVE_OUT=%d (P1=%c, P2=%c, P3=%c)", 
                     antenna_select, rx_ant_used, drive_out_on, p1_char, p2_char, p3_char);
            
            radio_subject_set_int_async(&radio_antenna_select_subject, antenna_select);
        } else {
            ESP_LOGW(TAG, "AN: Invalid parameters: P1='%c' (expected 0-1), P2='%c' (expected 0-1), P3='%c' (expected 0-1)", 
                     p1_char, p2_char, p3_char);
        }
    } else {
        ESP_LOGW(TAG, "Invalid AN command format or length: %s. Expected ANP1P2P3[;]", response);
    }
}

void parse_ai_command(const char *response) {
    // Command format: AIP1; per TS-590SG specification
    // P1: 0 (AI OFF), 2 (AI ON no backup), 4 (AI ON with backup)
    // Prefix verified by dispatcher; just check minimum length
    if (response[2] == '\0') {
        ESP_LOGW(TAG, "Invalid AI command format: %s", response);
        return;
    }

    char ai_mode_char = response[2];
    ESP_LOGD(TAG, "Parsed AI command: AI%c", ai_mode_char);

    // Validate AI mode per specification
    if (ai_mode_char == '0') {
        // AI OFF - need polling
        ESP_LOGD(TAG, "AI mode: OFF (polling needed)");
        cat_polling_update_ai_mode(AI_MODE_OFF);
    } else if (ai_mode_char == '2') {
        // AI ON (no backup) - no polling needed
        ESP_LOGD(TAG, "AI mode: ON (no backup, no polling needed)");
        cat_polling_update_ai_mode(AI_MODE_ON);
    } else if (ai_mode_char == '4') {
        // AI ON (with backup) - no polling needed
        ESP_LOGD(TAG, "AI mode: ON (with backup, no polling needed)");
        cat_polling_update_ai_mode(AI_MODE_ON_BACKUP);
    } else {
        ESP_LOGW(TAG, "AI: Invalid mode '%c', expected 0, 2, or 4", ai_mode_char);
    }
}

void parse_af_gain(const char *response) {
    // TS-590SG spec (see ts590sg_cat_commands_v3.json):
    // - set/answer: "AGP1P2P2P2;" where P1 is '0' and P2 is 000–255
    // - read:      "AGP1;" where P1 is '0'  (i.e., "AG0;")
    // Our UART reader strips the terminator, so examples seen here:
    // - answer/set: "AG0ddd" (length >= 6)
    // - read req.:  "AG0"    (length == 3)

    if (strncmp(response, "AG", 2) != 0) {
        return;
    }

    size_t len = strlen(response);

    // Handle read request form "AG0" (terminator removed by reader)
    if (len == 3 && response[2] == '0') {
        // Benign read query; nothing to update in state. Do not warn.
        ESP_LOGV(TAG, "AG read request received");
        return;
    }

    // Handle set/answer form: expect "AG0ddd" (ddd = 000–255)
    if (len >= 6 && response[2] == '0') {
        int af_gain = parse_int(response + 3, 3); // P2 (3 digits)
        if (af_gain >= 0 && af_gain <= 255) {
            ESP_LOGV(TAG, "Parsed AF Gain: %d", af_gain);
            radio_subject_set_int_async(&radio_af_gain_subject, af_gain);
        } else {
            ESP_LOGW(TAG, "AF Gain value out of range (0-255): %d", af_gain);
        }
        return;
    }

    // Anything else is malformed per spec
    if (len >= 3) {
        if (response[2] != '0') {
            ESP_LOGW(TAG, "Invalid AG command P1 parameter: %c, expected '0'", response[2]);
        } else {
            ESP_LOGW(TAG, "Invalid AG command format or length: %s", response);
        }
    } else {
        ESP_LOGW(TAG, "Invalid AG command format or length: %s", response);
    }
}

void parse_rf_gain(const char *response) {
    // Command format: RGP1P1P1; where P1P1P1 is 000-255
    // Example: RG000; RG100; RG255; or RG085 (without semicolon)
    // Minimum length: RG000 (5 chars)
    if (strncmp(response, "RG", 2) == 0 && strlen(response) >= 5) {
        int rf_gain = -1;
        // Extract P1P1P1 (3 digits starting from response[2])
        rf_gain = parse_int(response + 2, 3);
        if (rf_gain >= 0 && rf_gain <= 255) {
            ESP_LOGV(TAG, "Parsed RF Gain: %d", rf_gain);
            radio_subject_set_int_async(&radio_rf_gain_subject, rf_gain);
        } else {
            ESP_LOGW(TAG, "RF Gain value out of range (0-255): %d", rf_gain);
        }
    } else {
        ESP_LOGW(TAG, "Invalid RG command format or length: %s", response);
    }
}

void parse_xi_command(const char *response) {
    // Command format: XIP1{11}P2{1}P3{1}P4{2};  (e.g., XI007050000002100;)
    // P1: Frequency (11 digits)
    // P2: Mode (1 digit)
    // P3: Data mode (1 digit: 0=OFF, 1=ON)
    // P4: Always "00" (2 digits)
    // Minimum length: "XI" + 11 + 1 + 1 + 2 = 17 chars (excluding optional semicolon)
    if (strncmp(response, "XI", 2) == 0 && strlen(response) >= 17) {
        static kenwood_xi_data_t xi_data_payload; // Static struct for the message payload
        memset(&xi_data_payload, 0, sizeof(kenwood_xi_data_t));

        // Parse P1: Transmit Frequency (11 digits)
        // response + 2 points to the start of P1
        xi_data_payload.transmit_frequency = (uint32_t) parse_long(response + 2, 11);
        ESP_LOGV(TAG, "Parsed XI Transmit Frequency: %" PRIu32, xi_data_payload.transmit_frequency);
        // VFO B often used for TX freq in split

        // Parse P2: Transmission Mode (1 digit)
        // response + 2 + 11 points to P2
        if (response[13] >= '0' && response[13] <= '9') {
            xi_data_payload.transmission_mode = response[13] - '0';
            ESP_LOGV(TAG, "Parsed XI Mode: %d", xi_data_payload.transmission_mode);
            radio_subject_set_int_async(&radio_mode_subject, xi_data_payload.transmission_mode);
        } else {
            ESP_LOGW(TAG, "Invalid XI mode character: %c", response[13]);
            xi_data_payload.transmission_mode = -1; // Indicate error
        }

        // Parse P3: Data Mode (1 digit)
        // response + 2 + 11 + 1 points to P3
        xi_data_payload.data_mode_on = (response[14] == '1');
        ESP_LOGV(TAG, "Parsed XI Data Mode: %s", xi_data_payload.data_mode_on ? "ON" : "OFF");
        radio_subject_set_int_async(&radio_data_mode_subject, xi_data_payload.data_mode_on ? 1 : 0);

        // P4: Always "00" per TS-590SG specification - validate this
        if (response[15] != '0' || response[16] != '0') {
            ESP_LOGW(TAG, "XI: Invalid P4 '%.2s', expected '00'", response + 15);
        }

        // Send the consolidated XI data message
        radio_subject_set_pointer_async(&radio_xi_data_subject, &xi_data_payload, sizeof(xi_data_payload));
    } else {
        ESP_LOGD(TAG, "XI command too short or not XI: %s", response);
    }
}

void parse_fr_ft_command(const char *response) {
    if (strlen(response) < 3) {
        // Needs "FRx" or "FTx"
        ESP_LOGW(TAG, "FR/FT command too short: %s", response);
        return;
    }

    char p1_char = response[2];
    int p1_val = -1;

    // FR supports 0/1/2, FT only supports 0/1 according to TS-590SG spec
    bool command_is_ft = (response[0] == 'F' && response[1] == 'T');
    
    if (command_is_ft) {
        // FT only supports 0/1 according to spec
        if (p1_char >= '0' && p1_char <= '1') {
            p1_val = p1_char - '0';
        } else {
            ESP_LOGW(TAG, "Invalid P1 parameter for FT command (only 0/1 supported): %c", p1_char);
            return;
        }
    } else {
        // FR supports 0/1/2
        if (p1_char >= '0' && p1_char <= '2') {
            p1_val = p1_char - '0';
        } else {
            ESP_LOGW(TAG, "Invalid P1 parameter for FR command: %c", p1_char);
            return;
        }
    }

    if (command_is_ft) {
        // FT commands set TX VFO only
        // FT0 = TX on VFO A, FT1 = TX on VFO B
        ESP_LOGV(TAG, "Parsed FT%d: TX VFO set to %s.",
                 p1_val, (p1_val == 0) ? "A" : "B");

        // Update global TX VFO state
        g_current_tx_vfo_function = p1_val;

        // Update polling manager with TX VFO information
        cat_vfo_t tx_vfo = (p1_val == 0) ? VFO_A : VFO_B;
        cat_polling_update_tx_vfo(tx_vfo);

    } else {
        // FR commands set RX VFO
        // FR0 = RX on VFO A, FR1 = RX on VFO B, FR2 = RX on Memory
        ESP_LOGV(TAG, "Parsed FR%d: RX VFO function set to %d.", p1_val, p1_val);

        // Update global RX VFO state
        g_current_rx_vfo_function = p1_val;

        // Update polling manager with RX VFO information
        cat_vfo_t rx_vfo = VFO_UNKNOWN;
        if (p1_val == 0) {
            rx_vfo = VFO_A;
        } else if (p1_val == 1) {
            rx_vfo = VFO_B;
        } else if (p1_val == 2) {
            rx_vfo = VFO_MEM;
        }

        if (rx_vfo != VFO_UNKNOWN) {
            cat_polling_update_rx_vfo(rx_vfo);
        }
    }

    // Detect split mode by comparing RX vs TX VFO (for ARCI button support)
    // Split is ON when RX and TX are on different VFOs (both must be A or B, not MEM)
    bool detected_split = false;
    if (g_current_rx_vfo_function <= 1 && g_current_tx_vfo_function <= 1) {
        detected_split = (g_current_rx_vfo_function != g_current_tx_vfo_function);
    }

    // Update split status if it changed
    if (detected_split != g_current_split_on) {
        g_current_split_on = detected_split;
        ESP_LOGI(TAG, "FR/FT detected split mode change: RX=VFO%c, TX=VFO%c, Split=%s",
                 g_current_rx_vfo_function == 0 ? 'A' : (g_current_rx_vfo_function == 1 ? 'B' : 'M'),
                 g_current_tx_vfo_function == 0 ? 'A' : 'B',
                 detected_split ? "ON" : "OFF");

        // Update split subject for UI update
        radio_subject_set_int_async(&radio_split_subject, detected_split ? 1 : 0);
    }

    // Trigger frequency display update
    bool current_split = cat_get_split_status();
    ESP_LOGV(TAG, "FR/FT command processed: RX=VFO%c, TX=VFO%c, Split=%s",
             g_current_rx_vfo_function == 0 ? 'A' : (g_current_rx_vfo_function == 1 ? 'B' : 'M'),
             g_current_tx_vfo_function == 0 ? 'A' : 'B',
             current_split ? "ON" : "OFF");

    update_frequency_displays(g_current_rx_vfo_function, g_current_tx_vfo_function, current_split);

    // Send VFO function update message (FR command only)
    if (!command_is_ft) {
        radio_subject_set_int_async(&radio_vfo_function_subject, p1_val);

        // When entering memory mode (FR2), request memory channel data
        // We'll use the IF command's memory_channel field to get the channel number
        // and request it via MR command
        if (p1_val == 2) {
            // Get current memory channel from IF data
            kenwood_if_data_t* if_data = radio_get_if_data_buffer();
            if (if_data && if_data->memory_channel >= 0 && if_data->memory_channel <= 999) {
                cat_polling_request_memory_channel((uint16_t)if_data->memory_channel);
            }
        }
    }
}

/**
 * @brief Parse SH/SL CAT commands from TS-590SG.
 *
 * Format:
 *   Read query:   "SH[;]" or "SL[;]"
 *   Set command:  "SHnn[;]" or "SLnn[;]"   (nn = 00–99)
 *   (items in [ ] are optional)
 *
 * @param response  null-terminated CAT string
 * @param isHigh    true=>SH (high-cut), false=>SL (low-cut)
 * @return the parsed index (0–99), or 0 on error or for a pure-read query
 */
uint8_t parse_sh_sl(const char *response, bool isHigh) {
    ESP_LOGI(TAG, "parse_sh_sl called: \"%s\", isHigh=%d", response, isHigh);
    const char *p = response;
    size_t len = strlen(response);

    // Minimum length is "SH" or "SL" (2 chars)
    if (len < 2 || p[0] != 'S' || (p[1] != 'H' && p[1] != 'L')) {
        ESP_LOGW(TAG, "SH/SL packet too short or invalid prefix: \"%s\"", response);
        return 0;
    }

    // Check for optional semicolon at the end of valid length commands
    if (len == 3 || len == 5) {
        // Lengths where a semicolon might be
        if (p[len - 1] != ';') {
            ESP_LOGW(TAG, "SH/SL packet for length %zu missing semicolon or has extra chars: \"%s\"", len, response);
            // Allow processing if it's a valid command without semicolon (len 2 or 4)
            // but if len is 3 or 5, it MUST end in ';'
            return 0;
        }
    } else if (len != 2 && len != 4) {
        // Valid lengths without semicolon
        ESP_LOGW(TAG, "SH/SL packet invalid length: %zu, content: \"%s\"", len, response);
        return 0;
    }


    // Are we handling the right letter?
    if (isHigh != (p[1] == 'H')) {
        // shouldn't happen if your hash dispatch is correct
        ESP_LOGW(TAG, "parse_sh_sl called for wrong letter: %s", response);
        return 0;
    }

    // Case 1: read-query ("SH" or "SL", with or without ";")
    // Length 2 ("SH") or Length 3 ("SH;")
    if (len == 2 || (len == 3 && p[2] == ';')) {
        uint8_t current = isHigh
                              ? get_current_highcut_index()
                              : get_current_lowcut_index();

        // echo back the current setting:
        // e.g. "SH05;" or "SL12;"
        char echo[6];
        snprintf(echo, sizeof(echo), "%c%c%02u;",
                 'S', isHigh ? 'H' : 'L', current);
        // Send reply using the configured CAT UART port without extra CR/LF
        (void) uart_write_raw(echo, strlen(echo));

        ESP_LOGV(TAG, "Answered %s with index %u",
                 isHigh ? "SH" : "SL", current);
        return current;
    }

    // Case 2: set command ("SHnn" or "SLnn", with or without ";")
    // Length 4 ("SHnn") or Length 5 ("SHnn;")
    // Must have exactly two digits at p[2] and p[3].
    // If len is 5, p[4] must be ';'. If len is 4, p[4] is the null terminator.
    if ((len == 4 || (len == 5 && p[4] == ';')) &&
        isdigit((unsigned char)p[2]) && isdigit((unsigned char)p[3])) {
        uint8_t idx = (p[2] - '0') * 10 + (p[3] - '0');

        // apply it to your filter logic:
        if (isHigh) {
            set_highcut_index(idx);
        } else {
            set_lowcut_index(idx);
        }

        ESP_LOGV(TAG, "Parsed %s index: %u",
                 isHigh ? "SH" : "SL", idx);

        // Notification to LVGL/UI is now handled within set_highcut_index/set_lowcut_index

        return idx;
    }

    // anything else is invalid
    ESP_LOGW(TAG, "%s command invalid format: \"%s\"",
             isHigh ? "SH" : "SL", response);
    return 0;
}

void parse_fw_command(const char *response) {
    // Command format: FW P1P1P1P1 ; where P1P1P1P1 is 0000-9999 Hz
    // Example: FW0500; (500 Hz)
    // Minimum length: "FWxxxx" (6 chars), or "FWxxxx;" (7 chars)
    size_t len = strlen(response);
    if (strncmp(response, "FW", 2) == 0 && (len == 6 || (len == 7 && response[6] == ';'))) {
        if (isdigit((unsigned char)response[2]) && isdigit((unsigned char)response[3]) &&
            isdigit((unsigned char)response[4]) && isdigit((unsigned char)response[5])) {
            int32_t width_val_i32 = parse_int(response + 2, 4);
            if (width_val_i32 >= 0 && width_val_i32 <= 9999) {
                uint16_t width_val_u16 = (uint16_t) width_val_i32;
                ESP_LOGV(TAG, "Parsed FW command, width: %u Hz", width_val_u16);
                radio_subject_set_int_async(&radio_cw_bandwidth_subject, width_val_u16);
            } else {
                ESP_LOGW(TAG, "Parsed FW width out of range: %" PRIi32, width_val_i32);
            }
        } else {
            ESP_LOGW(TAG, "FW command has non-digit parameters: %s", response);
        }
    } else {
        ESP_LOGW(TAG, "Invalid FW command format or length: %s", response);
    }
}

static transverter_state_t g_transverter_state = {0};

// Helper functions for transverter offset calculations
bool transverter_is_enabled(void) {
    bool mix_enabled = ui_get_xvtr_offset_mix_enabled();
    bool xo_valid = g_transverter_state.xo_data.valid;
    bool uixd_enabled = ui_get_transverter_enabled();

    // If UIXD is enabled, ARCI firmware translates all frequencies (FA/FB/IF)
    // so we should NOT apply offset locally
    if (uixd_enabled) {
        return false;
    }

    // Only apply offset locally if "XVTR Offset Mix" toggle is explicitly enabled
    // and we have valid XO data (offset frequency and direction)
    if (mix_enabled && xo_valid) {
        return true;
    }

    // No offset mixing - neither UIXD nor manual override is active
    return false;
}

uint64_t apply_transverter_offset_for_display(uint64_t radio_frequency) {
    bool enabled = transverter_is_enabled();

    if (!enabled) {
        return radio_frequency;
    }

    if (g_transverter_state.xo_data.direction_plus) {
        // Plus direction: display_freq = radio_freq + offset
        return radio_frequency + g_transverter_state.xo_data.offset_frequency;
    } else {
        // Minus direction: display_freq = radio_freq - offset
        if (radio_frequency >= g_transverter_state.xo_data.offset_frequency) {
            return radio_frequency - g_transverter_state.xo_data.offset_frequency;
        } else {
            // Prevent underflow
            ESP_LOGW(TAG, "Transverter offset would cause frequency underflow");
            return 0;
        }
    }
}

uint64_t compute_radio_frequency_from_display(uint64_t display_frequency) {
    if (!transverter_is_enabled()) {
        return display_frequency;
    }
    
    if (g_transverter_state.xo_data.direction_plus) {
        // Plus direction: radio_freq = display_freq - offset
        if (display_frequency >= g_transverter_state.xo_data.offset_frequency) {
            return display_frequency - g_transverter_state.xo_data.offset_frequency;
        } else {
            // Prevent underflow
            ESP_LOGW(TAG, "Display frequency would cause radio frequency underflow");
            return 0;
        }
    } else {
        // Minus direction: radio_freq = display_freq + offset
        return display_frequency + g_transverter_state.xo_data.offset_frequency;
    }
}

transverter_state_t* get_transverter_state(void) {
    return &g_transverter_state;
}

// Request fresh VFO frequencies from radio when transverter state changes.
// Sends FA; and FB; commands to get current frequencies reflecting UIXD translation state.
void cat_request_transverter_display_refresh(void) {
    // Skip if UART not yet initialized (called during early UI observer setup)
    if (!uart_is_ready()) {
        return;
    }
    // When transverter state changes (UIXD toggle), cached frequencies are stale
    // because they were received under different ARCI translation conditions.
    // Request fresh FA/FB commands to get current frequencies from radio/ARCI.
    ESP_LOGI(TAG, "Requesting fresh VFO frequencies for transverter state change");
    uart_write_message("FA;");
    uart_write_message("FB;");
}

// Request fresh transverter configuration from radio
void cat_request_transverter_config_update(void) {
    ESP_LOGI(TAG, "Requesting fresh transverter configuration from radio");
    uart_write_message("XO;");
    uart_write_message("EX0560000;");
}

// Request filter mode settings from radio (EX028 for SSB, EX029 for SSB-DATA)
void cat_request_filter_mode_update(void) {
    ESP_LOGI(TAG, "Requesting filter mode settings from radio");
    uart_write_message("EX0280000;"); // SSB filter mode (Hi/Lo vs Width/Shift)
    uart_write_message("EX0290000;"); // SSB-DATA filter mode
}

// Request Radio Menu settings from radio
void cat_request_cw_menu_update(void) {
    if (!uart_is_ready()) {
        return;
    }
    ESP_LOGI(TAG, "Requesting Radio Menu settings from radio");
    uart_write_message("EX0060000;"); // Sidetone volume
    uart_write_message("EX0400000;"); // Side tone/pitch frequency
    uart_write_message("EX0530000;"); // FM mic gain
    uart_write_message("EX0710000;"); // USB audio input level
    uart_write_message("EX0720000;"); // USB audio output level
    uart_write_message("EX0730000;"); // ACC2 AF input level
    uart_write_message("EX0740000;"); // ACC2 AF output level
}

void parse_xo_command(const char *response) {
    // Command format: XOP1P2P2P2P2P2P2P2P2P2P2P2; 
    // P1: Direction (1 digit: 0=Plus, 1=Minus)
    // P2: Offset frequency (11 digits in Hz)
    // Example: XO014500000000; (Plus direction, 14.5 GHz offset)
    // Minimum length: "XO" + 1 + 11 = 14 chars, or 15 with semicolon
    
    if (!response) {
        ESP_LOGW(TAG, "XO command called with NULL response");
        return;
    }
    
    size_t len = strlen(response);
    ESP_LOGI(TAG, "XO RESPONSE RECEIVED: '%s' (length: %zu)", response, len);
    if (strncmp(response, "XO", 2) == 0 && (len == 14 || (len == 15 && response[14] == ';'))) {
        // Validate P1 (direction)
        char direction_char = response[2];
        if (direction_char != '0' && direction_char != '1') {
            ESP_LOGW(TAG, "Invalid XO direction parameter: %c", direction_char);
            return;
        }
        
        // Validate all P2 digits (frequency)
        bool all_digits = true;
        for (int i = 3; i < 14; i++) {
            if (!isdigit((unsigned char)response[i])) {
                all_digits = false;
                break;
            }
        }
        
        if (!all_digits) {
            ESP_LOGW(TAG, "XO command has non-digit frequency parameters: %s", response);
            return;
        }
        
        // Parse the values
        transverter_xo_data_t xo_data = {0};
        xo_data.direction_plus = (direction_char == '0');
        xo_data.offset_frequency = parse_long(response + 3, 11);
        xo_data.valid = true;
        
        // Update global state
        g_transverter_state.xo_data = xo_data;
        
        ESP_LOGV(TAG, "Parsed XO command: direction=%s, offset=%llu Hz",
                 xo_data.direction_plus ? "Plus" : "Minus", 
                 (unsigned long long)xo_data.offset_frequency);
        
        // Send individual XO data update
        radio_subject_notify_async(&radio_transverter_xo_subject);

        // Send combined state update
        radio_subject_notify_async(&radio_transverter_state_subject);

        // Immediately refresh displayed frequency using cached raw VFO-A/B, if available
        if (g_last_raw_vfo_a_hz > 0) {
            static uint32_t last_xo_freq_a = 0;
            const uint32_t FREQ_CHANGE_THRESHOLD = 2; // Lower threshold for faster display updates
            uint64_t display_frequency_a = apply_transverter_offset_for_display((uint64_t) g_last_raw_vfo_a_hz);
            uint32_t display_freq_a_u32 = (uint32_t) display_frequency_a;
            ESP_LOGV(TAG, "XO change: VFO-A raw=%lu Hz, display=%lu Hz, mix_enabled=%d",
                     g_last_raw_vfo_a_hz, display_freq_a_u32, ui_get_xvtr_offset_mix_enabled());
            if (abs((int32_t)display_freq_a_u32 - (int32_t)last_xo_freq_a) >= FREQ_CHANGE_THRESHOLD) {
                last_xo_freq_a = display_freq_a_u32;
            }
        }
        if (g_last_raw_vfo_b_hz > 0) {
            static uint32_t last_xo_freq_b = 0;
            const uint32_t FREQ_CHANGE_THRESHOLD = 2; // Lower threshold for faster display updates
            uint64_t display_frequency_b = apply_transverter_offset_for_display((uint64_t) g_last_raw_vfo_b_hz);
            uint32_t display_freq_b_u32 = (uint32_t) display_frequency_b;
            if (abs((int32_t)display_freq_b_u32 - (int32_t)last_xo_freq_b) >= FREQ_CHANGE_THRESHOLD) {
                last_xo_freq_b = display_freq_b_u32;
            }
        }
    } else {
        ESP_LOGW(TAG, "Invalid XO command format or length: %s", response);
    }
}

// Handle EX028: SSB filter mode (Hi/Lo Cut vs Width/Shift)
static void parse_ex028_filter_mode(int value) {
    if (value != 0 && value != 1) {
        ESP_LOGW(TAG, "Invalid EX028 value: %d (expected 0 or 1)", value);
        return;
    }

    uint8_t mode = (uint8_t)value;
    radio_set_ssb_filter_mode(mode);

    ESP_LOGI(TAG, "EX028 SSB filter mode: %s", mode == 0 ? "Hi/Lo Cut" : "Width/Shift");
}

// Handle EX029: SSB-DATA filter mode (Hi/Lo Cut vs Width/Shift)
static void parse_ex029_filter_mode(int value) {
    if (value != 0 && value != 1) {
        ESP_LOGW(TAG, "Invalid EX029 value: %d (expected 0 or 1)", value);
        return;
    }

    uint8_t mode = (uint8_t)value;
    radio_set_ssb_data_filter_mode(mode);

    ESP_LOGI(TAG, "EX029 SSB-DATA filter mode: %s", mode == 0 ? "Hi/Lo Cut" : "Width/Shift");
}

// Handle EX006: CW sidetone volume (0=OFF, 1-9)
static void parse_ex006_sidetone_volume(int value) {
    if (value < 0 || value > 9) {
        ESP_LOGW(TAG, "Invalid EX006 value: %d (expected 0-9)", value);
        return;
    }

    radio_subject_set_int_async(&radio_cw_sidetone_volume_subject, value);
    ESP_LOGD(TAG, "EX006 CW sidetone volume: %d", value);
}

// Handle EX040: CW TX pitch / sidetone frequency (Hz, 300-1000 step 50)
static void parse_ex040_sidetone_pitch(int value) {
    if (value < 300 || value > 1000 || (value % 50) != 0) {
        ESP_LOGW(TAG, "Invalid EX040 value: %d (expected 300-1000 in 50Hz steps)", value);
        return;
    }

    radio_subject_set_int_async(&radio_cw_pitch_hz_subject, value);
    ESP_LOGD(TAG, "EX040 CW pitch: %d Hz", value);
}

// Handle EX053: FM mic gain (1-3)
static void parse_ex053_fm_mic_gain(int value) {
    if (value < 1 || value > 3) {
        ESP_LOGW(TAG, "Invalid EX053 value: %d (expected 1-3)", value);
        return;
    }

    radio_subject_set_int_async(&radio_fm_mic_gain_subject, value);
    ESP_LOGD(TAG, "EX053 FM mic gain: %d", value);
}

// Handle EX071: USB audio input level (0-9)
static void parse_ex071_usb_input_level(int value) {
    if (value < 0 || value > 9) {
        ESP_LOGW(TAG, "Invalid EX071 value: %d (expected 0-9)", value);
        return;
    }

    radio_subject_set_int_async(&radio_usb_input_level_subject, value);
    ESP_LOGD(TAG, "EX071 USB audio input level: %d", value);
}

// Handle EX072: USB audio output level (0-9)
static void parse_ex072_usb_output_level(int value) {
    if (value < 0 || value > 9) {
        ESP_LOGW(TAG, "Invalid EX072 value: %d (expected 0-9)", value);
        return;
    }

    radio_subject_set_int_async(&radio_usb_output_level_subject, value);
    ESP_LOGD(TAG, "EX072 USB audio output level: %d", value);
}

// Handle EX073: ACC2 AF input level (0-9)
static void parse_ex073_acc2_input_level(int value) {
    if (value < 0 || value > 9) {
        ESP_LOGW(TAG, "Invalid EX073 value: %d (expected 0-9)", value);
        return;
    }

    radio_subject_set_int_async(&radio_acc2_input_level_subject, value);
    ESP_LOGD(TAG, "EX073 ACC2 AF input level: %d", value);
}

// Handle EX074: ACC2 AF output level (0-9)
static void parse_ex074_acc2_output_level(int value) {
    if (value < 0 || value > 9) {
        ESP_LOGW(TAG, "Invalid EX074 value: %d (expected 0-9)", value);
        return;
    }

    radio_subject_set_int_async(&radio_acc2_output_level_subject, value);
    ESP_LOGD(TAG, "EX074 ACC2 AF output level: %d", value);
}

// Handle EX056: Transverter settings
static void parse_ex056_transverter(int value) {
    transverter_ex056_data_t ex056_data = {0};

    switch (value) {
        case 0:
            ex056_data.enabled = false;
            ex056_data.power_down_mode = false;
            break;
        case 1:
            ex056_data.enabled = true;
            ex056_data.power_down_mode = false;
            break;
        case 2:
            ex056_data.enabled = true;
            ex056_data.power_down_mode = true;
            break;
        default:
            ESP_LOGW(TAG, "Invalid EX056 value: %d", value);
            return;
    }

    ex056_data.valid = true;
    g_transverter_state.ex056_data = ex056_data;

    ESP_LOGV(TAG, "Parsed EX056: enabled=%s, power_down=%s",
             ex056_data.enabled ? "true" : "false",
             ex056_data.power_down_mode ? "true" : "false");

    radio_subject_notify_async(&radio_transverter_ex056_subject);
    radio_subject_notify_async(&radio_transverter_state_subject);

    // Refresh displayed frequency using cached raw VFO-A/B
    if (g_last_raw_vfo_a_hz > 0) {
        static uint32_t last_ex056_freq_a = 0;
        const uint32_t FREQ_CHANGE_THRESHOLD = 2;
        uint64_t display_frequency_a = apply_transverter_offset_for_display((uint64_t) g_last_raw_vfo_a_hz);
        uint32_t display_freq_a_u32 = (uint32_t) display_frequency_a;
        if (abs((int32_t)display_freq_a_u32 - (int32_t)last_ex056_freq_a) >= FREQ_CHANGE_THRESHOLD) {
            last_ex056_freq_a = display_freq_a_u32;
        }
    }
    if (g_last_raw_vfo_b_hz > 0) {
        static uint32_t last_ex056_freq_b = 0;
        const uint32_t FREQ_CHANGE_THRESHOLD = 2;
        uint64_t display_frequency_b = apply_transverter_offset_for_display((uint64_t) g_last_raw_vfo_b_hz);
        uint32_t display_freq_b_u32 = (uint32_t) display_frequency_b;
        if (abs((int32_t)display_freq_b_u32 - (int32_t)last_ex056_freq_b) >= FREQ_CHANGE_THRESHOLD) {
            last_ex056_freq_b = display_freq_b_u32;
        }
    }
}

void parse_ex_command(const char *response) {
    // Command format: EX[menu][0000][value];
    // Examples: EX02800001; (EX028 value 1), EX05600002; (EX056 value 2)
    // Format: "EX" + 3-digit menu + "0000" + value (1+ digits)

    if (!response) {
        ESP_LOGW(TAG, "EX command called with NULL response");
        return;
    }

    size_t len = strlen(response);
    ESP_LOGD(TAG, "EX RESPONSE: '%s' (len: %zu)", response, len);

    if (len < 2 || strncmp(response, "EX", 2) != 0) {
        return;
    }

    // Minimum: "EX" + 3-digit menu + "0000" + 1-digit value = 10 chars (+ optional semicolon)
    if (len < 10) {
        ESP_LOGD(TAG, "EX command too short: %zu chars", len);
        return;
    }

    // Parse 3-digit menu number at positions 2-4
    if (!isdigit((unsigned char)response[2]) ||
        !isdigit((unsigned char)response[3]) ||
        !isdigit((unsigned char)response[4])) {
        ESP_LOGW(TAG, "EX command invalid menu number: %s", response);
        return;
    }
    int menu_num = (response[2] - '0') * 100 + (response[3] - '0') * 10 + (response[4] - '0');

    // Validate "0000" padding at positions 5-8
    if (strncmp(response + 5, "0000", 4) != 0) {
        ESP_LOGW(TAG, "EX command missing 0000 padding: %s", response);
        return;
    }

    // Parse value starting at position 9
    if (len < 10 || !isdigit((unsigned char)response[9])) {
        ESP_LOGW(TAG, "EX command missing or invalid value: %s", response);
        return;
    }

    int value = response[9] - '0';
    // Handle multi-digit values if present
    for (size_t i = 10; i < len && isdigit((unsigned char)response[i]); i++) {
        value = value * 10 + (response[i] - '0');
    }

    // Dispatch to specific menu handlers
    switch (menu_num) {
        case 6:
            parse_ex006_sidetone_volume(value);
            break;
        case 28:
            parse_ex028_filter_mode(value);
            break;
        case 29:
            parse_ex029_filter_mode(value);
            break;
        case 40:
            parse_ex040_sidetone_pitch(value);
            break;
        case 53:
            parse_ex053_fm_mic_gain(value);
            break;
        case 56:
            parse_ex056_transverter(value);
            break;
        case 71:
            parse_ex071_usb_input_level(value);
            break;
        case 72:
            parse_ex072_usb_output_level(value);
            break;
        case 73:
            parse_ex073_acc2_input_level(value);
            break;
        case 74:
            parse_ex074_acc2_output_level(value);
            break;
        default:
            ESP_LOGD(TAG, "EX%03d not handled, value=%d", menu_num, value);
            break;
    }
}

// Getter for memory channel data
memory_channel_data_t* get_memory_channel_data(void) {
    return &g_memory_channel_data;
}

void parse_mr_command(const char *response) {
    // MR command format (answer):
    // MRP1P2P3P3P4...P4P5P6P7P8P8P9P9P10P10P10P11P12P13...P13P14P14P15P16...P16;
    // P1: Split flag (1 digit: 0=Simplex, 1=Split)
    // P2: Memory channel hundreds digit (1 digit)
    // P3: Memory channel last two digits (2 digits)
    // P4: Frequency (11 digits)
    // P5: Mode (1 digit)
    // P6: Data mode (1 digit)
    // P7: Tone mode (1 digit)
    // P8: Tone freq index (2 digits)
    // P9: CTCSS freq index (2 digits)
    // P10: Always 000 (3 digits)
    // P11: Filter A/B (1 digit)
    // P12: Always 0 (1 digit)
    // P13: Always 000000000 (9 digits)
    // P14: FM width (2 digits)
    // P15: Channel lockout (1 digit)
    // P16: Memory name (variable, up to 8 ASCII chars)
    //
    // Minimum fixed portion: 2 (MR) + 1 + 1 + 2 + 11 + 1 + 1 + 1 + 2 + 2 + 3 + 1 + 1 + 9 + 2 + 1 = 41 chars
    // Plus optional name (0-8 chars) + optional semicolon

    if (!response) {
        ESP_LOGW(TAG, "MR command called with NULL response");
        return;
    }

    size_t len = strlen(response);
    ESP_LOGD(TAG, "MR RESPONSE: '%s' (len: %zu)", response, len);

    // Validate prefix
    if (len < 2 || strncmp(response, "MR", 2) != 0) {
        return;
    }

    // Minimum length check (41 chars for fixed portion)
    if (len < 41) {
        ESP_LOGW(TAG, "MR command too short: %zu chars (need >= 41)", len);
        return;
    }

    // Parse fields - positions relative to start of response
    // P1: Split flag at position 2
    char split_char = response[2];
    if (split_char != '0' && split_char != '1') {
        ESP_LOGW(TAG, "MR invalid split flag: %c", split_char);
        return;
    }
    bool is_split = (split_char == '1');

    // P2: Hundreds digit at position 3
    if (!isdigit((unsigned char)response[3])) {
        ESP_LOGW(TAG, "MR invalid channel hundreds digit");
        return;
    }
    int hundreds = response[3] - '0';

    // P3: Last two digits at positions 4-5
    if (!isdigit((unsigned char)response[4]) || !isdigit((unsigned char)response[5])) {
        ESP_LOGW(TAG, "MR invalid channel last two digits");
        return;
    }
    int channel = hundreds * 100 + (response[4] - '0') * 10 + (response[5] - '0');

    // Validate all 11 frequency digits (P4) at positions 6-16
    for (int i = 6; i < 17; i++) {
        if (!isdigit((unsigned char)response[i])) {
            ESP_LOGW(TAG, "MR invalid frequency digit at position %d", i);
            return;
        }
    }
    uint32_t frequency = (uint32_t)parse_long(response + 6, 11);

    // P5: Mode at position 17
    if (!isdigit((unsigned char)response[17])) {
        ESP_LOGW(TAG, "MR invalid mode digit");
        return;
    }
    uint8_t mode = response[17] - '0';

    // P6: Data mode at position 18
    if (!isdigit((unsigned char)response[18])) {
        ESP_LOGW(TAG, "MR invalid data mode digit");
        return;
    }
    bool data_mode = (response[18] == '1');

    // P7: Tone mode at position 19
    if (!isdigit((unsigned char)response[19])) {
        ESP_LOGW(TAG, "MR invalid tone mode digit");
        return;
    }
    uint8_t tone_mode = response[19] - '0';

    // P8: Tone freq index at positions 20-21
    uint8_t tone_freq_index = (uint8_t)parse_int(response + 20, 2);

    // P9: CTCSS freq index at positions 22-23
    uint8_t ctcss_freq_index = (uint8_t)parse_int(response + 22, 2);

    // P10: Skip 000 at positions 24-26
    // P11: Filter at position 27
    uint8_t filter = 0;
    if (isdigit((unsigned char)response[27])) {
        filter = response[27] - '0';
    }

    // P12: Skip 0 at position 28
    // P13: Skip 000000000 at positions 29-37
    // P14: FM width at positions 38-39
    bool fm_narrow = false;
    if (len >= 40) {
        fm_narrow = (response[39] == '1');  // 01 = narrow
    }

    // P15: Lockout at position 40
    bool lockout = false;
    if (len >= 41) {
        lockout = (response[40] == '1');
    }

    // P16: Memory name starting at position 41
    char name[9] = {0};
    if (len > 41) {
        size_t name_start = 41;
        size_t name_len = 0;
        // Copy up to 8 characters, stopping at semicolon or end
        for (size_t i = name_start; i < len && name_len < 8; i++) {
            if (response[i] == ';') break;
            name[name_len++] = response[i];
        }
        name[name_len] = '\0';
        // Trim trailing spaces
        while (name_len > 0 && name[name_len - 1] == ' ') {
            name[--name_len] = '\0';
        }
    }

    // Update global memory channel data
    g_memory_channel_data.channel = (uint16_t)channel;
    g_memory_channel_data.frequency = frequency;
    g_memory_channel_data.mode = mode;
    g_memory_channel_data.data_mode = data_mode;
    g_memory_channel_data.tone_mode = tone_mode;
    g_memory_channel_data.tone_freq_index = tone_freq_index;
    g_memory_channel_data.ctcss_freq_index = ctcss_freq_index;
    g_memory_channel_data.filter = filter;
    g_memory_channel_data.fm_narrow = fm_narrow;
    g_memory_channel_data.lockout = lockout;
    g_memory_channel_data.split = is_split;
    strncpy(g_memory_channel_data.name, name, sizeof(g_memory_channel_data.name) - 1);
    g_memory_channel_data.name[sizeof(g_memory_channel_data.name) - 1] = '\0';
    g_memory_channel_data.valid = true;

    ESP_LOGI(TAG, "Parsed MR: ch=%d freq=%lu mode=%d name='%s'",
             channel, (unsigned long)frequency, mode, name);

    // Copy data to subject buffer and notify observers
    memory_channel_data_t* subject_buf = radio_get_memory_channel_buffer();
    if (subject_buf) {
        *subject_buf = g_memory_channel_data;
        radio_subject_notify_async(&radio_memory_channel_subject);
    }
}

#define CAT_ACTIVITY_MONITOR_INTERVAL_MS (180 * 1000) // 3 minutes
static TimerHandle_t cat_activity_monitor_timer = NULL;

// FreeRTOS timer callback for monitoring CAT activity
static void cat_activity_monitor_timer_cb(TimerHandle_t xTimer) {
    LV_UNUSED(xTimer);
    if (g_cat_data_activity_flag) {
        ESP_LOGD(TAG, "CAT activity detected by monitor timer");
        g_cat_data_activity_flag = false; // Reset the flag
    } else {
        ESP_LOGD(TAG, "No CAT activity detected by monitor timer since last check.");
    }
}

// Initialize the CAT activity monitoring timer
void cat_parser_init_activity_monitor(void) {
    if (cat_activity_monitor_timer == NULL) {
        cat_activity_monitor_timer = xTimerCreate(
            "catActivityMonTimer",
            pdMS_TO_TICKS(CAT_ACTIVITY_MONITOR_INTERVAL_MS),
            pdTRUE, // Auto-reload timer
            (void *)0, // Timer ID, not used
            cat_activity_monitor_timer_cb);

        if (cat_activity_monitor_timer != NULL) {
            if (xTimerStart(cat_activity_monitor_timer, 0) == pdPASS) {
                ESP_LOGI(TAG, "CAT activity monitor timer started (interval: %d ms).", CAT_ACTIVITY_MONITOR_INTERVAL_MS);
            } else {
                ESP_LOGE(TAG, "Failed to start CAT activity monitor timer.");
                xTimerDelete(cat_activity_monitor_timer, 0); // Clean up if start fails
                cat_activity_monitor_timer = NULL;
            }
        } else {
            ESP_LOGE(TAG, "Failed to create CAT activity monitor timer.");
        }
    } else {
        ESP_LOGW(TAG, "CAT activity monitor timer already initialized.");
    }
}

// Function to check and reset the CAT activity flag
bool cat_check_and_reset_activity_flag(void) {
    bool activity_detected = g_cat_data_activity_flag;
    g_cat_data_activity_flag = false; // Reset the flag
    return activity_detected;
}

#define CAT_LOG_LEVEL ESP_LOG_INFO  // Change to ESP_LOG_DEBUG to reduce logging in production

// Fast hash function for two-character command prefix
// ReSharper disable once CppRedundantInlineSpecifier
static inline uint16_t cmd_hash(const char *cmd) {
    if (!cmd || cmd[0] == '\0' || cmd[1] == '\0') return 0;
    return ((uint16_t) cmd[0] << 8) | cmd[1];
}

#include "freertos/task.h" // Required for uxTaskGetStackHighWaterMark and pcTaskGetName
#include "ui/screens/ui_Screen2.h" // For UI toggle state

// Macro UI functions are declared in ui_Screen2.h

/**
 * @brief Parse MX* macro response commands
 * Handles MXR (read macro), MXA (assignment query/set), MXW (write ack), MXD (delete ack)
 * Protocol: MX<subcmd><payload>; with | as command separator within macros
 */
static void parse_mx_command(const char *response) {
    if (!response || strlen(response) < 3) {
        ESP_LOGW(TAG, "Invalid MX command: too short");
        return;
    }

    char subcmd = response[2];  // MXR, MXA, MXW, MXD, MXE

    if (subcmd == 'R') {
        // MXR<ID>,<name>,<cmd>|<cmd>|...;
        // Example: MXR01,20M FT8,FA00014074000|MD2|DA1|PR0;
        const char *payload = response + 3;  // Skip "MXR"

        // Parse ID (2 digits)
        if (strlen(payload) < 2) {
            ESP_LOGW(TAG, "MXR: ID too short");
            return;
        }

        uint8_t id = (uint8_t)((payload[0] - '0') * 10 + (payload[1] - '0'));
        if (id < 1 || id > 50) {
            ESP_LOGW(TAG, "MXR: Invalid ID %d", id);
            return;
        }

        // Find first comma (separates ID from name)
        const char *comma1 = strchr(payload, ',');
        if (!comma1) {
            ESP_LOGW(TAG, "MXR: Missing first comma");
            return;
        }

        // Find second comma (separates name from commands)
        const char *comma2 = strchr(comma1 + 1, ',');
        if (!comma2) {
            ESP_LOGW(TAG, "MXR: Missing second comma");
            return;
        }

        // Find end of commands - either ';' or end of string (framing may strip ';')
        const char *end = strchr(comma2 + 1, ';');
        if (!end) {
            end = comma2 + 1 + strlen(comma2 + 1);  // Use end of string
        }

        // Extract name (between comma1 and comma2)
        size_t name_len = comma2 - comma1 - 1;
        char name[33];
        if (name_len >= 33) name_len = 32;
        strncpy(name, comma1 + 1, name_len);
        name[name_len] = '\0';

        // Extract commands (between comma2 and end)
        size_t cmd_len = end - comma2 - 1;
        char commands[65];
        if (cmd_len >= 65) cmd_len = 64;
        strncpy(commands, comma2 + 1, cmd_len);
        commands[cmd_len] = '\0';

        // Skip empty/undefined macros (no name means slot is empty)
        if (name[0] == '\0') {
            ESP_LOGD(TAG, "MXR: Skipping empty macro slot %d", id);
        } else {
            // Update UI cache and request debounced refresh
            ui_macro_set_cached(id, name, commands);
            ui_macro_request_refresh();
            ESP_LOGI(TAG, "MXR: Cached macro %d (%s)", id, name);
        }

    } else if (subcmd == 'A') {
        // MXA query response: MXA01,02,03,04,05,00; (6 F-key assignments, 2-digit each)
        // MXA assign ack: MXA<slot>,<id>; (single digit slot 1-6, then comma, then 2-digit id)
        const char *payload = response + 3;  // Skip "MXA"

        // Detect format: assignment ack has single digit (1-6) followed by comma
        if (payload[0] >= '1' && payload[0] <= '6' && payload[1] == ',') {
            // Assignment acknowledgment: MXA<slot>,<id>
            uint8_t slot = payload[0] - '0';
            uint8_t macro_id = 0;
            if (payload[2] >= '0' && payload[2] <= '9' &&
                payload[3] >= '0' && payload[3] <= '9') {
                macro_id = (uint8_t)((payload[2] - '0') * 10 + (payload[3] - '0'));
            }
            ui_macro_set_fkey_assignment(slot, macro_id);
            ESP_LOGI(TAG, "MXA: F%d assigned to macro %d", slot, macro_id);
        } else {
            // Query response: MXA01,02,03,04,05,00 (6 comma-separated 2-digit IDs)
            ESP_LOGI(TAG, "MXA: F-key assignments: %s", payload);

            // Parse comma-separated macro IDs for F1-F6
            uint8_t fkey = 1;
            const char *p = payload;
            while (*p && fkey <= 6) {
                // Parse 2-digit macro ID
                if (p[0] >= '0' && p[0] <= '9' && p[1] >= '0' && p[1] <= '9') {
                    uint8_t macro_id = (uint8_t)((p[0] - '0') * 10 + (p[1] - '0'));
                    ui_macro_set_fkey_assignment(fkey, macro_id);
                    ESP_LOGD(TAG, "MXA: F%d = macro %d", fkey, macro_id);
                    p += 2;
                }

                // Skip comma or find end
                if (*p == ',') {
                    p++;
                    fkey++;
                } else if (*p == ';' || *p == '\0') {
                    break;
                } else {
                    p++;  // Skip unexpected character
                }
            }
        }
        ui_macro_request_refresh();

    } else if (subcmd == 'W') {
        // MXW<ID>; - Save acknowledgment
        ui_macro_refresh_list();
        ESP_LOGI(TAG, "MXW: Macro saved, refreshing list");

    } else if (subcmd == 'D') {
        // MXD<ID>; - Delete acknowledgment
        ui_macro_refresh_list();
        ESP_LOGI(TAG, "MXD: Macro deleted, refreshing list");

    } else if (subcmd == 'E') {
        // MXE<slot>; - Execute acknowledgment (no action needed on display)
        ESP_LOGD(TAG, "MXE: Macro executed on panel");

    } else {
        ESP_LOGW(TAG, "Unknown MX subcommand: %c", subcmd);
    }
}

void parse_cat_command(const char *response) {
    // Reset watchdog since this function can take time with complex commands
    esp_task_wdt_reset();
    
    ESP_LOGD(TAG, "Stack HWM for %s (task: %s): %u bytes", __func__, pcTaskGetName(NULL),
             uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGV(TAG, "Processing CAT command: %s", response);
    // heap_caps_check_integrity_all(true); // Check heap at entry of main parser (REMOVED FOR PERFORMANCE)
    if (!response || response[0] == '\0' || response[1] == '\0') {
        return; // Invalid response, too short
    }

    // Set a flag indicating CAT data activity
    g_cat_data_activity_flag = true;
    
    // Mark CAT activity for polling manager
    cat_polling_mark_activity();

    // Special case for IF command which may be embedded in other text
    if (response[0] == 'I' && response[1] == 'F') {
        parse_if_command(response);
        return;
    }
    for (size_t i = 1; response[i] != '\0'; ++i) {
        if (response[i - 1] == 'I' && response[i] == 'F') {
            parse_if_command(&response[i - 1]);
            return;
        }
    }

    // Fast command dispatch using hash of first two characters
    uint16_t hash = cmd_hash(response);

    switch (hash) {
        case ((uint16_t) 'F' << 8) | 'A': // FA
            parse_fa_frequency(response);
            break;
        case ((uint16_t) 'F' << 8) | 'B': // FB
            parse_fb_frequency(response);
            break;
        case ((uint16_t) 'F' << 8) | 'R': // FR
        case ((uint16_t) 'F' << 8) | 'T': // FT
            parse_fr_ft_command(response);
            break;
        case ((uint16_t) 'M' << 8) | 'D': // MD
            parse_mode(response);
            break;
        case ((uint16_t) 'P' << 8) | 'S': // PS
            parse_ps_status(response);
            break;
        case ((uint16_t) 'F' << 8) | 'L': // FL
            parse_filter(response);
            break;
        case ((uint16_t) 'R' << 8) | 'T': // RT
            parse_rit_status(response);
            break;
        case ((uint16_t) 'R' << 8) | 'U': // RU
            parse_rit_frequency(response);
            break;
        case ((uint16_t) 'X' << 8) | 'T': // XT
            parse_xit_status(response);
            break;
        case ((uint16_t) 'X' << 8) | 'U': // XU
            parse_xit_frequency(response);
            break;
        case ((uint16_t) 'S' << 8) | 'P': // SP - split frequency operation (NOT split mode)
            parse_split_operation_status(response);
            break;
        case ((uint16_t) 'G' << 8) | 'C': // GC - AGC mode
            parse_agc_mode(response);
            break;
        case ((uint16_t) 'G' << 8) | 'T': // GT - AGC time constant
            parse_agc_time(response);
            break;
        case ((uint16_t) 'P' << 8) | 'C': // PC
            parse_power(response);
            break;
        case ((uint16_t) 'P' << 8) | 'A': // PA
            parse_preamp(response);
            break;
        case ((uint16_t) 'P' << 8) | 'R': // PR
            parse_processor(response);
            break;
        case ((uint16_t) 'R' << 8) | 'A': // RA
            parse_att(response);
            break;
        case ((uint16_t) 'N' << 8) | 'B': // NB
            ESP_LOGI(TAG, "Dispatching NB command: \"%s\"", response);
            parse_nb_status(response);
            break;
        case ((uint16_t) 'N' << 8) | 'R': // NR
            parse_nr_status(response);
            break;
        case ((uint16_t) 'B' << 8) | 'C': // BC
            parse_bc_status(response);
            break;
        case ((uint16_t) 'N' << 8) | 'T': // NT
            parse_notch_status(response);
            break;
        case ((uint16_t) 'B' << 8) | 'P': // BP
            parse_notch_frequency(response);
            break;
        case ((uint16_t) 'K' << 8) | 'S': // KS
            parse_cw_speed(response);
            break;
        case ((uint16_t) 'V' << 8) | 'X': // VX
            parse_vox_status(response);
            break;
        case ((uint16_t) 'V' << 8) | 'G': // VG
            parse_vox_gain(response);
            break;
        case ((uint16_t) 'R' << 8) | 'I': // RI
            parse_ri_command(response);
            break;
        case ((uint16_t) 'R' << 8) | 'X': // RX
            parse_rx_command(response);
            break;
        case ((uint16_t) 'R' << 8) | 'M': // RM
            parse_rm_command(response);
            break;
        case ((uint16_t) 'S' << 8) | 'M': // SM
            parse_sm_command(response);
            break;
        case ((uint16_t) 'D' << 8) | 'A': // DA
            parse_data_command(response);
            break;
        case ((uint16_t) 'T' << 8) | 'S': // TS
            parse_ts_command(response);
            break;
        case ((uint16_t) 'T' << 8) | 'X': // TX
            parse_tx_command(response);
            break;
        case ((uint16_t) 'A' << 8) | 'C': // AC
            parse_ac_command(response);
            break;
        case ((uint16_t) 'A' << 8) | 'I': // AI
            parse_ai_command(response);
            break;
        case ((uint16_t) 'A' << 8) | 'N': // AN
            parse_an_command(response);
            break;
        case ((uint16_t) 'A' << 8) | 'G': // AG
            parse_af_gain(response);
            break;
        case ((uint16_t) 'R' << 8) | 'G': // RG
            parse_rf_gain(response);
            break;
        case ((uint16_t) 'X' << 8) | 'I': // XI
            parse_xi_command(response);
            break;
        case ((uint16_t) 'S' << 8) | 'H': // SH
            ESP_LOGI(TAG, "Dispatching SH command: \"%s\"", response);
            parse_sh_sl(response, true); // true for isHigh
            break;
        case ((uint16_t) 'S' << 8) | 'L': // SL
            ESP_LOGI(TAG, "Dispatching SL command: \"%s\"", response);
            parse_sh_sl(response, false); // false for isHigh (low-cut)
            break;
        case ((uint16_t) 'F' << 8) | 'W': // FW
            parse_fw_command(response);
            break;
        case ((uint16_t) 'X' << 8) | 'O': // XO
            parse_xo_command(response);
            break;
        case ((uint16_t) 'E' << 8) | 'X': // EX
            parse_ex_command(response);
            break;
        case ((uint16_t) 'M' << 8) | 'R': // MR (Memory Read)
            parse_mr_command(response);
            break;

        // ========== UI Meta Commands (panel-display only) ==========
        case ((uint16_t) 'U' << 8) | 'I': // UI* commands
            // Handle UI meta commands - check third character
            if (response[2] == 'P' && response[3] == 'C') {
                // UIPC - Power Control
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIPC received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_POWER) {
                    ui_power_popup_set_value(value);
                } else if (value >= 5 && value <= 100) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_POWER, value);
                }
            } else if (response[2] == 'M' && response[3] == 'L') {
                // UIML - TX Monitor Level
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIML received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_CARRIER_LEVEL) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 20) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_CARRIER_LEVEL, value);
                }
            } else if (response[2] == 'C' && response[3] == 'G') {
                // UICG - CW Carrier Level (0-100%)
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UICG received: value=%d%%", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_CW_CARRIER_LEVEL) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 100) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_CW_CARRIER_LEVEL, value);
                }
            } else if (response[2] == 'R' && response[3] == 'L') {
                // UIRL - NR1 Level
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIRL received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_NR_LEVEL) {
                    ui_power_popup_set_value(value);
                } else if (value >= 1 && value <= 10) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_NR_LEVEL, value);
                }
            } else if (response[2] == 'R' && response[3] == 'S') {
                // UIRS - NR2 SPAC Speed
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIRS received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_NR2_SPEED) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 9) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_NR2_SPEED, value);
                }
            } else if (response[2] == 'N' && response[3] == 'L') {
                // UINL - NB Level
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UINL received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_NB_LEVEL) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 10) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_NB_LEVEL, value);
                }
            } else if (response[2] == 'P' && response[3] == 'I') {
                // UIPI - Processor Input Level
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIPI received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_PROC_INPUT_LEVEL) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 100) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_PROC_INPUT_LEVEL, value);
                }
            } else if (response[2] == 'P' && response[3] == 'O') {
                // UIPO - Processor Output Level
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIPO received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_PROC_OUTPUT_LEVEL) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 100) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_PROC_OUTPUT_LEVEL, value);
                }
            } else if (response[2] == 'D' && response[3] == 'A') {
                // UIDA - Data Mode
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIDA received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_DATA_MODE) {
                    ui_power_popup_set_value(value);
                } else if (value == 0 || value == 1) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_DATA_MODE, value);
                }
            } else if (response[2] == 'N' && response[3] == 'F') {
                // UINF - Notch Frequency
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UINF received: value=%d", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_NOTCH_FREQUENCY) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 127) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_NOTCH_FREQUENCY, value);
                }
            } else if (response[2] == 'I' && response[3] == 'S') {
                // UIIS - IF Shift (0-9999 Hz)
                int value = atoi(response + 4);
                ESP_LOGI(TAG, "UIIS received: value=%d Hz", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_IF_SHIFT) {
                    ui_power_popup_set_value(value);
                } else if (value >= 0 && value <= 9999) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_IF_SHIFT, value);
                }
            } else if (response[2] == 'R' && response[3] == 'I') {
                // UIRI - RIT/XIT Offset (-9999 to +9999 Hz)
                // Format: UIRI+NNNN; or UIRI-NNNN;
                int value = atoi(response + 4);  // atoi handles signed values
                ESP_LOGI(TAG, "UIRI received: value=%+d Hz", value);
                if (ui_power_popup_is_visible() &&
                    ui_power_popup_get_type() == UI_CONTROL_RIT_XIT_OFFSET) {
                    ui_power_popup_set_value(value);
                } else if (value >= -9999 && value <= 9999) {
                    // Show popup if not visible
                    ui_power_popup_show(UI_CONTROL_RIT_XIT_OFFSET, value);
                }
            } else if (response[2] == 'B' && response[3] == 'L') {
                // UIBL - Backlight control (query: UIBL; | set: UIBL000–UIBL255;)
                const char *payload = response + 4;
                if (payload[0] == '\0' || payload[0] == ';') {
                    uint8_t level = lcd_get_backlight_level();
                    char reply[12];
                    snprintf(reply, sizeof(reply), "UIBL%03u;", level);
                    (void) uart_write_raw(reply, strlen(reply));
                    ESP_LOGI(TAG, "UIBL query answered: %s", reply);
                } else {
                    size_t len = strlen(payload);
                    if (len > 0 && payload[len - 1] == ';') {
                        len--;
                    }
                    if (len == 0 || len > 3) {
                        ESP_LOGW(TAG, "UIBL invalid length: %s", response);
                    } else if (!isdigit((unsigned char)payload[0]) ||
                               (len > 1 && !isdigit((unsigned char)payload[1])) ||
                               (len > 2 && !isdigit((unsigned char)payload[2]))) {
                        ESP_LOGW(TAG, "UIBL invalid digits: %s", response);
                    } else {
                        int level = (int) parse_int(payload, (int) len);
                        if (level < 0 || level > 255) {
                            ESP_LOGW(TAG, "UIBL out of range: %d", level);
                        } else {
                            uint8_t level_u8 = (uint8_t) level;
                            lcd_set_backlight_level(level_u8);
                            screensaver_update_backlight(level_u8);
                            ESP_LOGI(TAG, "UIBL set level: %d", level);
                        }
                    }
                }
            } else if (response[2] == 'M' && response[3] == 'N') {
                // UIMN - Menu State (0=hide, 1=show)
                int state = atoi(response + 4);
                ESP_LOGI(TAG, "UIMN received: state=%d", state);
                if (state == 0) {
                    ui_power_popup_hide();
                }
                // UIMN1 alone doesn't show - need UIPC/UIML/UIRL/UIRS/UINL/UIPI/UIPO/UINF with value
            } else if (response[2] == 'X' && response[3] == 'D') {
                // UIXD - Transverter Enabled (ARCI custom command)
                // UIXD0; = disable, UIXD1; = enable
                // When enabled, ARCI presents frequency already mixed with transverter offset
                int state = atoi(response + 4);
                ESP_LOGI(TAG, "UIXD received: state=%d", state);

                // Update transverter toggle state via subject (thread-safe)
                radio_subject_set_int_async(&radio_transverter_enabled_subject, state);
            } else if (response[2] == 'P' && response[3] == 'S') {
                // UIPS - Panel Status (wake/sleep signaling from remote panel)
                // UIPS;  = query current state
                // UIPS0; = panel idle (optional, display manages own timeout)
                // UIPS1; = panel activity detected, wake screensaver and reset idle timer
                const char *payload = response + 4;
                if (payload[0] == '\0' || payload[0] == ';') {
                    // Query: respond with current wake state
                    // UIPS0; = screensaver active (asleep)
                    // UIPS1; = display awake
                    bool is_awake = !screensaver_is_active();
                    char reply[8];
                    snprintf(reply, sizeof(reply), "UIPS%d;", is_awake ? 1 : 0);
                    (void) uart_write_raw(reply, strlen(reply));
                    ESP_LOGI(TAG, "UIPS query answered: %s", reply);
                } else if (payload[0] == '1') {
                    // Panel activity detected - wake screensaver and reset idle timer
                    // Rate limit logging to avoid spam during active tuning
                    static int64_t last_uips1_log_time = 0;
                    constexpr int64_t UIPS1_LOG_INTERVAL_US = 5000000; // 5 seconds

                    screensaver_trigger_activity();

                    int64_t now = esp_timer_get_time();
                    if (now - last_uips1_log_time > UIPS1_LOG_INTERVAL_US) {
                        ESP_LOGI(TAG, "UIPS1 received: panel activity");
                        last_uips1_log_time = now;
                    }
                } else if (payload[0] == '0') {
                    // Panel idle - informational only, display manages own timeout
                    ESP_LOGD(TAG, "UIPS0 received: panel idle");
                } else {
                    ESP_LOGW(TAG, "UIPS invalid parameter: %s", response);
                }
            } else if (response[2] == 'P' && response[3] == 'T') {
                // UIPT - Panel Timeout query (screensaver timeout in minutes)
                // UIPT; = query current screensaver timeout
                // Returns: UIPT<n>; where n = 0 (disabled), 5, 10, 15, or 30
                int timeout_min = (int)screensaver_get_timeout();
                char reply[12];
                snprintf(reply, sizeof(reply), "UIPT%d;", timeout_min);
                (void) uart_write_raw(reply, strlen(reply));
                ESP_LOGI(TAG, "UIPT query answered: %s", reply);
            } else if (response[2] == 'D' && response[3] == 'E') {
                // UIDE - Display Communication Enable/Disable
                // UIDE;  = query current state
                // UIDE0; = disable display communications (panel can still send UIDE to re-enable)
                // UIDE1; = enable display communications
                const char *payload = response + 4;
                if (payload[0] == '\0' || payload[0] == ';') {
                    // Query: respond with current communication state
                    bool enabled = radio_state().display_communication_enabled.load();
                    char reply[8];
                    snprintf(reply, sizeof(reply), "UIDE%d;", enabled ? 1 : 0);
                    (void) uart_write_raw(reply, strlen(reply));
                    ESP_LOGD(TAG, "UIDE query answered: %s (comms %s)", reply, enabled ? "enabled" : "disabled");
                } else if (payload[0] == '0') {
                    // Disable display communications - only log if state changed
                    // NOTE: No reply sent for SET operations to prevent feedback loop
                    bool was_enabled = radio_state().display_communication_enabled.load();
                    radio_state().display_communication_enabled.store(false);
                    if (was_enabled) {
                        ESP_LOGI(TAG, "UIDE: display communications disabled");
                    }
                } else if (payload[0] == '1') {
                    // Enable display communications - only log if state changed
                    // NOTE: No reply sent for SET operations to prevent feedback loop
                    bool was_enabled = radio_state().display_communication_enabled.load();
                    radio_state().display_communication_enabled.store(true);
                    if (!was_enabled) {
                        ESP_LOGI(TAG, "UIDE: display communications enabled");
                    }
                } else {
                    ESP_LOGW(TAG, "UIDE invalid parameter: %s", response);
                }
            } else {
                ESP_LOGD(TAG, "Unknown UI command: %s", response);
            }
            break;

        case ((uint16_t) 'M' << 8) | 'X': // MX* commands (macros)
            // Handle MX protocol for user-defined macros
            // MX commands: MXR, MXA, MXW, MXD, MXE
            ESP_LOGI(TAG, "Received macro command: \"%s\"", response);
            parse_mx_command(response);
            break;

        default:
            // Log unrecognized commands to help debug RX issues
            if (strncmp(response, "RX", 2) == 0) {
                ESP_LOGW(TAG, "RX-like command not matching dispatcher: \"%s\" (len=%zu)", response, strlen(response));
            } else if (strlen(response) <= 10) { // Only log short commands to avoid spam
                ESP_LOGD(TAG, "Unrecognized CAT command: \"%s\"", response);
            }
            break;
    }
}

// ======== PEP (Peak Envelope Power) Implementation ========

static pep_data_t g_pep_data = {
    .current_power_raw = 0,
    .pep_power_raw = 0,
    .current_power_watts = 0.0f,
    .pep_power_watts = 0.0f,
    .pep_timestamp = 0,
    .enabled = true
};

static const power_cal_table_t g_power_cal_table = TS590_PWR_CAL;
static const uint32_t PEP_INTEGRATION_TIME_MS = 15; // 15ms integration time (within 10-20ms range)
// UI controls peak-hold decay cadence; do not decay PEP here

float convert_raw_power_to_watts(int raw_power) {
    // Clamp input to valid range
    if (raw_power <= 0) return 0.0f;
    if (raw_power >= 30) return 100.0f;
    
    // Find the two calibration points to interpolate between
    for (int i = 0; i < g_power_cal_table.num_points - 1; i++) {
        if (raw_power >= g_power_cal_table.points[i].raw_value && 
            raw_power <= g_power_cal_table.points[i + 1].raw_value) {
            
            // Linear interpolation between points
            float x1 = (float)g_power_cal_table.points[i].raw_value;
            float y1 = g_power_cal_table.points[i].watts;
            float x2 = (float)g_power_cal_table.points[i + 1].raw_value;
            float y2 = g_power_cal_table.points[i + 1].watts;
            
            return y1 + (y2 - y1) * ((float)raw_power - x1) / (x2 - x1);
        }
    }
    
    // Fallback (shouldn't reach here with proper calibration table)
    return 0.0f;
}

void pep_init(void) {
    g_pep_data.current_power_raw = 0;
    g_pep_data.pep_power_raw = 0;
    g_pep_data.current_power_watts = 0.0f;
    g_pep_data.pep_power_watts = 0.0f;
    g_pep_data.pep_timestamp = 0;
    g_pep_data.enabled = true;
    ESP_LOGV(TAG, "PEP system initialized");
}

void pep_enable(bool enabled) {
    g_pep_data.enabled = enabled;
    if (!enabled) {
        pep_reset();
    }
    ESP_LOGD(TAG, "PEP tracking %s", enabled ? "enabled" : "disabled");
}

void pep_update_power(int power_value) {
    if (!g_pep_data.enabled) {
        return;
    }
    
    // Clamp power value to valid range
    if (power_value < 0) power_value = 0;
    if (power_value > 30) power_value = 30;
    
    g_pep_data.current_power_raw = power_value;
    g_pep_data.current_power_watts = convert_raw_power_to_watts(power_value);
    uint32_t current_time = lv_tick_get();
    
    // Update peak only when current power exceeds previous peak.
    // Decay cadence is handled by the UI peak timer for unified behavior.
    bool peak_updated = false;
    if (power_value > g_pep_data.pep_power_raw) {
        peak_updated = true;
        g_pep_data.pep_power_raw = power_value;
        g_pep_data.pep_power_watts = convert_raw_power_to_watts(power_value);
        g_pep_data.pep_timestamp = current_time;
    }
    // If radio reports zero power explicitly, clear peak immediately.
    if (power_value == 0 && g_pep_data.pep_power_raw != 0) {
        peak_updated = true;
        g_pep_data.pep_power_raw = 0;
        g_pep_data.pep_power_watts = 0.0f;
        g_pep_data.pep_timestamp = current_time;
    }
    
    // Send update only if we have a new peak or significant change
    static uint32_t last_pep_update_time = 0;
    if (peak_updated || lv_tick_elaps(last_pep_update_time) >= PEP_INTEGRATION_TIME_MS) {
        ESP_LOGV(TAG, "PEP Update: Current=%d(%.1fW), Peak=%d(%.1fW)", 
                 g_pep_data.current_power_raw, g_pep_data.current_power_watts,
                 g_pep_data.pep_power_raw, g_pep_data.pep_power_watts);
        last_pep_update_time = current_time;
    }
}

pep_data_t* pep_get_data(void) {
    return &g_pep_data;
}

void pep_reset(void) {
    g_pep_data.current_power_raw = 0;
    g_pep_data.pep_power_raw = 0;
    g_pep_data.current_power_watts = 0.0f;
    g_pep_data.pep_power_watts = 0.0f;
    g_pep_data.pep_timestamp = 0;
    ESP_LOGD(TAG, "PEP data reset");
}
