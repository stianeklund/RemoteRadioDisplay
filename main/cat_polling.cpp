#include "cat_polling.h"
#include "cat_parser.h"
#include "radio/radio_subjects.h"
#include "radio/radio_subject_updater.h"
#include "esp_check.h"
#include "uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "task_handles.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAT_POLLING";

// LVGL mutex timeout for timer operations (ms)
// Use longer timeout during init when LVGL may be busy with display setup
#define LVGL_TIMER_MUTEX_TIMEOUT_MS 100
#define LVGL_TIMER_MUTEX_TIMEOUT_INIT_MS 500

// Thread-safe LVGL timer wrappers using esp_lvgl_port
static lv_timer_t* safe_lv_timer_create(lv_timer_cb_t cb, uint32_t period, void* user_data) {
    lv_timer_t* timer = NULL;
    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_MS)) {
        timer = lv_timer_create(cb, period, user_data);
        lvgl_port_unlock();
    } else {
        ESP_LOGW(TAG, "Failed to acquire LVGL lock for timer create");
    }
    return timer;
}

// Timer create with retry for initialization - longer timeout and retry on failure
static lv_timer_t* safe_lv_timer_create_with_retry(lv_timer_cb_t cb, uint32_t period, void* user_data) {
    lv_timer_t* timer = NULL;

    // First attempt with longer init timeout
    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_INIT_MS)) {
        timer = lv_timer_create(cb, period, user_data);
        lvgl_port_unlock();
        return timer;
    }

    // Retry after short delay if first attempt failed
    ESP_LOGD(TAG, "LVGL lock busy, retrying timer create...");
    vTaskDelay(pdMS_TO_TICKS(50));

    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_INIT_MS)) {
        timer = lv_timer_create(cb, period, user_data);
        lvgl_port_unlock();
    } else {
        ESP_LOGW(TAG, "Failed to acquire LVGL lock for timer create after retry");
    }
    return timer;
}

static void safe_lv_timer_pause(lv_timer_t* timer) {
    if (!timer) return;
    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_MS)) {
        lv_timer_pause(timer);
        lvgl_port_unlock();
    }
}

static void safe_lv_timer_resume(lv_timer_t* timer) {
    if (!timer) return;
    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_MS)) {
        lv_timer_resume(timer);
        lvgl_port_unlock();
    }
}

static void safe_lv_timer_set_period(lv_timer_t* timer, uint32_t period) {
    if (!timer) return;
    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_MS)) {
        lv_timer_set_period(timer, period);
        lvgl_port_unlock();
    }
}

static void safe_lv_timer_set_repeat_count(lv_timer_t* timer, int32_t count) {
    if (!timer) return;
    if (lvgl_port_lock(LVGL_TIMER_MUTEX_TIMEOUT_MS)) {
        lv_timer_set_repeat_count(timer, count);
        lvgl_port_unlock();
    }
}

// Polling intervals in milliseconds (optimized for ~35% UART traffic reduction)
#define POLLING_INTERVAL_IF    150   // IF command - main status (was 100ms)
#define POLLING_INTERVAL_SM    300   // SM command - S-meter (was 200ms, 3.3Hz is sufficient)
#define POLLING_INTERVAL_RM    100   // RM command - cycles through SWR/PWR/ALC (was 67ms, 300ms cycle)
#define POLLING_INTERVAL_FR    500   // FR command - RX VFO
#define POLLING_INTERVAL_FA    250   // FA command - VFO A frequency (was 200ms)
#define POLLING_INTERVAL_FB    250   // FB command - VFO B frequency (was 200ms)
#define POLLING_INTERVAL_FT    500   // FT command - TX VFO
#define POLLING_INTERVAL_FILTER 2000 // SH/SL filter commands - check every 2 seconds
#define POLLING_INTERVAL_AI_STATUS 10000 // AI status monitoring - check every 10 seconds
#define CAT_ACTIVITY_TIMEOUT_MS 5000 // Consider CAT inactive after 5 seconds
#define AGC_QUERY_INTERVAL_MS 1000   // Retry AGC read until response is received

// Global polling state
static cat_polling_state_t g_polling_state = {
    .polling_enabled = false,
    .user_polling_override = false, 
    .ai_mode = AI_MODE_UNKNOWN,
    .expected_ai_mode = AI_MODE_UNKNOWN,
    .rx_vfo = VFO_UNKNOWN,
    .tx_vfo = VFO_UNKNOWN,
    .cat_connection_active = false,
    .last_cat_activity = 0
};

// LVGL timers for polling
static lv_timer_t *timer_if = NULL;
static lv_timer_t *timer_sm = NULL;
static lv_timer_t *timer_rm = NULL;
static lv_timer_t *timer_fr = NULL;
static lv_timer_t *timer_fa = NULL;
static lv_timer_t *timer_fb = NULL;
static lv_timer_t *timer_ft = NULL;
static lv_timer_t *timer_ai_check = NULL;
static lv_timer_t *timer_ai_status_monitor = NULL;
static lv_timer_t *timer_agc_query = NULL;

// RM meter type cycling state (1=SWR, 2=COMP/PWR, 3=ALC)
static uint8_t s_rm_meter_index = 0;
static const char *s_rm_meter_commands[] = {"RM1;", "RM2;", "RM3;"};
#define RM_METER_COUNT 3

// Boot sequence state
static lv_timer_t *boot_seq_timer = NULL;
static size_t boot_seq_index = 0;
static bool boot_seq_running = false;
static bool boot_seq_done = false;

// AGC query retry state
static bool agc_query_active = false;
static bool agc_state_known = false;

// Periodic re-polling state (every ~5 minutes)
#define PERIODIC_REPOLLING_INTERVAL_MS 300000  // 5 minutes
#define PERIODIC_POLL_SPACING_MS 40            // 40ms between commands (~25Hz)
static lv_timer_t *periodic_poll_timer = NULL;
static lv_timer_t *periodic_poll_spacing_timer = NULL;
static bool periodic_poll_sequence_running = false;
static size_t periodic_poll_sequence_index = 0;

static void boot_seq_timer_cb(lv_timer_t *timer);
static void periodic_poll_timer_cb(lv_timer_t *timer);
static void periodic_poll_spacing_cb(lv_timer_t *timer);
static void agc_query_timer_cb(lv_timer_t *timer);
static void start_agc_query(void);
static void stop_agc_query(void);

// Commands to populate initial state once radio responsiveness is verified (PS1)
static const char *boot_sequence[] = {
    // Query key status/state so parser updates internal/UI state
    "AI;", "IF;", "TS;", "TP;", "PR;", "FR;", "FT;", "MD;", "ML;", "FA;", "FB;",
    "FL;", "GC;", "NT;", "XT;", "MC;", "RT;", "SH;", "SL;", "SP;", "SQ0;", "TO;", "TP;", "PC;",
    // Additional state queries for periodic re-polling
    "NR;", "NB;", "PA;", "RA;", "AC;",
    // Query filter mode settings (EX028=SSB, EX029=SSB-DATA)
    // These determine if SH/SL are High/Low cut or Width/Shift
    "EX0280000;", "EX0290000;",
    // Query Radio Menu items
    "EX0060000;", "EX0400000;", "EX0530000;",
    "EX0710000;", "EX0720000;", "EX0730000;", "EX0740000;"
    // TEMPORARILY DISABLED: Query transverter configuration during early boot
    // TODO: Re-enable after investigating heap corruption crash
    // "XO;", "EX0560000;"
};
static const size_t boot_sequence_len = sizeof(boot_sequence) / sizeof(boot_sequence[0]);

// Forward declarations
static void polling_timer_if_cb(lv_timer_t *timer);
static void polling_timer_sm_cb(lv_timer_t *timer);
static void polling_timer_rm_cb(lv_timer_t *timer);
static void polling_timer_fr_cb(lv_timer_t *timer);
static void polling_timer_fa_cb(lv_timer_t *timer);
static void polling_timer_fb_cb(lv_timer_t *timer);
static void polling_timer_ft_cb(lv_timer_t *timer);
static void polling_timer_ai_check_cb(lv_timer_t *timer);
static void polling_timer_ai_status_monitor_cb(lv_timer_t *timer);
static void update_polling_state(void);
static void start_polling_timers(void);
static void stop_polling_timers(void);
static void start_ai_status_monitoring(void);
static void stop_ai_status_monitoring(void);
static void start_periodic_polling(void);
static void stop_periodic_polling(void);

esp_err_t cat_polling_init(void) {
    ESP_LOGI(TAG, "Initializing CAT polling manager");
    
    // Initialize state
    g_polling_state.polling_enabled = false;
    g_polling_state.user_polling_override = false;
    g_polling_state.ai_mode = AI_MODE_UNKNOWN;
    g_polling_state.expected_ai_mode = AI_MODE_UNKNOWN;
    g_polling_state.rx_vfo = VFO_UNKNOWN;
    g_polling_state.tx_vfo = VFO_UNKNOWN;
    g_polling_state.cat_connection_active = true;  // Assume CAT is available from start
    g_polling_state.last_cat_activity = esp_timer_get_time() / 1000;
    
    // Create AI mode check timer (runs once to check AI mode)
    // Use retry version during init as LVGL may be busy with display setup
    timer_ai_check = safe_lv_timer_create_with_retry(polling_timer_ai_check_cb, 2000, NULL);
    safe_lv_timer_set_repeat_count(timer_ai_check, 1); // Run only once

    // Query AGC until we get a valid response
    start_agc_query();
    
    // Check if polling should start immediately (in case user override is already enabled)
    update_polling_state();
    
    ESP_LOGI(TAG, "CAT polling manager initialized");
    return ESP_OK;
}

esp_err_t cat_polling_start(void) {
    ESP_LOGI(TAG, "Starting CAT polling");

    g_polling_state.cat_connection_active = true;
    g_polling_state.last_cat_activity = esp_timer_get_time() / 1000;

    start_agc_query();

    // Coalesced initial probe: PS checks radio, AI+FA+FB get initial state
    // TS-590SG supports multiple commands in single UART transaction
    (void) uart_write_message("PS;AI;FA;FB;");

    // Query macro configuration from panel interface using MX protocol
    // MXA gets F-key assignments, then query all macros individually
    (void) uart_write_message("MXA;");
    for (int i = 1; i <= 50; i++) {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), "MXR%02d;", i);
        (void) uart_write_message(cmd);
    }

    // Start periodic state refresh (runs independently of polling toggle)
    start_periodic_polling();

    update_polling_state();
    return ESP_OK;
}

void cat_polling_stop(void) {
    ESP_LOGI(TAG, "Stopping CAT polling");

    stop_polling_timers();
    stop_periodic_polling();
    stop_ai_status_monitoring();
    stop_agc_query();
    agc_state_known = false;
    g_polling_state.polling_enabled = false;
    g_polling_state.cat_connection_active = false;
}

void cat_polling_set_user_override(bool enabled) {
    ESP_LOGI(TAG, "Setting user polling override: %s", enabled ? "enabled" : "disabled");
    
    g_polling_state.user_polling_override = enabled;
    update_polling_state();
}

bool cat_polling_is_enabled(void) {
    return g_polling_state.polling_enabled;
}

bool cat_polling_get_user_override(void) {
    return g_polling_state.user_polling_override;
}

esp_err_t cat_polling_set_ai_mode(cat_ai_mode_t mode) {
    if (mode != AI_MODE_OFF && mode != AI_MODE_ON && mode != AI_MODE_ON_BACKUP) {
        ESP_LOGE(TAG, "Invalid AI mode for setting: %d", mode);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting AI mode to: AI%d", mode);
    
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "AI%d;", mode);
    
    esp_err_t ret = uart_write_message(cmd);
    if (ret == ESP_OK) {
        cat_ai_mode_t old_mode = g_polling_state.ai_mode;
        g_polling_state.ai_mode = mode;
        g_polling_state.expected_ai_mode = mode; // Track what we expect the radio to be set to
        
        bool old_polling_enabled = g_polling_state.polling_enabled;
        update_polling_state();
        bool new_polling_enabled = g_polling_state.polling_enabled;
        
        // Start AI status monitoring if setting AI2 or AI4
        if (mode == AI_MODE_ON || mode == AI_MODE_ON_BACKUP) {
            start_ai_status_monitoring();
        } else {
            stop_ai_status_monitoring();
        }
        
        // If polling state didn't change, we still need to notify UI about AI mode change
        if (old_polling_enabled == new_polling_enabled && old_mode != mode) {
            ESP_LOGI(TAG, "AI mode changed from %d to %d, notifying UI", old_mode, mode);
            radio_subject_notify_async(&radio_cat_polling_state_subject);
        }
    }
    
    return ret;
}

cat_ai_mode_t cat_polling_get_ai_mode(void) {
    return g_polling_state.ai_mode;
}

void cat_polling_update_ai_mode(cat_ai_mode_t mode) {
    if (g_polling_state.ai_mode == mode) {
        update_polling_state(); // Call to ensure consistency, but no extra notification needed
        return; // No change
    }

    ESP_LOGI(TAG, "AI mode updated to: %d", mode);
    g_polling_state.ai_mode = mode;
    
    // Check for AI mode mismatch if we have an expected mode set
    if (g_polling_state.expected_ai_mode != AI_MODE_UNKNOWN && 
        g_polling_state.expected_ai_mode != mode &&
        (g_polling_state.expected_ai_mode == AI_MODE_ON || g_polling_state.expected_ai_mode == AI_MODE_ON_BACKUP)) {
        ESP_LOGW(TAG, "AI mode mismatch detected! Expected: %d, Actual: %d. Attempting recovery.", 
                 g_polling_state.expected_ai_mode, mode);
        
        // Attempt to restore expected AI mode
        char recovery_cmd[8];
        snprintf(recovery_cmd, sizeof(recovery_cmd), "AI%d;", g_polling_state.expected_ai_mode);
        esp_err_t ret = uart_write_message(recovery_cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send AI recovery command: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Sent AI recovery command: %s", recovery_cmd);
        }
        return; // Don't update state yet, wait for response
    }

    bool old_polling_enabled = g_polling_state.polling_enabled;
    update_polling_state(); // This will update g_polling_state.polling_enabled and notify if the state flips
    bool new_polling_enabled = g_polling_state.polling_enabled;

    // If the polling state did not change (e.g., switching from AI2 to AI4),
    // the UI would not have been notified from update_polling_state().
    // We must send a notification here to ensure the button text updates.
    if (old_polling_enabled == new_polling_enabled) {
        radio_subject_notify_async(&radio_cat_polling_state_subject);
    }
}

void cat_polling_update_rx_vfo(cat_vfo_t vfo) {
    if (g_polling_state.rx_vfo != vfo) {
        ESP_LOGI(TAG, "RX VFO updated: %d", vfo);
        g_polling_state.rx_vfo = vfo;
    }
}

void cat_polling_update_tx_vfo(cat_vfo_t vfo) {
    if (g_polling_state.tx_vfo != vfo) {
        ESP_LOGI("CAT_PARSER", "TX VFO updated: %d", vfo);
        g_polling_state.tx_vfo = vfo;
    }
}

cat_vfo_t cat_polling_get_rx_vfo(void) {
    return g_polling_state.rx_vfo;
}

cat_vfo_t cat_polling_get_tx_vfo(void) {
    return g_polling_state.tx_vfo;
}

void cat_polling_mark_activity(void) {
    g_polling_state.last_cat_activity = esp_timer_get_time() / 1000;
    
    if (!g_polling_state.cat_connection_active) {
        ESP_LOGI(TAG, "CAT connection became active");
        g_polling_state.cat_connection_active = true;
        update_polling_state();
    }
}

bool cat_polling_is_cat_active(void) {
    uint64_t now = esp_timer_get_time() / 1000;
    uint64_t time_since_activity = now - g_polling_state.last_cat_activity;

    if (g_polling_state.cat_connection_active && time_since_activity > CAT_ACTIVITY_TIMEOUT_MS) {
        ESP_LOGW(TAG, "CAT connection timeout - no activity for %llu ms", time_since_activity);
        g_polling_state.cat_connection_active = false;
        update_polling_state();
    }

    return g_polling_state.cat_connection_active;
}

uint32_t cat_polling_get_last_activity_time(void) {
    return (uint32_t)g_polling_state.last_cat_activity;
}

const cat_polling_state_t* cat_polling_get_state(void) {
    return &g_polling_state;
}

// Private functions

static void update_polling_state(void) {
    bool should_poll = false;

    // The user_polling_override is a master switch from the UI.
    // If it's OFF, we never poll.
    // If it's ON, we always poll regardless of AI mode or CAT connection status.
    if (g_polling_state.user_polling_override) {
        should_poll = true;
    } else {
        // UI switch is OFF, so never poll.
        should_poll = false;
    }

    if (should_poll != g_polling_state.polling_enabled) {
        ESP_LOGI(TAG, "Polling state changed: %s -> %s (UI switch: %s)",
                 g_polling_state.polling_enabled ? "enabled" : "disabled",
                 should_poll ? "enabled" : "disabled",
                 g_polling_state.user_polling_override ? "ON" : "OFF");

        g_polling_state.polling_enabled = should_poll;

        if (should_poll) {
            start_polling_timers();
        } else {
            stop_polling_timers();
        }

        // Notify UI about polling state change (marshalled to LVGL task)
        radio_subject_notify_async(&radio_cat_polling_state_subject);
    }
}

static void start_polling_timers(void) {
    ESP_LOGI(TAG, "Starting polling timers");

    // Query current state immediately to ensure UI is up to date
    // This prevents display issues when FB updates arrive before FT state is known
    ESP_LOGI(TAG, "Querying current VFO and frequency state");
    uart_write_message("FR;FT;FA;FB;");

    // Create timers if they don't exist (all wrapped with mutex protection)
    if (timer_if == NULL) {
        timer_if = safe_lv_timer_create(polling_timer_if_cb, POLLING_INTERVAL_IF, NULL);
    }
    if (timer_sm == NULL) {
        timer_sm = safe_lv_timer_create(polling_timer_sm_cb, POLLING_INTERVAL_SM, NULL);
    }
    if (timer_rm == NULL) {
        timer_rm = safe_lv_timer_create(polling_timer_rm_cb, POLLING_INTERVAL_RM, NULL);
    }
    if (timer_fr == NULL) {
        timer_fr = safe_lv_timer_create(polling_timer_fr_cb, POLLING_INTERVAL_FR, NULL);
    }
    if (timer_fa == NULL) {
        timer_fa = safe_lv_timer_create(polling_timer_fa_cb, POLLING_INTERVAL_FA, NULL);
    }
    if (timer_fb == NULL) {
        timer_fb = safe_lv_timer_create(polling_timer_fb_cb, POLLING_INTERVAL_FB, NULL);
    }
    if (timer_ft == NULL) {
        timer_ft = safe_lv_timer_create(polling_timer_ft_cb, POLLING_INTERVAL_FT, NULL);
    }
    // Note: Filter polling removed - SH/SL commands only sent during boot sequence

    // Resume timers
    safe_lv_timer_resume(timer_if);
    safe_lv_timer_resume(timer_sm);
    safe_lv_timer_resume(timer_rm);
    safe_lv_timer_resume(timer_fr);
    safe_lv_timer_resume(timer_fa);
    safe_lv_timer_resume(timer_fb);
    safe_lv_timer_resume(timer_ft);
}

static void stop_polling_timers(void) {
    ESP_LOGI(TAG, "Stopping polling timers");

    // Pause timers (safe wrappers handle NULL check)
    safe_lv_timer_pause(timer_if);
    safe_lv_timer_pause(timer_sm);
    safe_lv_timer_pause(timer_rm);
    safe_lv_timer_pause(timer_fr);
    safe_lv_timer_pause(timer_fa);
    safe_lv_timer_pause(timer_fb);
    safe_lv_timer_pause(timer_ft);
}

static void start_periodic_polling(void) {
    ESP_LOGI(TAG, "Starting periodic re-polling timer (5 min interval)");
    periodic_poll_sequence_running = false;
    periodic_poll_sequence_index = 0;

    if (periodic_poll_timer == NULL) {
        periodic_poll_timer = safe_lv_timer_create(periodic_poll_timer_cb, PERIODIC_REPOLLING_INTERVAL_MS, NULL);
        if (periodic_poll_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create periodic polling timer");
            return;
        }
    } else {
        safe_lv_timer_set_period(periodic_poll_timer, PERIODIC_REPOLLING_INTERVAL_MS);
        safe_lv_timer_resume(periodic_poll_timer);
    }
}

static void stop_periodic_polling(void) {
    ESP_LOGI(TAG, "Stopping periodic re-polling timer");
    safe_lv_timer_pause(periodic_poll_timer);
    safe_lv_timer_pause(periodic_poll_spacing_timer);
    periodic_poll_sequence_running = false;
}

// Timer callback functions

static void polling_timer_if_cb(lv_timer_t *timer) {
    (void)timer;

    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("IF;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send IF command: %s", esp_err_to_name(ret));
        }
    }
}

static void polling_timer_sm_cb(lv_timer_t *timer) {
    (void)timer;
    
    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("SM;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send SM command: %s", esp_err_to_name(ret));
        }
    }
}

static void polling_timer_rm_cb(lv_timer_t *timer) {
    (void)timer;

    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        // Cycle through RM1 (SWR), RM2 (COMP/PWR), RM3 (ALC) to get all meter readings
        // This ensures we poll all three meter types, not just the user-selected one
        const char *cmd = s_rm_meter_commands[s_rm_meter_index];
        esp_err_t ret = uart_write_message(cmd);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send %s command: %s", cmd, esp_err_to_name(ret));
        }

        // Advance to next meter type for next poll cycle
        s_rm_meter_index = (s_rm_meter_index + 1) % RM_METER_COUNT;
    }
}

static void polling_timer_fr_cb(lv_timer_t *timer) {
    (void)timer;
    
    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("FR;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send FR command: %s", esp_err_to_name(ret));
        }
    }
}

static void polling_timer_fa_cb(lv_timer_t *timer) {
    (void)timer;
    
    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("FA;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send FA command: %s", esp_err_to_name(ret));
        }
    }
}

static void polling_timer_fb_cb(lv_timer_t *timer) {
    (void)timer;
    
    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("FB;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send FB command: %s", esp_err_to_name(ret));
        }
    }
}

static void polling_timer_ft_cb(lv_timer_t *timer) {
    (void)timer;
    
    if (g_polling_state.polling_enabled && g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("FT;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send FT command: %s", esp_err_to_name(ret));
        }
    }
}


static void polling_timer_ai_check_cb(lv_timer_t *timer) {
    (void)timer;

    ESP_LOGI(TAG, "Initial AI mode check");

    // Coalesced: AI mode + VFO status queries in single transaction
    esp_err_t ret = uart_write_message("AI;FR;FT;");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send initial AI/VFO query: %s", esp_err_to_name(ret));
    }
}

static void polling_timer_ai_status_monitor_cb(lv_timer_t *timer) {
    (void)timer;
    
    // Only monitor if we expect AI2 or AI4 and CAT is active
    if (!g_polling_state.cat_connection_active) {
        return;
    }
    
    if (g_polling_state.expected_ai_mode != AI_MODE_ON && 
        g_polling_state.expected_ai_mode != AI_MODE_ON_BACKUP) {
        return; // Not monitoring AI0 or unknown modes
    }
    
    ESP_LOGD(TAG, "AI status monitoring check - expected: AI%d, current: AI%d", 
             g_polling_state.expected_ai_mode, g_polling_state.ai_mode);
    
    // Send AI query to check current mode
    esp_err_t ret = uart_write_message("AI;");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send AI status check: %s", esp_err_to_name(ret));
    }
}

void cat_polling_set_expected_ai_mode(cat_ai_mode_t mode) {
    g_polling_state.expected_ai_mode = mode;
    ESP_LOGI(TAG, "Expected AI mode set to: %d", mode);
    
    // Start monitoring if setting AI2 or AI4
    if (mode == AI_MODE_ON || mode == AI_MODE_ON_BACKUP) {
        start_ai_status_monitoring();
    } else {
        stop_ai_status_monitoring();
    }
}

static void start_ai_status_monitoring(void) {
    ESP_LOGI(TAG, "Starting AI status monitoring timer");

    // Create timer if it doesn't exist
    if (timer_ai_status_monitor == NULL) {
        timer_ai_status_monitor = safe_lv_timer_create(polling_timer_ai_status_monitor_cb, POLLING_INTERVAL_AI_STATUS, NULL);
        if (timer_ai_status_monitor == NULL) {
            ESP_LOGE(TAG, "Failed to create AI status monitoring timer");
            return;
        }
    }

    // Resume timer
    safe_lv_timer_resume(timer_ai_status_monitor);
}

static void stop_ai_status_monitoring(void) {
    ESP_LOGI(TAG, "Stopping AI status monitoring timer");
    safe_lv_timer_pause(timer_ai_status_monitor);
}

static void agc_query_timer_cb(lv_timer_t *timer) {
    LV_UNUSED(timer);

    if (!agc_query_active || agc_state_known) {
        safe_lv_timer_pause(timer_agc_query);
        return;
    }

    if (!g_polling_state.cat_connection_active) {
        return;
    }

    esp_err_t ret = uart_write_message("GC;");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send AGC query: %s", esp_err_to_name(ret));
    }
}

static void start_agc_query(void) {
    agc_query_active = true;
    agc_state_known = false;

    if (timer_agc_query == NULL) {
        // Use retry version as this may be called during init when LVGL is busy
        timer_agc_query = safe_lv_timer_create_with_retry(agc_query_timer_cb, AGC_QUERY_INTERVAL_MS, NULL);
    } else {
        safe_lv_timer_set_period(timer_agc_query, AGC_QUERY_INTERVAL_MS);
        safe_lv_timer_resume(timer_agc_query);
    }

    if (g_polling_state.cat_connection_active) {
        esp_err_t ret = uart_write_message("GC;");
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send initial AGC query: %s", esp_err_to_name(ret));
        }
    }
}

static void stop_agc_query(void) {
    agc_query_active = false;
    safe_lv_timer_pause(timer_agc_query);
}

// Periodic state refresh callback - triggers every 5 minutes
// Runs independently of the polling toggle to keep state in sync
static void periodic_poll_timer_cb(lv_timer_t *timer) {
    LV_UNUSED(timer);

    // Only requires active CAT connection, NOT the polling toggle
    if (!g_polling_state.cat_connection_active) {
        return;
    }

    ESP_LOGI(TAG, "Periodic state refresh - querying radio state");

    // Start the periodic polling sequence
    periodic_poll_sequence_running = true;
    periodic_poll_sequence_index = 0;

    if (periodic_poll_spacing_timer == NULL) {
        periodic_poll_spacing_timer = safe_lv_timer_create(periodic_poll_spacing_cb, PERIODIC_POLL_SPACING_MS, NULL);
        if (periodic_poll_spacing_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create periodic poll spacing timer");
            periodic_poll_sequence_running = false;
            return;
        }
    } else {
        safe_lv_timer_set_period(periodic_poll_spacing_timer, PERIODIC_POLL_SPACING_MS);
        safe_lv_timer_resume(periodic_poll_spacing_timer);
    }
}

// Helper callback for spacing out periodic re-poll commands (~40ms apart)
static void periodic_poll_spacing_cb(lv_timer_t *timer) {
    if (!periodic_poll_sequence_running || periodic_poll_sequence_index >= boot_sequence_len) {
        // Sequence complete
        periodic_poll_sequence_running = false;
        safe_lv_timer_pause(timer);
        ESP_LOGI(TAG, "Periodic re-polling sequence completed (%zu commands sent)", boot_sequence_len);
        return;
    }

    const char *cmd = boot_sequence[periodic_poll_sequence_index++];
    esp_err_t ret = uart_write_message(cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Periodic poll cmd failed (%s): %s", esp_err_to_name(ret), cmd);
    }
}

// ===== Boot sequence helpers =====
static void start_boot_sequence(void) {
    ESP_LOGI(TAG, "Starting CAT boot sequence (%u cmds)", (unsigned)boot_sequence_len);
    boot_seq_index = 0;
    boot_seq_running = true;
    if (boot_seq_timer == NULL) {
        boot_seq_timer = safe_lv_timer_create(boot_seq_timer_cb, 40, NULL); // ~25 Hz pacing
    } else {
        safe_lv_timer_set_period(boot_seq_timer, 40);
        safe_lv_timer_resume(boot_seq_timer);
    }
}

static void stop_boot_sequence(void) {
    safe_lv_timer_pause(boot_seq_timer);
    boot_seq_running = false;
}

static void boot_seq_timer_cb(lv_timer_t *timer) {
    LV_UNUSED(timer);
    if (!boot_seq_running) return;
    if (boot_seq_index >= boot_sequence_len) {
        stop_boot_sequence();
        boot_seq_done = true;
        ESP_LOGI(TAG, "CAT boot sequence completed");
        return;
    }
    const char *cmd = boot_sequence[boot_seq_index++];
    esp_err_t ret = uart_write_message(cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Boot seq send failed (%s): %s", esp_err_to_name(ret), cmd);
    }
}

// Public: called by parser when PS; answer parsed
void cat_polling_handle_ps_status(bool ps_on) {
    if (ps_on) {
        if (!boot_seq_running && !boot_seq_done) {
            start_boot_sequence();
        }
    } else {
        // Power off or invalid; cancel any boot attempt
        stop_boot_sequence();
        boot_seq_done = false;
    }
}

void cat_polling_handle_agc_response(void) {
    if (!agc_state_known) {
        ESP_LOGI(TAG, "AGC response received; stopping AGC query retries");
    }
    agc_state_known = true;
    stop_agc_query();
}

// ============================================================================
// Memory Channel Polling
// ============================================================================

static uint16_t s_last_memory_channel = 0xFFFF;  // 0xFFFF = not yet requested

void cat_polling_request_memory_channel(uint16_t channel) {
    if (channel > 999) {
        ESP_LOGW(TAG, "Invalid memory channel: %u", channel);
        return;
    }

    // Build MR command: MR0[hundreds][tens][units];
    // P1=0 (simplex), P2=hundreds digit, P3=last two digits
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "MR0%d%02d;", channel / 100, channel % 100);

    ESP_LOGI(TAG, "Requesting memory channel %u: %s", channel, cmd);
    esp_err_t ret = uart_write_message(cmd);
    if (ret == ESP_OK) {
        s_last_memory_channel = channel;
    } else {
        ESP_LOGW(TAG, "Failed to send MR command: %s", esp_err_to_name(ret));
    }
}

uint16_t cat_polling_get_last_memory_channel(void) {
    return s_last_memory_channel;
}
