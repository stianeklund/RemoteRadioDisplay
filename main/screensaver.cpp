/**
 * @file screensaver.cpp
 * @brief LCD screensaver implementation to prevent image sticking
 *
 * Prevents LCD ghosting by blanking screen and turning off backlight after
 * configurable period of touch inactivity.
 */
#include "screensaver.h"
#include "gfx/lcd_init.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "uart.h"  // For uart_write_raw to notify panel of state changes

static const char *TAG = "SCREENSAVER";

// Screensaver state
// Default to 15 min timeout (good default to prevent image sticking)
// This must match the value set in screensaver_init() for UI consistency
static struct {
    screensaver_timeout_t timeout;
    bool active;
    uint8_t saved_backlight;
    lv_obj_t *overlay_screen;
    lv_timer_t *check_timer;
} g_screensaver = {
    .timeout = SCREENSAVER_15_MIN,  // Default enabled - UI reads this before screensaver_init()
    .active = false,
    .saved_backlight = 255,
    .overlay_screen = NULL,
    .check_timer = NULL
};

// Forward declarations
static void screensaver_check_cb(lv_timer_t *timer);
static void screensaver_activate(void);
static void screensaver_deactivate(void);

/**
 * @brief Activate screensaver (blank screen + backlight off)
 */
static void screensaver_activate(void) {
    if (g_screensaver.active) {
        return; // Already active
    }

    ESP_LOGI(TAG, "Activating screensaver");

    // Save current backlight level
    g_screensaver.saved_backlight = lcd_get_backlight_level();

    // Create fullscreen black overlay on current screen
    lv_obj_t *active_screen = lv_screen_active();
    if (!active_screen) {
        ESP_LOGW(TAG, "No active screen - cannot activate screensaver");
        return;
    }

    g_screensaver.overlay_screen = lv_obj_create(active_screen);
    if (!g_screensaver.overlay_screen) {
        ESP_LOGE(TAG, "Failed to create screensaver overlay");
        return;
    }

    // Configure overlay: fullscreen, black, opaque, no scrolling
    lv_obj_set_size(g_screensaver.overlay_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(g_screensaver.overlay_screen, 0, 0);
    lv_obj_set_style_bg_color(g_screensaver.overlay_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_screensaver.overlay_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_screensaver.overlay_screen, 0, 0);
    lv_obj_set_style_radius(g_screensaver.overlay_screen, 0, 0);
    lv_obj_remove_flag(g_screensaver.overlay_screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_screensaver.overlay_screen, LV_OBJ_FLAG_CLICKABLE);

    // Move to front
    lv_obj_move_foreground(g_screensaver.overlay_screen);

    // Turn off backlight
    lcd_set_backlight_level(0);

    g_screensaver.active = true;
    ESP_LOGI(TAG, "Screensaver active - backlight off");

    // Notify panel that display is now asleep
    (void) uart_write_raw("UIPS0;", 6);
}

/**
 * @brief Deactivate screensaver (restore display)
 */
static void screensaver_deactivate(void) {
    if (!g_screensaver.active) {
        return; // Not active
    }

    ESP_LOGI(TAG, "Deactivating screensaver");

    // Remove overlay
    if (g_screensaver.overlay_screen) {
        lv_obj_delete(g_screensaver.overlay_screen);
        g_screensaver.overlay_screen = NULL;
    }

    // Restore backlight
    lcd_set_backlight_level(g_screensaver.saved_backlight);

    g_screensaver.active = false;
    ESP_LOGI(TAG, "Screensaver deactivated - display restored");

    // Notify panel that display is now awake
    (void) uart_write_raw("UIPS1;", 6);
}

/**
 * @brief LVGL timer callback to check for inactivity
 *
 * Runs at 1 Hz to check if screensaver should activate/deactivate.
 * Only tracks touch/display inactivity via LVGL.
 */
static void screensaver_check_cb(lv_timer_t *timer) {
    (void)timer;

    // Screensaver disabled - nothing to do
    if (g_screensaver.timeout == SCREENSAVER_DISABLED) {
        if (g_screensaver.active) {
            screensaver_deactivate(); // Disabled while active - deactivate
        }
        return;
    }

    // Calculate timeout threshold in milliseconds
    uint32_t threshold_ms = g_screensaver.timeout * 60 * 1000;

    // Get touch/display inactivity time from LVGL
    uint32_t idle_ms = lv_display_get_inactive_time(NULL);

    // Debug logging (verbose level to avoid spam)
    ESP_LOGV(TAG, "Inactivity check: idle=%lu ms, threshold=%lu ms", idle_ms, threshold_ms);

    if (idle_ms >= threshold_ms) {
        if (!g_screensaver.active) {
            ESP_LOGI(TAG, "Inactivity timeout reached (%lu ms >= %lu ms) - activating screensaver",
                     idle_ms, threshold_ms);
            screensaver_activate();
        }
    } else {
        if (g_screensaver.active) {
            ESP_LOGI(TAG, "Touch activity detected - deactivating screensaver");
            screensaver_deactivate();
        }
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t screensaver_init(void) {
    ESP_LOGI(TAG, "Initializing screensaver module");

    // Initialize state with 15-minute timeout (good default to prevent image sticking)
    g_screensaver.timeout = SCREENSAVER_15_MIN;
    g_screensaver.active = false;
    g_screensaver.saved_backlight = lcd_get_backlight_level();
    g_screensaver.overlay_screen = NULL;

    // Create LVGL timer to check inactivity at 1 Hz
    if (!lvgl_port_lock(100)) {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock for timer creation");
        return ESP_FAIL;
    }

    g_screensaver.check_timer = lv_timer_create(screensaver_check_cb, 1000, NULL);
    if (!g_screensaver.check_timer) {
        lvgl_port_unlock();
        ESP_LOGE(TAG, "Failed to create screensaver check timer");
        return ESP_FAIL;
    }

    lvgl_port_unlock();

    ESP_LOGI(TAG, "Screensaver initialized (default: 15 minutes)");
    return ESP_OK;
}

void screensaver_set_timeout(screensaver_timeout_t timeout) {
    ESP_LOGI(TAG, "Setting screensaver timeout to %d minutes", (int)timeout);
    g_screensaver.timeout = timeout;

    // If disabled, deactivate immediately
    if (timeout == SCREENSAVER_DISABLED && g_screensaver.active) {
        if (lvgl_port_lock(100)) {
            screensaver_deactivate();
            lvgl_port_unlock();
        }
    }
}

screensaver_timeout_t screensaver_get_timeout(void) {
    return g_screensaver.timeout;
}

bool screensaver_is_active(void) {
    return g_screensaver.active;
}

void screensaver_trigger_activity(void) {
    // Trigger LVGL activity to reset inactivity timer
    lv_display_trigger_activity(NULL);

    // If screensaver is active, deactivate it immediately
    if (g_screensaver.active) {
        if (lvgl_port_lock(100)) {
            ESP_LOGD(TAG, "Manual activity trigger - waking screensaver");
            screensaver_deactivate();
            lvgl_port_unlock();
        }
        return;
    }

    // Also restore backlight if it was manually dimmed via UIBL000
    // (screensaver not active, but backlight is off)
    uint8_t current_backlight = lcd_get_backlight_level();
    if (current_backlight == 0 && g_screensaver.saved_backlight == 0) {
        // Backlight was manually turned off - restore to full brightness
        ESP_LOGI(TAG, "Panel activity detected - restoring backlight from manual dim");
        lcd_set_backlight_level(255);
        g_screensaver.saved_backlight = 255;
    }
}

void screensaver_update_backlight(uint8_t new_level) {
    // Update saved backlight level so screensaver restores correct value
    if (!g_screensaver.active) {
        // Not active - just update saved level
        g_screensaver.saved_backlight = new_level;
        ESP_LOGD(TAG, "Backlight level updated: %u (screensaver inactive)", new_level);
    } else {
        // Active - this is an external change, wake up and apply new level
        if (lvgl_port_lock(100)) {
            ESP_LOGI(TAG, "External backlight change detected - waking screensaver");
            screensaver_deactivate();
            // Deactivate will restore saved_backlight, but we want the new level
            lcd_set_backlight_level(new_level);
            g_screensaver.saved_backlight = new_level;
            lvgl_port_unlock();
        }
    }

    // Trigger LVGL activity to reset inactivity timer
    lv_display_trigger_activity(NULL);
}
