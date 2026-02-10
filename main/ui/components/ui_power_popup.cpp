/**
 * @file ui_power_popup.c
 * @brief Power/Carrier level adjustment popup implementation
 */

#include "ui_power_popup.h"
#include "../ui.h"
#include "../ui_scale.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"  // For lvgl_port_lock/unlock
#include "uart.h"           // For uart_write_message to send commands
#include <stdio.h>

static const char *TAG = "UI_POPUP";

// Theme colors (matching Screen2 style exactly)
#define COLOR_BLUE_PRIMARY     0x2095F6  // Main theme blue
#define COLOR_SELECTIVE_YELLOW 0xFFBC1F  // Golden yellow accent
#define COLOR_BG_DARK          0x292831  // Panel background
#define COLOR_BG_MEDIUM        0x363636  // Control backgrounds
#define COLOR_TEXT             0xFFFFFF  // Primary text
#define COLOR_HINT_TEXT        0x666666  // Subtle text (matches COLOR_BG_LIGHT)

// Popup dimensions (design reference: 800x480)
#define POPUP_WIDTH   ui_sx(280)
#define POPUP_HEIGHT  ui_sy(120)

// UI elements
static lv_obj_t *popup_container = NULL;
static lv_obj_t *title_label = NULL;
static lv_obj_t *value_label = NULL;
static lv_obj_t *slider = NULL;
static lv_obj_t *toggle_button = NULL;
static lv_obj_t *hint_label = NULL;

// State
static ui_control_type_t active_control = UI_CONTROL_NONE;
static int current_value = 0;
static int min_value = 0;
static int max_value = 100;

// Forward declarations
static void update_value_label(void);
static void update_toggle_button(void);
static void toggle_button_event_cb(lv_event_t *e);
static const char *get_title_for_control(ui_control_type_t type);
static const char *get_hint_for_control(ui_control_type_t type);
static const char *get_unit_for_control(ui_control_type_t type);

void ui_power_popup_init(lv_obj_t *parent_screen)
{
    if (popup_container != NULL) {
        ESP_LOGW(TAG, "Popup already initialized");
        return;
    }

    ESP_LOGI(TAG, "Initializing power popup");

    // Create popup container (initially hidden)
    popup_container = lv_obj_create(parent_screen);
    lv_obj_set_size(popup_container, POPUP_WIDTH, POPUP_HEIGHT);
    lv_obj_set_y(popup_container, ui_sy(340));  // Position near bottom of screen
    lv_obj_set_align(popup_container, LV_ALIGN_TOP_MID);

    // Style the container (matching Screen2 panel style)
    lv_obj_set_style_bg_color(popup_container, lv_color_hex(COLOR_BG_DARK), 0);
    lv_obj_set_style_bg_opa(popup_container, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(popup_container, 8, 0);
    lv_obj_set_style_border_width(popup_container, 0, 0);
    lv_obj_set_style_shadow_width(popup_container, 15, 0);
    lv_obj_set_style_shadow_color(popup_container, lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(popup_container, LV_OPA_40, 0);
    lv_obj_set_style_pad_hor(popup_container, 16, 0);
    lv_obj_set_style_pad_ver(popup_container, 12, 0);

    // Disable scrollbar
    lv_obj_remove_flag(popup_container, LV_OBJ_FLAG_SCROLLABLE);

    // Use column flex layout with spacing between elements
    lv_obj_set_flex_flow(popup_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(popup_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Header row (title + value)
    lv_obj_t *header = lv_obj_create(popup_container);
    lv_obj_set_size(header, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_all(header, 0, 0);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Title label (matches Screen2 menu item labels)
    title_label = lv_label_create(header);
    lv_label_set_text(title_label, "RF Power");
    lv_obj_set_style_text_color(title_label, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_set_style_text_font(title_label, ui_btn_small_med_font(), 0);

    // Value label (matches Screen2 slider value labels)
    value_label = lv_label_create(header);
    lv_obj_set_style_text_color(value_label, lv_color_hex(COLOR_SELECTIVE_YELLOW), 0);
    lv_obj_set_style_text_font(value_label, ui_btn_small_reg_font(), 0);
    lv_label_set_text(value_label, "100W");

    // Slider
    slider = lv_slider_create(popup_container);
    lv_obj_set_width(slider, LV_PCT(100));
    lv_slider_set_range(slider, 5, 100);

    // Slider styling (matches Screen2 sliders exactly)
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_BG_MEDIUM), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_TEXT), LV_PART_KNOB);
    lv_obj_set_style_pad_all(slider, 4, LV_PART_KNOB);

    // Disable slider touch interaction (controlled by encoder only)
    lv_obj_remove_flag(slider, LV_OBJ_FLAG_CLICKABLE);

    // Toggle button (for binary controls like DATA mode)
    // Initially hidden, shown only for toggle controls (min=0, max=1)
    toggle_button = lv_button_create(popup_container);
    lv_obj_set_size(toggle_button, LV_PCT(100), 36);

    // Style the toggle button
    lv_obj_set_style_radius(toggle_button, 6, 0);
    lv_obj_set_style_bg_color(toggle_button, lv_color_hex(COLOR_BG_MEDIUM), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(toggle_button, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_STATE_CHECKED);
    lv_obj_set_style_border_width(toggle_button, 2, 0);
    lv_obj_set_style_border_color(toggle_button, lv_color_hex(COLOR_BG_MEDIUM), LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(toggle_button, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_STATE_CHECKED);

    // Add label to button
    lv_obj_t *btn_label = lv_label_create(toggle_button);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(COLOR_HINT_TEXT), LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(COLOR_BG_DARK), LV_STATE_CHECKED);
    lv_obj_set_style_text_font(btn_label, ui_btn_small_med_font(), 0);
    lv_label_set_text(btn_label, "OFF");
    lv_obj_center(btn_label);

    // Enable checkable state and add event handler
    lv_obj_add_flag(toggle_button, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(toggle_button, toggle_button_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Start hidden (only show for toggle controls)
    lv_obj_add_flag(toggle_button, LV_OBJ_FLAG_HIDDEN);

    // Hint label (subtle text for instructions)
    hint_label = lv_label_create(popup_container);
    lv_obj_set_style_text_color(hint_label, lv_color_hex(COLOR_HINT_TEXT), 0);
    lv_obj_set_style_text_font(hint_label, ui_btn_small_reg_font(), 0);
    lv_label_set_text(hint_label, "MULTI to adjust");

    // Start hidden
    lv_obj_add_flag(popup_container, LV_OBJ_FLAG_HIDDEN);

    ESP_LOGI(TAG, "Power popup initialized");
}

void ui_power_popup_show(ui_control_type_t type, int initial_value)
{
    if (popup_container == NULL) {
        ESP_LOGE(TAG, "Popup not initialized");
        return;
    }

    active_control = type;

    // Configure for control type
    switch (type) {
        case UI_CONTROL_POWER:
            min_value = 5;
            max_value = 100;
            break;
        case UI_CONTROL_CARRIER_LEVEL:
            min_value = 0;
            max_value = 20;
            break;
        case UI_CONTROL_AF_GAIN:
        case UI_CONTROL_RF_GAIN:
        case UI_CONTROL_MIC_GAIN:
            min_value = 0;
            max_value = 255;
            break;
        case UI_CONTROL_NR_LEVEL:
            min_value = 1;  // NR1: level 01-10
            max_value = 10;
            break;
        case UI_CONTROL_NB_LEVEL:
            min_value = 1;  // NL: 001-010 per spec
            max_value = 10;
            break;
        case UI_CONTROL_NR2_SPEED:
            min_value = 0;  // NR2: SPAC speed 00-09 (2-20ms)
            max_value = 9;
            break;
        case UI_CONTROL_PROC_INPUT_LEVEL:
        case UI_CONTROL_PROC_OUTPUT_LEVEL:
            min_value = 0;  // Processor levels: 0-100
            max_value = 100;
            break;
        case UI_CONTROL_NOTCH_FREQUENCY:
            min_value = 0;  // Manual notch frequency: 0-127
            max_value = 127;
            break;
        case UI_CONTROL_IF_SHIFT:
            min_value = 0;  // IF shift: 0-9999 Hz
            max_value = 9999;
            break;
        case UI_CONTROL_RIT_XIT_OFFSET:
            min_value = -9999;  // RIT/XIT offset: -9999 to +9999 Hz
            max_value = 9999;
            break;
        case UI_CONTROL_DATA_MODE:
            min_value = 0;  // Data mode toggle: 0=OFF, 1=ON
            max_value = 1;
            break;
        case UI_CONTROL_CW_CARRIER_LEVEL:
            min_value = 0;  // CW carrier level: 0-100%
            max_value = 100;
            break;
        default:
            ESP_LOGW(TAG, "Unknown control type: %d", type);
            return;
    }

    current_value = initial_value;
    if (current_value < min_value) current_value = min_value;
    if (current_value > max_value) current_value = max_value;

    // Acquire LVGL mutex - called from CAT parser task
    if (!lvgl_port_lock(100)) {
        ESP_LOGW(TAG, "Failed to acquire LVGL lock for popup show");
        return;
    }

    // Update UI elements
    lv_label_set_text(title_label, get_title_for_control(type));
    lv_label_set_text(hint_label, get_hint_for_control(type));

    // Detect if this is a toggle control (min=0, max=1)
    const bool is_toggle = (min_value == 0 && max_value == 1);

    if (is_toggle) {
        // Hide slider, show toggle button
        lv_obj_add_flag(slider, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(toggle_button, LV_OBJ_FLAG_HIDDEN);
        update_toggle_button();
    } else {
        // Hide toggle button, show slider
        lv_obj_add_flag(toggle_button, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(slider, LV_OBJ_FLAG_HIDDEN);
        lv_slider_set_range(slider, min_value, max_value);
        lv_slider_set_value(slider, current_value, LV_ANIM_OFF);
    }

    update_value_label();

    // Show popup
    lv_obj_remove_flag(popup_container, LV_OBJ_FLAG_HIDDEN);

    lvgl_port_unlock();

    ESP_LOGI(TAG, "Showing popup: type=%d, value=%d", type, initial_value);
}

void ui_power_popup_hide(void)
{
    if (popup_container == NULL) {
        return;
    }

    // Acquire LVGL mutex - called from CAT parser task
    if (!lvgl_port_lock(100)) {
        ESP_LOGW(TAG, "Failed to acquire LVGL lock for popup hide");
        return;
    }

    lv_obj_add_flag(popup_container, LV_OBJ_FLAG_HIDDEN);
    active_control = UI_CONTROL_NONE;

    lvgl_port_unlock();

    ESP_LOGI(TAG, "Popup hidden");
}

void ui_power_popup_set_value(int value)
{
    if (popup_container == NULL || active_control == UI_CONTROL_NONE) {
        return;
    }

    // Clamp to valid range
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;

    // Acquire LVGL mutex - called from CAT parser task
    if (!lvgl_port_lock(100)) {
        ESP_LOGW(TAG, "Failed to acquire LVGL lock for popup value update");
        return;
    }

    current_value = value;

    // Update appropriate UI element
    const bool is_toggle = (min_value == 0 && max_value == 1);
    if (is_toggle) {
        update_toggle_button();
    } else {
        lv_slider_set_value(slider, value, LV_ANIM_ON);
    }

    update_value_label();

    lvgl_port_unlock();

    ESP_LOGD(TAG, "Popup value updated: %d", value);
}

bool ui_power_popup_is_visible(void)
{
    if (popup_container == NULL) return false;
    return !lv_obj_has_flag(popup_container, LV_OBJ_FLAG_HIDDEN);
}

ui_control_type_t ui_power_popup_get_type(void)
{
    return active_control;
}

void ui_power_popup_cleanup(void)
{
    // LVGL handles deletion when parent screen is deleted
    // Just reset our pointers
    popup_container = NULL;
    title_label = NULL;
    value_label = NULL;
    slider = NULL;
    toggle_button = NULL;
    hint_label = NULL;
    active_control = UI_CONTROL_NONE;

    ESP_LOGI(TAG, "Popup cleanup complete");
}

// ============================================================================
// Private helpers
// ============================================================================

static void update_value_label(void)
{
    if (value_label == NULL) return;

    char buf[16];
    // Special handling for different control types
    if (active_control == UI_CONTROL_DATA_MODE) {
        // Data mode: show OFF/ON instead of 0/1
        snprintf(buf, sizeof(buf), "%s", current_value ? "ON" : "OFF");
    } else if (active_control == UI_CONTROL_RIT_XIT_OFFSET) {
        // RIT/XIT: use signed format to show + or - prefix
        snprintf(buf, sizeof(buf), "%+d%s", current_value, get_unit_for_control(active_control));
    } else {
        snprintf(buf, sizeof(buf), "%d%s", current_value, get_unit_for_control(active_control));
    }
    lv_label_set_text(value_label, buf);
}

static const char *get_title_for_control(ui_control_type_t type)
{
    switch (type) {
        case UI_CONTROL_POWER:         return "RF Power";
        case UI_CONTROL_CARRIER_LEVEL: return "TX Monitor";
        case UI_CONTROL_AF_GAIN:       return "AF Gain";
        case UI_CONTROL_RF_GAIN:       return "RF Gain";
        case UI_CONTROL_MIC_GAIN:      return "Mic Gain";
        case UI_CONTROL_NR_LEVEL:      return "NR1 Level";
        case UI_CONTROL_NB_LEVEL:      return "NB Level";
        case UI_CONTROL_NR2_SPEED:     return "NR2 Speed";
        case UI_CONTROL_PROC_INPUT_LEVEL:  return "Proc Input";
        case UI_CONTROL_PROC_OUTPUT_LEVEL: return "Proc Output";
        case UI_CONTROL_NOTCH_FREQUENCY:   return "Notch Freq";
        case UI_CONTROL_IF_SHIFT:          return "IF Shift";
        case UI_CONTROL_RIT_XIT_OFFSET:    return "RIT/XIT";
        case UI_CONTROL_DATA_MODE:         return "Data Mode";
        case UI_CONTROL_CW_CARRIER_LEVEL:  return "CW Carrier";
        default:                       return "Setting";
    }
}

static const char *get_hint_for_control(ui_control_type_t type)
{
    switch (type) {
        case UI_CONTROL_POWER:         return "5-100W | MULTI to adjust";
        case UI_CONTROL_CARRIER_LEVEL: return "0-20 | MULTI to adjust";
        case UI_CONTROL_AF_GAIN:       return "0-255 | MULTI to adjust";
        case UI_CONTROL_RF_GAIN:       return "0-255 | MULTI to adjust";
        case UI_CONTROL_MIC_GAIN:      return "0-100 | MULTI to adjust";
        case UI_CONTROL_NR_LEVEL:      return "1-10 | MULTI to adjust";
        case UI_CONTROL_NB_LEVEL:      return "1-10 | MULTI to adjust";
        case UI_CONTROL_NR2_SPEED:     return "0-9 (2-20ms) | MULTI to adjust";
        case UI_CONTROL_PROC_INPUT_LEVEL:  return "0-100 | MULTI to adjust";
        case UI_CONTROL_PROC_OUTPUT_LEVEL: return "0-100 | MULTI to adjust";
        case UI_CONTROL_NOTCH_FREQUENCY:   return "0-127 | MULTI to adjust";
        case UI_CONTROL_IF_SHIFT:          return "0-9999 Hz";
        case UI_CONTROL_RIT_XIT_OFFSET:    return "+/- 9999 Hz | CLR to reset";
        case UI_CONTROL_DATA_MODE:         return "Touch or MULTI to toggle";
        case UI_CONTROL_CW_CARRIER_LEVEL:  return "0-100% | MULTI to adjust";
        default:                       return "MULTI to adjust";
    }
}

static const char *get_unit_for_control(ui_control_type_t type)
{
    switch (type) {
        case UI_CONTROL_POWER:         return "W";
        case UI_CONTROL_IF_SHIFT:      return " Hz";
        case UI_CONTROL_RIT_XIT_OFFSET: return " Hz";
        case UI_CONTROL_CW_CARRIER_LEVEL: return "%";
        default:                       return "";
    }
}

static void update_toggle_button(void)
{
    if (toggle_button == NULL) return;

    // Update button checked state based on current_value
    if (current_value) {
        lv_obj_add_state(toggle_button, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(toggle_button, LV_STATE_CHECKED);
    }

    // Update button label text
    lv_obj_t *btn_label = lv_obj_get_child(toggle_button, 0);
    if (btn_label != NULL) {
        lv_label_set_text(btn_label, current_value ? "ON" : "OFF");
    }
}

static void toggle_button_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = static_cast<lv_obj_t *>(lv_event_get_target(e));

    if (code == LV_EVENT_VALUE_CHANGED) {
        // Toggle button was clicked (touch or encoder press)
        const bool is_checked = lv_obj_has_state(btn, LV_STATE_CHECKED);
        current_value = is_checked ? 1 : 0;

        // Update button label
        lv_obj_t *btn_label = lv_obj_get_child(btn, 0);
        if (btn_label != NULL) {
            lv_label_set_text(btn_label, current_value ? "ON" : "OFF");
        }

        // Update value label in header
        update_value_label();

        ESP_LOGI(TAG, "Toggle button changed: value=%d (touch)", current_value);

        // Send command to radio based on control type
        if (active_control == UI_CONTROL_DATA_MODE) {
            char cmd[8];
            snprintf(cmd, sizeof(cmd), "DA%d;", current_value);
            uart_write_message(cmd);
            ESP_LOGI(TAG, "Sent DATA mode command: %s", cmd);
        }
    }
}
