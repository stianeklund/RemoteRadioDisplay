#include "lcd_init.h"
#include "../ui.h"
#include "../ui_scale.h"
#include "../../cat_parser.h"
#include "../../cat_polling.h"
#include "../../settings_storage.h"
#include "../../task_handles.h"
#include "../../gfx/lcd_init.h"
#include "../../radio/radio_subjects.h"  // LVGL 9 native observer subjects
#include "../../antenna_control.h"
#include "../../websocket_client.h"
#include "../../uart.h"  // For uart_write_message
#include "../../screensaver.h"  // For screensaver control
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_lvgl_port.h"  // For lvgl_port_lock/unlock
#include "esp_log.h"

static const char *TAG = "ui_Screen2";

// DEBUG: Include menu private header to inspect storage pointer
#include "lvgl_private.h"

// Color definitions from Screen1 for consistent UI theming
#define COLOR_BLUE_PRIMARY         0x2095F6  // Main theme blue
#define COLOR_ORCHID_PINK          0xE667DE  // Accent pink
#define COLOR_EMERALD_GREEN        0x4BC97B  // Success green
#define COLOR_ORANGE_VIBRANT       0xF26419  // Vibrant orange accent
#define COLOR_SELECTIVE_YELLOW     0xFFBC1F  // Golden yellow accent
#define COLOR_BG_DARK              0x292831  // Panel background
#define COLOR_GUNMETAL             0x22323c  // Sidebar background
#define COLOR_BG_MEDIUM            0x363636  // Control backgrounds
#define COLOR_TEXT                 0xFFFFFF  // Primary text
#define COLOR_RED_LIGHT            0xD93F3F  // Alert/warning color
#define COLOR_BG_LIGHT     0x666666  // disabled buttons  
#define COLOR_BG_DARK_GRAY_BUTTON  0x414141  // Darker gray for antenna buttons

lv_obj_t * ui_Screen2;
lv_obj_t * ui_ReturnToScreen1Button;
lv_obj_t * ui_ReturnToScreen1BtnLabel;
lv_obj_t * ui_SettingsPageTitle;
lv_obj_t * ui_BrightnessLabel;
lv_obj_t * ui_BrightnessSlider;
lv_obj_t * ui_CwSidetoneSlider;
lv_obj_t * ui_CwSidetoneValueLabel;
lv_obj_t * ui_CwPitchSlider;
lv_obj_t * ui_CwPitchValueLabel;
lv_obj_t * ui_FmMicGainSlider;
lv_obj_t * ui_FmMicGainValueLabel;
lv_obj_t * ui_UsbAudioInputSlider;
lv_obj_t * ui_UsbAudioInputValueLabel;
lv_obj_t * ui_UsbAudioOutputSlider;
lv_obj_t * ui_UsbAudioOutputValueLabel;
lv_obj_t * ui_Acc2InputSlider;
lv_obj_t * ui_Acc2InputValueLabel;
lv_obj_t * ui_Acc2OutputSlider;
lv_obj_t * ui_Acc2OutputValueLabel;
lv_obj_t * ui_PlaceholderButton1;
lv_obj_t * ui_TransverterButton;
lv_obj_t * ui_PlaceholderButton2;
lv_obj_t * ui_PlaceholderButton3;
lv_obj_t * ui_PlaceholderLabel3;
lv_obj_t * ui_PeakHoldLabel;
lv_obj_t * ui_PeakHoldSwitch;
lv_obj_t * ui_PeakHoldDurationLabel;
lv_obj_t * ui_PeakHoldDurationSlider;
lv_obj_t * ui_PeakHoldDurationValueLabel; // New: Value label for peak duration
lv_obj_t * ui_BrightnessValueLabel; // New: Value label for brightness
lv_obj_t * ui_PepToggleLabel;
lv_obj_t * ui_PepToggleSwitch;
lv_obj_t * ui_SMeterAveragingSwitch;
lv_obj_t * ui_RebootButton;
lv_obj_t * ui_ScreensaverButton;
lv_obj_t * ui_ScreensaverLabel;

// Internal menu root for the Settings screen
static lv_obj_t * ui_Menu;
static lv_obj_t * ui_MenuPageDisplay;
static lv_obj_t * ui_MenuPageRadioMenu;
static lv_obj_t * ui_MenuPageCat;
static lv_obj_t * ui_MenuPageMeter;
static lv_obj_t * ui_MenuPageAntennas;
static lv_obj_t * ui_MenuPageMacros;
static lv_obj_t * ui_MenuPageSystem;
static bool ui_MenuDeferredBuilt = false;

// Macro management state
#define MAX_UI_MACROS 50
#define MACRO_NAME_MAX 32
#define MACRO_COMMAND_MAX 64
#define MACRO_FKEY_SLOTS 6  // F1-F6

typedef struct {
    uint8_t id;
    uint8_t fkey;  // 0=unassigned, 1-6=F1-F6
    char name[MACRO_NAME_MAX];
    char command[MACRO_COMMAND_MAX];
    lv_obj_t *row_container;  // For dynamic updates/deletion
} ui_macro_item_t;

static ui_macro_item_t ui_macro_cache[MAX_UI_MACROS] = {0};
static uint8_t ui_macro_count = 0;
static uint8_t ui_macro_fkey_assignments[MACRO_FKEY_SLOTS] = {0};  // macro_id for each F-key slot (0=empty)
static int ui_macro_selected_index = -1;  // Currently selected row for deletion (-1=none)

// Inline macro editor state (table-based editing)
typedef struct {
    lv_obj_t *edit_overlay;       // Full overlay panel for editing (covers macro list)
    lv_obj_t *keyboard;           // Keyboard in overlay
    lv_obj_t *active_textarea;    // Currently editing textarea (name or command)
    lv_obj_t *edit_label;         // Label showing what's being edited
    lv_obj_t *table_container;    // Main table container for scrolling
    lv_obj_t *fkey_popup;         // F-key assignment popup
    uint8_t editing_macro_id;     // 0 = new macro, 1-50 = existing
    bool is_editing;              // Currently in edit mode
    bool editing_name;            // true=editing name, false=editing command
} ui_macro_editor_t;

static ui_macro_editor_t macro_editor = {0};

// Debounced refresh timer for macro list (avoid flashing when receiving many MXR responses)
static TimerHandle_t macro_refresh_timer = NULL;
static const uint32_t MACRO_REFRESH_DEBOUNCE_MS = 100;  // Wait 100ms after last update before refresh

// Theme colors for menu icons (from ui_Screen1.c)
#define COLOR_AQUA_LIGHT     0x2DD6C7  // middle icon glow
#define COLOR_SELECTIVE_YELLOW 0xFFBC1F  // golden yellow accent
#define COLOR_SUCCESS_GREEN      0x57C25F  // success messages
#define COLOR_ARGENTINIAN_BLUE   0x4DAAFF  // lighter, for hovers or highlights
#define COLOR_PURPLE_ACCENT      0x9C27B0  // for antenna menu icon

// Settings tracking for save-on-exit
typedef struct {
    bool xvtr_offset_mix_enabled;
    bool cat_polling_enabled;
    cat_ai_mode_t ai_mode;
    bool peak_hold_enabled;
    uint32_t peak_hold_duration_ms;
    bool pep_enabled;
    bool smeter_averaging_enabled;
    bool antenna_switch_enabled;
    bool settings_changed;
} ui_settings_state_t;

static ui_settings_state_t current_settings = {0};

// FreeRTOS queue for NVS operations (LCD sync fixed via sdkconfig)
static QueueHandle_t nvs_save_queue = NULL;

// Static variable to track XVTR offset mix toggle state
static bool xvtr_offset_mix_enabled = false;

// Static variable to track Transverter (UIXD) enabled state
static bool transverter_enabled = false;

// Static variable to track Panel Data (UIDE) enabled state (default: enabled)
static bool panel_data_enabled = true;

// Static variable to store antenna switch button reference
static lv_obj_t *ui_AntennaSwitchButton = NULL;
static lv_obj_t *ui_PanelDataButton = NULL;

typedef struct {
    uint16_t menu;
    int min;
    int max;
    int step;
    lv_subject_t *subject;
    lv_obj_t *slider;
    lv_obj_t *value_label;
    const char *suffix;
    bool show_off;
} radio_menu_slider_t;

static radio_menu_slider_t g_radio_menu_fm_mic_gain = {53, 1, 3, 1, &radio_fm_mic_gain_subject, NULL, NULL, NULL, false};
static radio_menu_slider_t g_radio_menu_usb_input = {71, 0, 9, 1, &radio_usb_input_level_subject, NULL, NULL, NULL, false};
static radio_menu_slider_t g_radio_menu_usb_output = {72, 0, 9, 1, &radio_usb_output_level_subject, NULL, NULL, NULL, false};
static radio_menu_slider_t g_radio_menu_acc2_input = {73, 0, 9, 1, &radio_acc2_input_level_subject, NULL, NULL, NULL, false};
static radio_menu_slider_t g_radio_menu_acc2_output = {74, 0, 9, 1, &radio_acc2_output_level_subject, NULL, NULL, NULL, false};

// Guard against recursive slider updates when snapping values
static bool s_cw_pitch_internal_update = false;
static bool s_radio_menu_internal_update = false;

// Forward declarations
void ui_update_polling_controls(void);
static void ui_load_saved_settings(void);
// static void post_screen2_init_cb(lv_timer_t *t);  // Commented out - not used
static void ui_event_PlaceholderButton1_clicked(lv_event_t * e);
static void ui_event_TransverterButton_clicked(lv_event_t * e);
static void ui_event_PollingButton_clicked(lv_event_t * e);
static void ui_event_AIModeButton_clicked(lv_event_t * e);
static void ui_event_ScreensaverButton_clicked(lv_event_t * e);
static void ui_event_CwSidetoneSlider(lv_event_t * e);
static void ui_event_CwPitchSlider(lv_event_t * e);
static void ui_event_RadioMenuSlider(lv_event_t * e);
static void ui_event_AntennaButton_clicked(lv_event_t * e);
static void ui_update_antenna_button_states(int current_antenna, const uint8_t* available_antennas, int available_count);
static void back_event_handler(lv_event_t * e);
static void queue_settings_save(void);
static void ui_update_cw_sidetone_display(int value);
static void ui_update_cw_pitch_display(int value);
static void ui_send_ex_menu_value(uint16_t menu, int value);
static lv_obj_t * create_radio_menu_row(lv_obj_t * parent, const char * label_text, int32_t min, int32_t max, int32_t val,
                                        lv_obj_t **out_slider, lv_obj_t **out_value_label);
static void ui_update_radio_menu_item(radio_menu_slider_t *item, int value);

// Macro management functions
static void ui_macro_populate_list(void);
static void ui_macro_sort_cache(void);
static void ui_macro_show_fkey_popup(uint8_t macro_id);
static void ui_macro_hide_fkey_popup(void);
static void ui_macro_show_delete_confirm(uint8_t macro_id);
static void ui_macro_start_inline_edit(int cache_index, bool edit_name);
static void ui_macro_end_inline_edit(bool save);
static void ui_macro_keyboard_event_cb(lv_event_t *e);
void ui_macro_refresh_list(void);  // Non-static: called from cat_parser.cpp
void ui_macro_request_refresh(void);  // Non-static: debounced refresh for batch MXR updates
void ui_macro_set_cached(uint8_t id, const char *name, const char *cmd);  // Non-static: called from cat_parser.cpp
void ui_macro_set_fkey_assignment(uint8_t fkey, uint8_t macro_id);  // Non-static: called from cat_parser.cpp

// Sidebar collapse/expand for full-screen macro editing
static bool g_sidebar_collapsed = false;
static lv_obj_t *g_sidebar_expand_btn = NULL;    // Expand button shown when collapsed (on screen)
static lv_obj_t *g_sidebar_header_chevron = NULL; // Chevron label in header for collapse indicator
static void ui_sidebar_set_collapsed(bool collapsed);
static void ui_sidebar_toggle(void);
static void ui_sidebar_toggle_cb(lv_event_t *e);

// Antenna state cache for smart updates
typedef struct {
    int current_antenna;
    uint8_t available_antennas[8];
    int available_count;
    bool cache_valid;
} antenna_state_cache_t;

static antenna_state_cache_t g_antenna_cache = {0};

// Store antenna button and label pointers for dynamic label updates
typedef struct {
    lv_obj_t *button;
    lv_obj_t *label;
} antenna_button_t;

static antenna_button_t g_antenna_buttons[8] = {0};

// Brightness slider now uses direct 0-255 values with improved PWM mapping

// Helper functions for creating menu items with icons
static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt);
static lv_obj_t * create_text_colored(lv_obj_t * parent, const char * icon, const char * txt, uint32_t icon_color);
static lv_obj_t * create_slider(lv_obj_t * parent, const char * icon, const char * txt, int32_t min, int32_t max, int32_t val);
static lv_obj_t * create_switch(lv_obj_t * parent, const char * icon, const char * txt, bool chk);
static lv_obj_t * create_button(lv_obj_t * parent, const char * icon, const char * txt, const char * btn_text);

// Back button event handler for menu root back button
static void back_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = (lv_obj_t*)lv_event_get_target(e);
    lv_obj_t * menu = (lv_obj_t *)lv_event_get_user_data(e);

    if(code == LV_EVENT_CLICKED) {
        // Handle clicks from either the back button or the Settings header
        bool is_back_button = lv_menu_back_button_is_root(menu, obj);
        bool is_settings_header = (obj == lv_menu_get_sidebar_header(menu));

        if (is_back_button || is_settings_header) {
            // NVS save disabled - just return to Screen1
            _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Screen1_screen_init);
        }
    }
}

// Helper function to create text menu items with icons
static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt)
{
    lv_obj_t * obj = lv_menu_cont_create(parent);
    lv_obj_set_style_pad_hor(obj, ui_sx(15), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_ver(obj, ui_sy(12), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(obj, 6, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Better dark mode UX - more pronounced highlight for pressed state
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x505050), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(obj, 150, LV_PART_MAIN | LV_STATE_PRESSED);

    if(icon) {
        lv_obj_t * icon_label = lv_label_create(obj);
        lv_label_set_text(icon_label, icon);
        lv_obj_set_style_text_font(icon_label, ui_symbol_font_lg(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(icon_label, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    if(txt) {
        lv_obj_t * label = lv_label_create(obj);
        lv_label_set_text(label, txt);
        lv_obj_set_style_text_font(label, ui_btn_small_med_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_flex_grow(label, 1);
    }

    return obj;
}

// Helper function to create text menu items with colored icons
static lv_obj_t * create_text_colored(lv_obj_t * parent, const char * icon, const char * txt, uint32_t icon_color)
{
    lv_obj_t * obj = lv_menu_cont_create(parent);
    lv_obj_set_style_pad_hor(obj, ui_sx(15), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_ver(obj, ui_sy(12), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(obj, 6, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Better dark mode UX - more pronounced highlight for pressed state
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x505050), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(obj, 150, LV_PART_MAIN | LV_STATE_PRESSED);

    // Focused/selected state - pronounced left border accent for better UX
    lv_obj_set_style_bg_color(obj, lv_color_hex(0x404040), LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_bg_opa(obj, 120, LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_width(obj, 4, LV_PART_MAIN | LV_STATE_FOCUSED);  // Wider border
    lv_obj_set_style_border_color(obj, lv_color_hex(icon_color), LV_PART_MAIN | LV_STATE_FOCUSED);
    lv_obj_set_style_border_opa(obj, 255, LV_PART_MAIN | LV_STATE_FOCUSED);  // Full opacity for vibrant colors
    lv_obj_set_style_border_side(obj, LV_BORDER_SIDE_LEFT, LV_PART_MAIN | LV_STATE_FOCUSED);

    if(icon) {
        lv_obj_t * icon_label = lv_label_create(obj);
        lv_label_set_text(icon_label, icon);
        lv_obj_set_style_text_font(icon_label, ui_symbol_font_lg(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(icon_label, lv_color_hex(icon_color), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    if(txt) {
        lv_obj_t * label = lv_label_create(obj);
        lv_label_set_text(label, txt);
        lv_obj_set_style_text_font(label, ui_btn_small_med_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_flex_grow(label, 1);
    }

    return obj;
}

// Helper function to create slider menu items with icons
static lv_obj_t * create_slider(lv_obj_t * parent, const char * icon, const char * txt, int32_t min, int32_t max, int32_t val)
{
    lv_obj_t * obj = create_text(parent, icon, txt);
    
    // Add value label for numeric display - only show ms for Peak Duration
    lv_obj_t * value_label = lv_label_create(obj);
    static char value_buf[16];
    if (strstr(txt, "Peak Duration") != NULL) {
        snprintf(value_buf, sizeof(value_buf), "%ldms", val);
    } else {
        snprintf(value_buf, sizeof(value_buf), "%ld", val);
    }
    lv_label_set_text(value_label, value_buf);
    lv_obj_set_style_text_font(value_label, ui_btn_small_reg_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(value_label, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    lv_obj_t * slider = lv_slider_create(obj);
    lv_obj_set_flex_grow(slider, 1);
    lv_obj_set_size(slider, ui_sx(230), ui_sy(24));
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, val, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_BG_MEDIUM), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_TEXT), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(slider, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    if(icon == NULL) {
        lv_obj_add_flag(slider, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    }

    return obj;
}

// Helper function to create switch menu items with icons
static lv_obj_t * create_switch(lv_obj_t * parent, const char * icon, const char * txt, bool chk)
{
    lv_obj_t * obj = create_text(parent, icon, txt);

    lv_obj_t * sw = lv_switch_create(obj);
    lv_obj_set_size(sw, ui_sx(50), ui_sy(28));
    lv_obj_add_state(sw, chk ? LV_STATE_CHECKED : LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COLOR_BG_MEDIUM), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(sw, lv_color_hex(COLOR_BLUE_PRIMARY), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_pad_all(sw, ui_sy(2), LV_PART_MAIN | LV_STATE_DEFAULT);

    return obj;
}

// Helper function to create button menu items with icons (for AI mode)
static lv_obj_t * create_button(lv_obj_t * parent, const char * icon, const char * txt, const char * btn_text)
{
    lv_obj_t * obj = create_text(parent, icon, txt);
    
    lv_obj_t * btn = lv_button_create(obj);
    lv_obj_set_size(btn, ui_sx(120), ui_sy(35));
    lv_obj_set_style_radius(btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(btn, lv_color_hex(COLOR_BG_MEDIUM), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(btn, 80, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    lv_obj_t * btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, btn_text);
    lv_obj_set_style_text_font(btn_label, ui_btn_small_med_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(btn_label);

    return obj;
}

// Helper function to create a two-column radio menu row with a large value display
static lv_obj_t * create_radio_menu_row(lv_obj_t * parent, const char * label_text, int32_t min, int32_t max, int32_t val,
                                        lv_obj_t **out_slider, lv_obj_t **out_value_label)
{
    lv_obj_t * row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(row, ui_sx(12), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(row, ui_sx(12), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(row, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(row, lv_color_hex(COLOR_BG_MEDIUM), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(row, lv_color_hex(0x505050), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(row, 150, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_border_width(row, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * label = lv_label_create(row);
    lv_label_set_text(label, label_text);
    lv_obj_set_style_text_font(label, ui_btn_small_med_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_width(label, ui_sx(190));

    lv_obj_t * slider = lv_slider_create(row);
    lv_obj_set_size(slider, ui_sx(230), ui_sy(24));
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, val, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_TEXT), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(slider, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * value_label = lv_label_create(row);
    lv_label_set_text(value_label, "0");
    lv_obj_set_style_text_font(value_label, ui_btn_small_med_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(value_label, lv_color_hex(COLOR_SELECTIVE_YELLOW), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(value_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_width(value_label, ui_sx(110));

    if (out_slider) {
        *out_slider = slider;
    }
    if (out_value_label) {
        *out_value_label = value_label;
    }

    return row;
}

// Helper to send EX menu commands (EX[menu]0000[value];)
static void ui_send_ex_menu_value(uint16_t menu, int value)
{
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "EX%03u0000%d;", (unsigned)menu, value);
    uart_write_message(cmd);
}

// No-op callback - all initialization now done synchronously
/* Commented out - not used
static void post_screen2_init_cb(lv_timer_t *t) {
    LV_UNUSED(t);
    // All initialization is now done synchronously in ui_Screen2_screen_init
    // This callback is no longer needed but kept for compatibility
}
*/

// FreeRTOS task for batched NVS operations
// Runs on low priority to minimize impact on display
// ESP32-S3 RGB LCD has known issue with display corruption during flash writes
// See: https://github.com/espressif/esp-idf/issues/10010
static void nvs_save_task(void *pvParameter) {
    ui_settings_state_t settings_to_save;

    while (1) {
        // Wait for settings to save
        if (xQueueReceive(nvs_save_queue, &settings_to_save, portMAX_DELAY)) {
            ESP_LOGI("NVS_SAVE_TASK", "Saving batched settings to NVS...");

            // Save antenna switch setting
            esp_err_t ret = settings_save_antenna_switch(settings_to_save.antenna_switch_enabled);
            if (ret != ESP_OK) {
                ESP_LOGW("NVS_SAVE_TASK", "Failed to save antenna switch: %s", esp_err_to_name(ret));
            }

            // Add other settings saves here as needed
            // ret = settings_save_xvtr_offset_mix(settings_to_save.xvtr_offset_mix_enabled);
            // ret = settings_save_cat_polling(settings_to_save.cat_polling_enabled);
            // etc.

            ESP_LOGI("NVS_SAVE_TASK", "Batched settings save completed");
        }
    }
}

// Queue settings for background save
static void queue_settings_save(void) {
    if (!current_settings.settings_changed || nvs_save_queue == NULL) {
        return;
    }
    
    // Send copy of current settings to background task
    if (xQueueSend(nvs_save_queue, &current_settings, 0) == pdTRUE) {
        current_settings.settings_changed = false;
        ESP_LOGI("UI_Screen2", "Settings queued for background save");
    } else {
        ESP_LOGW("UI_Screen2", "Failed to queue settings save");
    }
}

// event funtions
void ui_event_ReturnToScreen1Button(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        // NVS save disabled - just return to Screen1
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Screen1_screen_init);
    }
}

static void event_handler_brightness_slider(lv_event_t * e)
{
    lv_obj_t * slider = (lv_obj_t*)lv_event_get_target(e);
    int32_t slider_value = lv_slider_get_value(slider);
    
    // Direct mapping - PWM function now handles the safe range mapping internally
    uint8_t brightness_level = (uint8_t)slider_value;
    
    // Update the value label to show brightness percentage (more intuitive)
    if (lv_obj_is_valid(ui_BrightnessValueLabel)) {
        static char value_text[16];
        uint8_t percentage = (brightness_level * 100) / 255;
        snprintf(value_text, sizeof(value_text), "%u%%", percentage);
        lv_label_set_text(ui_BrightnessValueLabel, value_text);
    }
    
    // lcd_set_backlight_level is declared in gfx/lcd_init.h, which is included via ../ui.h
    lcd_set_backlight_level(brightness_level);
}

// Event handler for XVTR offset mix toggle button
static void ui_event_PlaceholderButton1_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *switch_obj = (lv_obj_t*)lv_event_get_target(e);
        bool is_checked = lv_obj_has_state(switch_obj, LV_STATE_CHECKED);

        xvtr_offset_mix_enabled = is_checked;

        ESP_LOGI("UI_Screen2", "XVTR Offset Mix toggled: %s (local offset application)",
                 is_checked ? "ON" : "OFF");

        // Trigger frequency display refresh with new transverter logic
        cat_request_transverter_display_refresh();
    }
}

// Getter function for XVTR offset mix toggle state
bool ui_get_xvtr_offset_mix_enabled(void) {
    return xvtr_offset_mix_enabled;
}

// Setter function for XVTR offset mix toggle state (called from CAT parser)
void ui_set_xvtr_offset_mix_enabled(bool enabled) {
    xvtr_offset_mix_enabled = enabled;

    // Update UI toggle state (must be called from LVGL task with mutex held)
    if (lv_obj_is_valid(ui_PlaceholderButton1)) {
        if (enabled) {
            lv_obj_add_state(ui_PlaceholderButton1, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_PlaceholderButton1, LV_STATE_CHECKED);
        }
    }

    ESP_LOGD("UI_Screen2", "XVTR Offset Mix state updated from CAT: %s", enabled ? "ON" : "OFF");
}

// Event handler for Transverter toggle button (sends UIXD to ARCI)
static void ui_event_TransverterButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *switch_obj = (lv_obj_t*)lv_event_get_target(e);
        bool is_checked = lv_obj_has_state(switch_obj, LV_STATE_CHECKED);

        transverter_enabled = is_checked;

        // Send UIXD command to ARCI firmware
        // When enabled, ARCI presents frequency already mixed with transverter offset
        const char *cmd = is_checked ? "UIXD1;" : "UIXD0;";
        uart_write_message(cmd);

        ESP_LOGI("UI_Screen2", "Transverter toggled: %s (sent %s to ARCI)",
                 is_checked ? "ON" : "OFF", cmd);

        // Trigger frequency display refresh
        cat_request_transverter_display_refresh();
    }
}

// Getter function for Transverter toggle state
bool ui_get_transverter_enabled(void) {
    return transverter_enabled;
}

// Setter function for Transverter toggle state (called from CAT parser)
void ui_set_transverter_enabled(bool enabled) {
    transverter_enabled = enabled;

    // Update UI toggle state (must be called from LVGL task with mutex held)
    if (lv_obj_is_valid(ui_TransverterButton)) {
        if (enabled) {
            lv_obj_add_state(ui_TransverterButton, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_TransverterButton, LV_STATE_CHECKED);
        }
    }

    ESP_LOGD("UI_Screen2", "Transverter state updated from CAT: %s", enabled ? "ON" : "OFF");
}

// Event handler for Panel Data toggle button (sends UIDE to panel)
static void ui_event_PanelDataButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *switch_obj = (lv_obj_t*)lv_event_get_target(e);
        bool is_checked = lv_obj_has_state(switch_obj, LV_STATE_CHECKED);

        // Only send command if state actually changed to prevent feedback loop
        if (is_checked != panel_data_enabled) {
            panel_data_enabled = is_checked;

            // Send UIDE command to panel firmware
            // When disabled, panel stops sending data updates to display
            const char *cmd = is_checked ? "UIDE1;" : "UIDE0;";
            uart_write_message(cmd);

            ESP_LOGI("UI_Screen2", "Panel Data toggled: %s (sent %s to panel)",
                     is_checked ? "ON" : "OFF", cmd);
        }
    }
}

// Getter function for Panel Data toggle state
bool ui_get_panel_data_enabled(void) {
    return panel_data_enabled;
}

// Setter function for Panel Data toggle state (called from CAT parser)
void ui_set_panel_data_enabled(bool enabled) {
    panel_data_enabled = enabled;

    // Update UI toggle state (must be called from LVGL task with mutex held)
    // Suppress events to prevent feedback loop when updating from CAT responses
    if (lv_obj_is_valid(ui_PanelDataButton)) {
        lv_obj_remove_event_cb(ui_PanelDataButton, ui_event_PanelDataButton_clicked);
        if (enabled) {
            lv_obj_add_state(ui_PanelDataButton, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_PanelDataButton, LV_STATE_CHECKED);
        }
        lv_obj_add_event_cb(ui_PanelDataButton, ui_event_PanelDataButton_clicked, LV_EVENT_VALUE_CHANGED, NULL);
    }

    ESP_LOGD("UI_Screen2", "Panel Data state updated from CAT: %s", enabled ? "ON" : "OFF");
}

// Event handler for CW sidetone volume slider (EX006)
static void ui_event_CwSidetoneSlider(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if (event_code == LV_EVENT_VALUE_CHANGED && lv_obj_is_valid(target)) {
        int32_t value = lv_slider_get_value(target);

        if (lv_obj_is_valid(ui_CwSidetoneValueLabel)) {
            static char value_text[16];
            if (value == 0) {
                snprintf(value_text, sizeof(value_text), "Off");
            } else {
                snprintf(value_text, sizeof(value_text), "%ld", (long)value);
            }
            lv_label_set_text(ui_CwSidetoneValueLabel, value_text);
        }

        ui_send_ex_menu_value(6, (int)value);
        lv_subject_set_int(&radio_cw_sidetone_volume_subject, value);

        ESP_LOGD("UI_Screen2", "CW sidetone volume set to: %ld", (long)value);
    }
}

// Event handler for CW TX pitch / sidetone frequency slider (EX040)
static void ui_event_CwPitchSlider(lv_event_t * e)
{
    if (s_cw_pitch_internal_update) {
        return;
    }

    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if (event_code == LV_EVENT_VALUE_CHANGED && lv_obj_is_valid(target)) {
        int32_t raw_value = lv_slider_get_value(target);
        int32_t snapped = (raw_value + 25) / 50 * 50;
        if (snapped < 300) snapped = 300;
        if (snapped > 1000) snapped = 1000;

        if (snapped != raw_value) {
            s_cw_pitch_internal_update = true;
            lv_slider_set_value(target, snapped, LV_ANIM_OFF);
            s_cw_pitch_internal_update = false;
        }
        if (lv_obj_is_valid(ui_CwPitchValueLabel)) {
            static char value_text[16];
            snprintf(value_text, sizeof(value_text), "%ld Hz", (long)snapped);
            lv_label_set_text(ui_CwPitchValueLabel, value_text);
        }
        ui_send_ex_menu_value(40, (int)snapped);
        lv_subject_set_int(&radio_cw_pitch_hz_subject, snapped);

        ESP_LOGD("UI_Screen2", "CW pitch set to: %ld Hz", (long)snapped);
    }
}


// Core polling state display update logic
static void ui_update_polling_state(void) {
    ui_update_polling_controls();
}

// LVGL 9 observer callback for CAT polling state updates
static void ui_polling_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    LV_UNUSED(subject);
    ui_update_polling_state();
}

// Core display refresh logic
static void ui_force_display_refresh(void) {
    ESP_LOGD("UI_Screen2", "Forcing display refresh after NVS operation");
    lcd_force_display_refresh();
}

// LVGL 9 observer callback for display refresh
static void ui_display_refresh_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    LV_UNUSED(subject);
    ui_force_display_refresh();
}

// Core backlight display update logic
static void ui_update_backlight_display(int level) {
    if (lv_obj_is_valid(ui_BrightnessSlider)) {
        lv_slider_set_value(ui_BrightnessSlider, level, LV_ANIM_OFF);
    }
    if (lv_obj_is_valid(ui_BrightnessValueLabel)) {
        static char value_text[16];
        uint8_t percentage = (level * 100) / 255;
        snprintf(value_text, sizeof(value_text), "%u%%", percentage);
        lv_label_set_text(ui_BrightnessValueLabel, value_text);
    }
}

// LVGL 9 observer callback for backlight level updates
static void ui_backlight_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    ui_update_backlight_display(lv_subject_get_int(subject));
}

// Core CW sidetone display update logic
static void ui_update_cw_sidetone_display(int value) {
    int clamped = value;
    if (clamped < 0) clamped = 0;
    if (clamped > 9) clamped = 9;

    if (lv_obj_is_valid(ui_CwSidetoneSlider)) {
        lv_obj_remove_event_cb(ui_CwSidetoneSlider, ui_event_CwSidetoneSlider);
        lv_slider_set_value(ui_CwSidetoneSlider, clamped, LV_ANIM_OFF);
        lv_obj_add_event_cb(ui_CwSidetoneSlider, ui_event_CwSidetoneSlider, LV_EVENT_VALUE_CHANGED, NULL);
    }

    if (lv_obj_is_valid(ui_CwSidetoneValueLabel)) {
        static char value_text[16];
        if (clamped == 0) {
            snprintf(value_text, sizeof(value_text), "Off");
        } else {
            snprintf(value_text, sizeof(value_text), "%d", clamped);
        }
        lv_label_set_text(ui_CwSidetoneValueLabel, value_text);
    }
}

// Core CW pitch display update logic
static void ui_update_cw_pitch_display(int value) {
    int clamped = value;
    if (clamped < 300) clamped = 300;
    if (clamped > 1000) clamped = 1000;
    clamped = ((clamped + 25) / 50) * 50;

    if (lv_obj_is_valid(ui_CwPitchSlider)) {
        lv_obj_remove_event_cb(ui_CwPitchSlider, ui_event_CwPitchSlider);
        s_cw_pitch_internal_update = true;
        lv_slider_set_value(ui_CwPitchSlider, clamped, LV_ANIM_OFF);
        s_cw_pitch_internal_update = false;
        lv_obj_add_event_cb(ui_CwPitchSlider, ui_event_CwPitchSlider, LV_EVENT_VALUE_CHANGED, NULL);
    }

    if (lv_obj_is_valid(ui_CwPitchValueLabel)) {
        static char value_text[16];
        snprintf(value_text, sizeof(value_text), "%d Hz", clamped);
        lv_label_set_text(ui_CwPitchValueLabel, value_text);
    }
}

static void ui_update_radio_menu_item(radio_menu_slider_t *item, int value) {
    if (!item) {
        return;
    }

    int clamped = value;
    if (clamped < item->min) clamped = item->min;
    if (clamped > item->max) clamped = item->max;
    if (item->step > 1) {
        clamped = ((clamped + (item->step / 2)) / item->step) * item->step;
    }

    if (lv_obj_is_valid(item->slider)) {
        lv_obj_remove_event_cb(item->slider, ui_event_RadioMenuSlider);
        lv_slider_set_value(item->slider, clamped, LV_ANIM_OFF);
        lv_obj_add_event_cb(item->slider, ui_event_RadioMenuSlider, LV_EVENT_VALUE_CHANGED, item);
    }

    if (lv_obj_is_valid(item->value_label)) {
        static char value_text[16];
        if (item->show_off && clamped == 0) {
            snprintf(value_text, sizeof(value_text), "Off");
        } else if (item->suffix) {
            snprintf(value_text, sizeof(value_text), "%d%s", clamped, item->suffix);
        } else {
            snprintf(value_text, sizeof(value_text), "%d", clamped);
        }
        lv_label_set_text(item->value_label, value_text);
    }
}

// Event handler for generic radio menu sliders
static void ui_event_RadioMenuSlider(lv_event_t * e)
{
    if (s_radio_menu_internal_update) {
        return;
    }

    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);
    radio_menu_slider_t *item = (radio_menu_slider_t*)lv_event_get_user_data(e);

    if (event_code == LV_EVENT_VALUE_CHANGED && lv_obj_is_valid(target) && item) {
        int32_t raw_value = lv_slider_get_value(target);
        int32_t clamped = raw_value;
        if (clamped < item->min) clamped = item->min;
        if (clamped > item->max) clamped = item->max;
        if (item->step > 1) {
            clamped = ((clamped + (item->step / 2)) / item->step) * item->step;
        }

        if (clamped != raw_value) {
            s_radio_menu_internal_update = true;
            lv_slider_set_value(target, clamped, LV_ANIM_OFF);
            s_radio_menu_internal_update = false;
        }

        if (lv_obj_is_valid(item->value_label)) {
            static char value_text[16];
            if (item->show_off && clamped == 0) {
                snprintf(value_text, sizeof(value_text), "Off");
            } else if (item->suffix) {
                snprintf(value_text, sizeof(value_text), "%ld%s", (long)clamped, item->suffix);
            } else {
                snprintf(value_text, sizeof(value_text), "%ld", (long)clamped);
            }
            lv_label_set_text(item->value_label, value_text);
        }

        ui_send_ex_menu_value(item->menu, (int)clamped);
        if (item->subject) {
            lv_subject_set_int(item->subject, clamped);
        }
    }
}

// LVGL 9 observer callback for CW sidetone volume updates
static void ui_cw_sidetone_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    ui_update_cw_sidetone_display(lv_subject_get_int(subject));
}

// LVGL 9 observer callback for CW pitch updates
static void ui_cw_pitch_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    ui_update_cw_pitch_display(lv_subject_get_int(subject));
}

// LVGL 9 observer callback for generic radio menu sliders
static void ui_radio_menu_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    radio_menu_slider_t *item = (radio_menu_slider_t*)lv_observer_get_user_data(observer);
    if (!item) {
        return;
    }
    ui_update_radio_menu_item(item, lv_subject_get_int(subject));
}

// LVGL 9 observer callback for XVTR offset mix toggle updates
static void ui_xvtr_offset_mix_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    int32_t enabled = lv_subject_get_int(subject);
    ui_set_xvtr_offset_mix_enabled(enabled != 0);
}

// LVGL 9 observer callback for Transverter toggle updates
static void ui_transverter_enabled_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    int32_t enabled = lv_subject_get_int(subject);
    ui_set_transverter_enabled(enabled != 0);
}

// Core transverter state display update logic
static void ui_update_transverter_display(void) {
    // Update button appearance based on transverter availability
    transverter_state_t* state = get_transverter_state();
    if (state && state->xo_data.valid && state->ex056_data.valid) {
        // Transverter data is available, enable the button functionality
        lv_obj_remove_state(ui_PlaceholderButton1, LV_STATE_DISABLED);
    } else {
        // No transverter data available - switch remains functional
    }
}

// LVGL 9 observer callback for transverter state updates
static void ui_transverter_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    LV_UNUSED(subject);
    ui_update_transverter_display();
}

// Update antenna button labels with fetched names from antenna_control
static void ui_update_antenna_button_labels(void)
{
    for (int i = 0; i < 8; i++) {
        if (g_antenna_buttons[i].label != NULL) {
            const char *name = antenna_get_name(i + 1);
            if (name != NULL) {
                lv_label_set_text(g_antenna_buttons[i].label, name);
                ESP_LOGD("UI_Antennas", "Updated button %d label to: %s", i + 1, name);
            }
        }
    }
    ESP_LOGI("UI_Antennas", "All antenna button labels refreshed with current names");
}

// Core antenna state update logic (for relay changes)
static void ui_handle_antenna_state_changed(void) {
    // Request fresh antenna status to update all button states correctly
    // This ensures the UI reflects the true system state after relay changes
    if (websocket_client_is_connected()) {
        ESP_LOGD("UI_Antennas", "Requesting fresh antenna status after relay change");
        websocket_client_get_status();
    } else {
        ESP_LOGW("UI_Antennas", "WebSocket not connected - cannot refresh antenna status");
        // If WebSocket is not connected, clear the antenna cache to force UI refresh next time
        g_antenna_cache.cache_valid = false;
    }
}

// Core antenna names update logic
static void ui_handle_antenna_names_updated(void) {
    ESP_LOGI("UI_Antennas", "Antenna names updated from API - refreshing button labels");
    ui_update_antenna_button_labels();
}

// LVGL 9 observer callback for antenna state changes
static void ui_antenna_state_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    LV_UNUSED(subject);
    ui_handle_antenna_state_changed();
}

// LVGL 9 observer callback for antenna names updates
static void ui_antenna_names_observer_cb(lv_observer_t *observer, lv_subject_t *subject) {
    LV_UNUSED(observer);
    LV_UNUSED(subject);
    ui_handle_antenna_names_updated();
}

static void ui_event_PollingButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        // Get the new state from the switch
        lv_obj_t *switch_obj = (lv_obj_t*)lv_event_get_target(e);
        bool is_checked = lv_obj_has_state(switch_obj, LV_STATE_CHECKED);

        // Toggle CAT polling
        cat_polling_set_user_override(is_checked);

        ESP_LOGI("UI_CAT", "CAT Polling toggled: %s", is_checked ? "ENABLED" : "DISABLED");

        // Save to settings (will be persisted when settings save is fixed)
        // For now, just log the change
        ESP_LOGD("UI_CAT", "Polling state change will be saved when NVS heap issue is resolved");
    }
}

// Antenna switch handler - NVS save disabled due to ESP32-S3 RGB LCD hardware limitation
// Flash writes cause display corruption, no known workaround works reliably
// See: https://github.com/espressif/esp-idf/issues/10010
// Setting is applied immediately but won't persist across reboots
static void ui_event_AntennaSwitchButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *switch_obj = (lv_obj_t*)lv_event_get_target(e);
        bool is_checked = lv_obj_has_state(switch_obj, LV_STATE_CHECKED);

        ESP_LOGI("UI_Antennas", "Antenna switch toggled: %s",
                 is_checked ? "ENABLED" : "DISABLED");

        if (is_checked) {
            // Initialize and start WebSocket client
            esp_err_t ret = websocket_client_init();
            if (ret == ESP_OK) {
                ret = websocket_client_start();
                if (ret == ESP_OK) {
                    ESP_LOGI("UI_Antennas", "WebSocket client started successfully");
                    // Request initial status after connection establishes
                    // (will be handled by connection event callback)
                } else {
                    ESP_LOGE("UI_Antennas", "Failed to start WebSocket client: %s", esp_err_to_name(ret));
                }
            } else if (ret == ESP_ERR_INVALID_STATE) {
                // Already initialized, just start it
                ret = websocket_client_start();
                if (ret == ESP_OK) {
                    ESP_LOGI("UI_Antennas", "WebSocket client restarted");
                }
            } else {
                ESP_LOGE("UI_Antennas", "Failed to initialize WebSocket client: %s", esp_err_to_name(ret));
            }
        } else {
            // Stop WebSocket client
            esp_err_t ret = websocket_client_stop();
            if (ret == ESP_OK) {
                ESP_LOGI("UI_Antennas", "WebSocket client stopped");
            } else {
                ESP_LOGW("UI_Antennas", "Failed to stop WebSocket client: %s", esp_err_to_name(ret));
            }

            // Invalidate antenna cache since we're disconnected
            g_antenna_cache.cache_valid = false;
        }
    }
}

static void ui_event_AIModeButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    
    if (event_code == LV_EVENT_CLICKED) {
        cat_ai_mode_t current_mode = cat_polling_get_ai_mode();
        cat_ai_mode_t new_mode;
        
        // Cycle through AI0, AI2, AI4
        switch (current_mode) {
            case AI_MODE_OFF:
                new_mode = AI_MODE_ON;
                break;
            case AI_MODE_ON:
                new_mode = AI_MODE_ON_BACKUP;
                break;
            case AI_MODE_ON_BACKUP:
                new_mode = AI_MODE_OFF;
                break;
            default: // Including AI_MODE_UNKNOWN
                new_mode = AI_MODE_OFF;
                break;
        }
        
        // Set expected AI mode first (for monitoring)
        cat_polling_set_expected_ai_mode(new_mode);
        
        // Set the new AI mode on the radio
        esp_err_t ret = cat_polling_set_ai_mode(new_mode);
        if (ret == ESP_OK) {
            // Track setting change for save-on-exit
            current_settings.ai_mode = new_mode;
            current_settings.settings_changed = true;
            
            // The button appearance will be updated by the message handler
            // triggered by update_polling_state() inside cat_polling_set_ai_mode.
            ESP_LOGI("UI_AI", "AI mode set to: AI%d", new_mode);
        } else {
            ESP_LOGW("UI_AI", "Failed to set AI mode: %s", esp_err_to_name(ret));
        }
    }
}

// Helper to update screensaver button appearance based on current state
static void ui_update_screensaver_button_style(screensaver_timeout_t timeout) {
    if (!lv_obj_is_valid(ui_ScreensaverButton) || !lv_obj_is_valid(ui_ScreensaverLabel)) {
        return;
    }

    const char *timeout_str;
    uint32_t bg_color;

    switch (timeout) {
        case SCREENSAVER_5_MIN:
            timeout_str = "5 min";
            bg_color = COLOR_BLUE_PRIMARY;  // Enabled - blue
            break;
        case SCREENSAVER_10_MIN:
            timeout_str = "10 min";
            bg_color = COLOR_BLUE_PRIMARY;
            break;
        case SCREENSAVER_15_MIN:
            timeout_str = "15 min";
            bg_color = COLOR_BLUE_PRIMARY;
            break;
        case SCREENSAVER_30_MIN:
            timeout_str = "30 min";
            bg_color = COLOR_BLUE_PRIMARY;
            break;
        case SCREENSAVER_DISABLED:
        default:
            timeout_str = "Disabled";
            bg_color = COLOR_BG_MEDIUM;  // Disabled - dark gray
            break;
    }

    lv_label_set_text(ui_ScreensaverLabel, timeout_str);
    lv_obj_set_style_bg_color(ui_ScreensaverButton, lv_color_hex(bg_color), LV_PART_MAIN | LV_STATE_DEFAULT);
}

// Screensaver timeout button handler - cycles through timeout options
static void ui_event_ScreensaverButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_CLICKED) {
        // Cycle through screensaver timeout options: 5→10→15→30→Disabled→5
        screensaver_timeout_t current = screensaver_get_timeout();
        screensaver_timeout_t next;

        switch (current) {
            case SCREENSAVER_5_MIN:
                next = SCREENSAVER_10_MIN;
                break;
            case SCREENSAVER_10_MIN:
                next = SCREENSAVER_15_MIN;
                break;
            case SCREENSAVER_15_MIN:
                next = SCREENSAVER_30_MIN;
                break;
            case SCREENSAVER_30_MIN:
                next = SCREENSAVER_DISABLED;
                break;
            case SCREENSAVER_DISABLED:
            default:
                next = SCREENSAVER_5_MIN;
                break;
        }

        // Update screensaver timeout
        screensaver_set_timeout(next);

        // Update button label and color
        ui_update_screensaver_button_style(next);
        ESP_LOGI("UI_Screensaver", "Screensaver timeout set to: %d min", (int)next);
    }
}

// Function to update polling UI controls based on current state
void ui_update_polling_controls(void) {
    // Update polling switch based on user override state
    bool user_override = cat_polling_get_user_override();
    if (lv_obj_is_valid(ui_PlaceholderButton2)) {
        if (user_override) {
            lv_obj_add_state(ui_PlaceholderButton2, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_PlaceholderButton2, LV_STATE_CHECKED);
        }
    }
    
    // Update AI mode button
    cat_ai_mode_t ai_mode = cat_polling_get_ai_mode();
    if (lv_obj_is_valid(ui_PlaceholderButton3) && lv_obj_is_valid(ui_PlaceholderLabel3)) {
        switch (ai_mode) {
            case AI_MODE_OFF:
                lv_obj_set_style_bg_color(ui_PlaceholderButton3, lv_color_hex(0xFF4444), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_label_set_text(ui_PlaceholderLabel3, "AI0 (OFF)");
                break;
            case AI_MODE_ON:
                lv_obj_set_style_bg_color(ui_PlaceholderButton3, lv_color_hex(0x007AFF), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_label_set_text(ui_PlaceholderLabel3, "AI2 (ON)");
                break;
            case AI_MODE_ON_BACKUP:
                lv_obj_set_style_bg_color(ui_PlaceholderButton3, lv_color_hex(0x00AA00), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_label_set_text(ui_PlaceholderLabel3, "AI4 (BCK)");
                break;
            case AI_MODE_UNKNOWN:
            default:
                lv_obj_set_style_bg_color(ui_PlaceholderButton3, lv_color_hex(0x4A4A4A), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_label_set_text(ui_PlaceholderLabel3, "Unknown");
                break;
        }
    }
}

void ui_event_PeakHoldSwitch(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        bool is_checked = lv_obj_has_state(target, LV_STATE_CHECKED);
        
        // Track setting change for save-on-exit
        current_settings.peak_hold_enabled = is_checked;
        current_settings.settings_changed = true;
        
        // ESP_LOGI("UI_Screen2", "Peak Hold Switch toggled: %s", is_checked ? "ON" : "OFF");
        lv_subject_set_int(&radio_peak_hold_enabled_subject, is_checked ? 1 : 0);
    }
}

void ui_event_PeakHoldDurationSlider(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        int32_t value = lv_slider_get_value(target);
        uint32_t duration_ms = (uint32_t)value;
        
        // Track setting change for save-on-exit
        current_settings.peak_hold_duration_ms = duration_ms;
        current_settings.settings_changed = true;
        
        // Update the value label to show current ms value
        if (lv_obj_is_valid(ui_PeakHoldDurationValueLabel)) {
            static char value_text[16];
            snprintf(value_text, sizeof(value_text), "%lums", duration_ms);
            lv_label_set_text(ui_PeakHoldDurationValueLabel, value_text);
        }
        
        // Update Screen1 peak hold timing via observer
        lv_subject_set_int(&radio_peak_hold_duration_subject, (int32_t)duration_ms);

        ESP_LOGI("UI_Screen2", "Peak Hold Duration changed to: %lu ms", duration_ms);
    }
}

void ui_event_PepToggleSwitch(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        bool is_checked = lv_obj_has_state(target, LV_STATE_CHECKED);
        
        // Update current settings
        current_settings.pep_enabled = is_checked;
        current_settings.settings_changed = true;
        
        // Update PEP system
        pep_enable(is_checked);
        
        ESP_LOGD("UI_Screen2", "PEP Power Display toggled: %s", is_checked ? "ON" : "OFF");
    }
}

// Timer callback to restore AI mode after screen initialization
static void ai_mode_restore_timer_cb(lv_timer_t* timer) {
    cat_ai_mode_t* mode = (cat_ai_mode_t*)lv_timer_get_user_data(timer);
    
    // Set expected AI mode first (for monitoring)
    cat_polling_set_expected_ai_mode(*mode);
    
    esp_err_t ai_ret = cat_polling_set_ai_mode(*mode);
    if (ai_ret != ESP_OK) {
        ESP_LOGW("UI_Settings", "Failed to restore AI mode: %s", esp_err_to_name(ai_ret));
    } else {
        ESP_LOGI("UI_Settings", "AI mode restored to: %d", *mode);
    }
    lv_timer_del(timer);  // Delete one-shot timer
}

// Load saved settings and apply them to UI and system state
static void ui_load_saved_settings(void) {
    user_settings_t settings;
    esp_err_t ret = settings_load(&settings);
    
    if (ret != ESP_OK) {
        ESP_LOGW("UI_Settings", "Failed to load settings, using defaults: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI("UI_Settings", "Loading saved settings");
    
    // Initialize current_settings tracking
    current_settings.xvtr_offset_mix_enabled = settings.xvtr_offset_mix_enabled;
    current_settings.cat_polling_enabled = settings.cat_polling_enabled;
    current_settings.ai_mode = settings.ai_mode;
    current_settings.peak_hold_enabled = settings.peak_hold_enabled;
    current_settings.peak_hold_duration_ms = settings.peak_hold_duration_ms;
    current_settings.pep_enabled = settings.pep_enabled;
    current_settings.smeter_averaging_enabled = settings.smeter_averaging_enabled;
    current_settings.antenna_switch_enabled = settings.antenna_switch_enabled;
    current_settings.settings_changed = false;
    
    // Apply XVTR offset mix setting
    xvtr_offset_mix_enabled = settings.xvtr_offset_mix_enabled;
    if (lv_obj_is_valid(ui_PlaceholderButton1)) {
        if (xvtr_offset_mix_enabled) {
            lv_obj_add_state(ui_PlaceholderButton1, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_PlaceholderButton1, LV_STATE_CHECKED);
        }
    }
    
    // Apply CAT polling setting
    cat_polling_set_user_override(settings.cat_polling_enabled);
    if (lv_obj_is_valid(ui_PlaceholderButton2)) {
        if (settings.cat_polling_enabled) {
            lv_obj_add_state(ui_PlaceholderButton2, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_PlaceholderButton2, LV_STATE_CHECKED);
        }
    }
    
    // Apply AI mode setting (only if it's a valid known mode, not UNKNOWN)
    // Use a short timer to defer this until after screen initialization is complete
    if (settings.ai_mode != AI_MODE_UNKNOWN) {
        static cat_ai_mode_t deferred_ai_mode;
        deferred_ai_mode = settings.ai_mode;
        
        // Create one-shot timer to restore AI mode after 100ms
        lv_timer_t* ai_restore_timer = lv_timer_create(ai_mode_restore_timer_cb, 100, &deferred_ai_mode);
        lv_timer_set_repeat_count(ai_restore_timer, 1);
    }
    
    // Apply peak hold setting
    if (settings.peak_hold_enabled) {
        lv_obj_add_state(ui_PeakHoldSwitch, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(ui_PeakHoldSwitch, LV_STATE_CHECKED);
    }
    // Update Screen1 state via observer
    lv_subject_set_int(&radio_peak_hold_enabled_subject, settings.peak_hold_enabled ? 1 : 0);
    
    // Apply peak hold duration setting
    if (lv_obj_is_valid(ui_PeakHoldDurationSlider)) {
        lv_slider_set_value(ui_PeakHoldDurationSlider, settings.peak_hold_duration_ms, LV_ANIM_OFF);
        
        // Update the duration label
        if (lv_obj_is_valid(ui_PeakHoldDurationLabel)) {
            static char duration_text[32];
            snprintf(duration_text, sizeof(duration_text), "Peak Hold Duration: %lums", settings.peak_hold_duration_ms);
            lv_label_set_text(ui_PeakHoldDurationLabel, duration_text);
        }
    }
    // Update Screen1 peak hold timing via observer
    lv_subject_set_int(&radio_peak_hold_duration_subject, (int32_t)settings.peak_hold_duration_ms);

    // Apply PEP enabled setting
    if (settings.pep_enabled) {
        lv_obj_add_state(ui_PepToggleSwitch, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(ui_PepToggleSwitch, LV_STATE_CHECKED);
    }
    // Update PEP system
    pep_enable(settings.pep_enabled);
    
    // Apply S-meter averaging setting
    if (lv_obj_is_valid(ui_SMeterAveragingSwitch)) {
        if (settings.smeter_averaging_enabled) {
            lv_obj_add_state(ui_SMeterAveragingSwitch, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_SMeterAveragingSwitch, LV_STATE_CHECKED);
        }
    }
    // Update Screen1 averaging behavior via observer
    lv_subject_set_int(&radio_smeter_averaging_subject, settings.smeter_averaging_enabled ? 1 : 0);

    // Apply antenna switch setting
    if (lv_obj_is_valid(ui_AntennaSwitchButton)) {
        if (settings.antenna_switch_enabled) {
            lv_obj_add_state(ui_AntennaSwitchButton, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(ui_AntennaSwitchButton, LV_STATE_CHECKED);
        }
    }

    ESP_LOGI("UI_Settings", "Settings loaded and applied successfully");
}

void ui_Screen2_screen_init(void) {
    
    // If already created, avoid rebuilding to keep init work minimal
    if (ui_Screen2 && lv_obj_is_valid(ui_Screen2)) {
        ESP_LOGI("UI_Screen2", "Screen2 already exists - skipping init");
        return;
    }
    
    ESP_LOGI("UI_Screen2", "Creating Screen2 UI elements...");
    // Base screen
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_Screen2, (lv_obj_flag_t)(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM));
    // Match Screen1 palette: keep a deep background for content area
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x101418), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Create a full-screen opaque background panel to prevent Screen1 bleed-through
    // This sits behind the menu and ensures complete coverage without modifying menu styling
    lv_obj_t *bg_panel = lv_obj_create(ui_Screen2);
    lv_obj_remove_style_all(bg_panel);
    lv_obj_set_size(bg_panel, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(bg_panel, lv_color_hex(0x101418), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(bg_panel, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_remove_flag(bg_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Create LVGL Complex Menu
    ESP_LOGI("UI_Screen2", "Creating menu on Screen2=%p", (void*)ui_Screen2);
    ui_Menu = lv_menu_create(ui_Screen2);
    ESP_LOGI("UI_Screen2", "Menu created: ui_Menu=%p", (void*)ui_Menu);

    if (ui_Menu == NULL) {
        ESP_LOGE("UI_Screen2", "FATAL: lv_menu_create returned NULL!");
        return;
    }

    // DEBUG: Check menu internal structure
    lv_menu_t* menu_priv = (lv_menu_t*)ui_Menu;
    ESP_LOGI("UI_Screen2", "Menu internals: storage=%p main=%p main_header=%p",
             (void*)menu_priv->storage, (void*)menu_priv->main, (void*)menu_priv->main_header);

    lv_obj_set_size(ui_Menu, LV_PCT(100), LV_PCT(100));
    lv_obj_center(ui_Menu);
    lv_menu_set_mode_header(ui_Menu, LV_MENU_HEADER_TOP_FIXED);
    // Enable proper LVGL menu back button instead of custom overlay
    lv_menu_set_mode_root_back_button(ui_Menu, LV_MENU_ROOT_BACK_BUTTON_ENABLED);
    lv_obj_add_event_cb(ui_Menu, back_event_handler, LV_EVENT_CLICKED, ui_Menu);

    // Style the menu header and back button properly
    lv_obj_t *header = lv_menu_get_main_header(ui_Menu);
    ESP_LOGI("UI_Screen2", "Got menu header: header=%p", (void*)header);
    lv_obj_set_style_bg_color(header, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(header, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Set header font that supports LVGL symbols (for back button arrow LV_SYMBOL_LEFT)
    lv_obj_set_style_text_font(header, ui_symbol_font_lg(), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Ensure the menu inherits the main background for the content panel
    lv_obj_set_style_bg_color(ui_Menu, lv_color_hex(0x101418), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Menu, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Create root page with proper sidebar styling
    ESP_LOGI("UI_Screen2", "About to create menu page with ui_Menu=%p", (void*)ui_Menu);
    lv_obj_t *root = lv_menu_page_create(ui_Menu, "Settings");
    lv_obj_set_style_pad_hor(root, lv_obj_get_style_pad_left(lv_menu_get_main_header(ui_Menu), LV_PART_MAIN), 0);

    // Enhanced sidebar styling with width for text content
    lv_obj_set_width(root, ui_sx(225)); // Wider for better text spacing
    lv_obj_set_scroll_dir(root, LV_DIR_VER); // Disable horizontal scrolling on sidebar

    // Clean sidebar styling without border
    lv_obj_set_style_bg_color(root, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(root, 0, LV_PART_MAIN | LV_STATE_DEFAULT);  // Remove blue border
    lv_obj_set_style_shadow_width(root, 8, LV_PART_MAIN | LV_STATE_DEFAULT);  // Subtle shadow
    lv_obj_set_style_shadow_color(root, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);  // Dark shadow
    lv_obj_set_style_shadow_opa(root, 30, LV_PART_MAIN | LV_STATE_DEFAULT);   // Subtle opacity
    lv_obj_set_style_radius(root, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *root_sec = lv_menu_section_create(root);
    // Add padding for better spacing
    lv_obj_set_style_pad_all(root_sec, ui_sx(15), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(root, ui_sy(18), 0);      // extra space above the first section
    lv_obj_set_style_pad_top(root_sec, ui_sy(18), 0);  // and above the section contents

    // Sub pages (store for deferred population)
    ui_MenuPageDisplay = lv_menu_page_create(ui_Menu, "Display");
    ui_MenuPageRadioMenu = lv_menu_page_create(ui_Menu, "Radio Menu");
    ui_MenuPageCat = lv_menu_page_create(ui_Menu, "CAT & Transverter");
    ui_MenuPageMeter = lv_menu_page_create(ui_Menu, "Meter");
    ui_MenuPageAntennas = lv_menu_page_create(ui_Menu, "Antennas");
    ui_MenuPageMacros = lv_menu_page_create(ui_Menu, "Macros");
    ui_MenuPageSystem = lv_menu_page_create(ui_Menu, "System");

    // Disable horizontal scrolling on all menu pages to prevent unwanted shifts
    lv_obj_set_scroll_dir(ui_MenuPageDisplay, LV_DIR_VER);
    lv_obj_set_scroll_dir(ui_MenuPageRadioMenu, LV_DIR_VER);
    lv_obj_set_scroll_dir(ui_MenuPageCat, LV_DIR_VER);
    lv_obj_set_scroll_dir(ui_MenuPageMeter, LV_DIR_VER);
    lv_obj_set_scroll_dir(ui_MenuPageAntennas, LV_DIR_VER);
    lv_obj_set_scroll_dir(ui_MenuPageMacros, LV_DIR_VER);
    lv_obj_set_scroll_dir(ui_MenuPageSystem, LV_DIR_VER);

    // Create sidebar menu items with themed colored LVGL symbols
    lv_obj_t *it_display = create_text_colored(root_sec, LV_SYMBOL_IMAGE, "Display", COLOR_AQUA_LIGHT);
    lv_menu_set_load_page_event(ui_Menu, it_display, ui_MenuPageDisplay);

    lv_obj_t *it_radio_menu = create_text_colored(root_sec, LV_SYMBOL_AUDIO, "Radio Menu", COLOR_ORANGE_VIBRANT);
    lv_menu_set_load_page_event(ui_Menu, it_radio_menu, ui_MenuPageRadioMenu);

    lv_obj_t *it_cat = create_text_colored(root_sec, LV_SYMBOL_LIST, "CAT & Transverter", COLOR_SELECTIVE_YELLOW);
    lv_menu_set_load_page_event(ui_Menu, it_cat, ui_MenuPageCat);

    lv_obj_t *it_meter = create_text_colored(root_sec, LV_SYMBOL_BARS, "Meter", COLOR_SUCCESS_GREEN);
    lv_menu_set_load_page_event(ui_Menu, it_meter, ui_MenuPageMeter);

    lv_obj_t *it_antennas = create_text_colored(root_sec, LV_SYMBOL_WIFI, "Antennas", COLOR_PURPLE_ACCENT);
    lv_menu_set_load_page_event(ui_Menu, it_antennas, ui_MenuPageAntennas);

    lv_obj_t *it_macros = create_text_colored(root_sec, LV_SYMBOL_SHUFFLE, "Macros", COLOR_SELECTIVE_YELLOW);
    lv_menu_set_load_page_event(ui_Menu, it_macros, ui_MenuPageMacros);

    lv_obj_t *it_system = create_text_colored(root_sec, LV_SYMBOL_SETTINGS, "System", COLOR_ARGENTINIAN_BLUE);
    lv_menu_set_load_page_event(ui_Menu, it_system, ui_MenuPageSystem);

    // Set the sidebar and default main page
    lv_menu_set_sidebar_page(ui_Menu, root);
    lv_menu_set_page(ui_Menu, ui_MenuPageDisplay);

    // Disable horizontal scrolling on menu and its internal components to prevent
    // unwanted horizontal shifts when interacting with switches/buttons
    lv_obj_set_scroll_dir(ui_Menu, LV_DIR_VER);
    if (menu_priv->main) {
        lv_obj_set_scroll_dir(menu_priv->main, LV_DIR_VER);
    }
    if (menu_priv->sidebar) {
        lv_obj_set_scroll_dir(menu_priv->sidebar, LV_DIR_VER);
    }

    // Enhanced sidebar header with collapse chevron
    lv_obj_t *sidebar_header = lv_menu_get_sidebar_header(ui_Menu);
    if (sidebar_header) {
        // Use flex layout for header: [Settings text] ... [chevron]
        lv_obj_set_flex_flow(sidebar_header, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(sidebar_header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        // Style the header container
        lv_obj_set_style_pad_ver(sidebar_header, ui_sy(12), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_hor(sidebar_header, ui_sx(15), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(sidebar_header, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);

        // Make the entire header clickable for sidebar toggle
        lv_obj_add_flag(sidebar_header, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(sidebar_header, lv_color_hex(0x3a3a4a), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(sidebar_header, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_PRESSED);

        // Find the existing "Settings" label created by lv_menu and style it
        lv_obj_t *header_label = lv_obj_get_child(sidebar_header, 0);
        if (header_label && lv_obj_check_type(header_label, &lv_label_class)) {
            lv_obj_set_style_text_font(header_label, ui_font30(), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(header_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
        }

        // Add collapse chevron on the right side of header
        g_sidebar_header_chevron = lv_label_create(sidebar_header);
        lv_label_set_text(g_sidebar_header_chevron, LV_SYMBOL_LEFT);
        lv_obj_set_style_text_font(g_sidebar_header_chevron, ui_symbol_font_sm(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(g_sidebar_header_chevron, lv_color_hex(0x888888), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(g_sidebar_header_chevron, ui_sx(8), LV_PART_MAIN | LV_STATE_DEFAULT);

        // Connect header click to sidebar toggle
        lv_obj_add_event_cb(sidebar_header, ui_sidebar_toggle_cb, LV_EVENT_CLICKED, NULL);
    }

    // Note: Skip back button customization for now to avoid crashes
    // The default LVGL menu back button should work, even if smaller

    // Create expand button on SCREEN (not menu) - fixed position on left edge
    // Parenting to screen ensures it stays in place when sidebar hides
    g_sidebar_expand_btn = lv_btn_create(ui_Screen2);
    lv_obj_set_size(g_sidebar_expand_btn, ui_sx(32), ui_sy(80));
    lv_obj_align(g_sidebar_expand_btn, LV_ALIGN_LEFT_MID, 0, 0);

    // Match sidebar background with subtle right border (drawer handle look)
    lv_obj_set_style_bg_color(g_sidebar_expand_btn, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(g_sidebar_expand_btn, lv_color_hex(0x3a3a4a), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(g_sidebar_expand_btn, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(g_sidebar_expand_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Right border as visual separator
    lv_obj_set_style_border_width(g_sidebar_expand_btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(g_sidebar_expand_btn, lv_color_hex(0x444444), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(g_sidebar_expand_btn, LV_BORDER_SIDE_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Subtle shadow for depth
    lv_obj_set_style_shadow_width(g_sidebar_expand_btn, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(g_sidebar_expand_btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(g_sidebar_expand_btn, LV_OPA_40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(g_sidebar_expand_btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(g_sidebar_expand_btn, ui_sidebar_toggle_cb, LV_EVENT_CLICKED, NULL);

    // Chevron icon pointing right (expand direction)
    lv_obj_t *expand_lbl = lv_label_create(g_sidebar_expand_btn);
    lv_label_set_text(expand_lbl, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_font(expand_lbl, ui_symbol_font_sm(), 0);
    lv_obj_set_style_text_color(expand_lbl, lv_color_hex(0x888888), 0);
    lv_obj_center(expand_lbl);

    // Start hidden - only show when sidebar is collapsed
    lv_obj_add_flag(g_sidebar_expand_btn, LV_OBJ_FLAG_HIDDEN);

    // ===== Build all pages immediately with enhanced styling =====
    // Enhanced Display page
    lv_obj_t *sec_disp = lv_menu_section_create(ui_MenuPageDisplay);
    lv_obj_set_style_bg_color(ui_MenuPageDisplay, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(sec_disp, ui_sx(20), LV_PART_MAIN | LV_STATE_DEFAULT);

    const lv_obj_t *row_bright = create_slider(sec_disp, NULL, "Brightness", 0, 255, lcd_get_backlight_level());
    ui_BrightnessSlider = lv_obj_get_child(row_bright, -1); // Get the slider from the container
    ui_BrightnessValueLabel = lv_obj_get_child(row_bright, -2); // Get the value label from the container
    lv_obj_add_event_cb(ui_BrightnessSlider, event_handler_brightness_slider, LV_EVENT_VALUE_CHANGED, NULL);

    // Update the initial value label to show percentage
    if (lv_obj_is_valid(ui_BrightnessValueLabel)) {
        static char initial_value_text[16];
        uint8_t current_level = lcd_get_backlight_level();
        uint8_t percentage = (current_level * 100) / 255;
        snprintf(initial_value_text, sizeof(initial_value_text), "%u%%", percentage);
        lv_label_set_text(ui_BrightnessValueLabel, initial_value_text);
    }

    // Screensaver timeout button (cycles through options)
    // Create with placeholder text - will be updated immediately after
    lv_obj_t *row_screensaver = create_button(sec_disp, NULL, "Screensaver", "---");
    ui_ScreensaverButton = lv_obj_get_child(row_screensaver, -1);
    ui_ScreensaverLabel = lv_obj_get_child(ui_ScreensaverButton, 0);
    lv_obj_add_event_cb(ui_ScreensaverButton, ui_event_ScreensaverButton_clicked, LV_EVENT_CLICKED, NULL);
    // Set initial button text and color based on current screensaver setting
    ui_update_screensaver_button_style(screensaver_get_timeout());

    // Radio Menu page (two-column layout)
    lv_obj_t *sec_radio = lv_menu_section_create(ui_MenuPageRadioMenu);
    lv_obj_set_style_bg_color(ui_MenuPageRadioMenu, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(sec_radio, ui_sx(20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(sec_radio, ui_sy(14), LV_PART_MAIN | LV_STATE_DEFAULT);

    int sidetone_init = lv_subject_get_int(&radio_cw_sidetone_volume_subject);
    create_radio_menu_row(sec_radio, "CW Sidetone", 0, 9, sidetone_init, &ui_CwSidetoneSlider, &ui_CwSidetoneValueLabel);
    lv_obj_add_event_cb(ui_CwSidetoneSlider, ui_event_CwSidetoneSlider, LV_EVENT_VALUE_CHANGED, NULL);
    ui_update_cw_sidetone_display(sidetone_init);

    int pitch_init = lv_subject_get_int(&radio_cw_pitch_hz_subject);
    create_radio_menu_row(sec_radio, "CW TX Pitch", 300, 1000, pitch_init, &ui_CwPitchSlider, &ui_CwPitchValueLabel);
    lv_obj_add_event_cb(ui_CwPitchSlider, ui_event_CwPitchSlider, LV_EVENT_VALUE_CHANGED, NULL);
    ui_update_cw_pitch_display(pitch_init);

    int fm_gain_init = lv_subject_get_int(&radio_fm_mic_gain_subject);
    create_radio_menu_row(sec_radio, "FM Mic Gain", 1, 3, fm_gain_init, &ui_FmMicGainSlider, &ui_FmMicGainValueLabel);
    g_radio_menu_fm_mic_gain.slider = ui_FmMicGainSlider;
    g_radio_menu_fm_mic_gain.value_label = ui_FmMicGainValueLabel;
    lv_obj_add_event_cb(ui_FmMicGainSlider, ui_event_RadioMenuSlider, LV_EVENT_VALUE_CHANGED, &g_radio_menu_fm_mic_gain);
    ui_update_radio_menu_item(&g_radio_menu_fm_mic_gain, fm_gain_init);

    int usb_in_init = lv_subject_get_int(&radio_usb_input_level_subject);
    create_radio_menu_row(sec_radio, "USB Audio In", 0, 9, usb_in_init, &ui_UsbAudioInputSlider, &ui_UsbAudioInputValueLabel);
    g_radio_menu_usb_input.slider = ui_UsbAudioInputSlider;
    g_radio_menu_usb_input.value_label = ui_UsbAudioInputValueLabel;
    lv_obj_add_event_cb(ui_UsbAudioInputSlider, ui_event_RadioMenuSlider, LV_EVENT_VALUE_CHANGED, &g_radio_menu_usb_input);
    ui_update_radio_menu_item(&g_radio_menu_usb_input, usb_in_init);

    int usb_out_init = lv_subject_get_int(&radio_usb_output_level_subject);
    create_radio_menu_row(sec_radio, "USB Audio Out", 0, 9, usb_out_init, &ui_UsbAudioOutputSlider, &ui_UsbAudioOutputValueLabel);
    g_radio_menu_usb_output.slider = ui_UsbAudioOutputSlider;
    g_radio_menu_usb_output.value_label = ui_UsbAudioOutputValueLabel;
    lv_obj_add_event_cb(ui_UsbAudioOutputSlider, ui_event_RadioMenuSlider, LV_EVENT_VALUE_CHANGED, &g_radio_menu_usb_output);
    ui_update_radio_menu_item(&g_radio_menu_usb_output, usb_out_init);

    int acc2_in_init = lv_subject_get_int(&radio_acc2_input_level_subject);
    create_radio_menu_row(sec_radio, "ACC2 AF In", 0, 9, acc2_in_init, &ui_Acc2InputSlider, &ui_Acc2InputValueLabel);
    g_radio_menu_acc2_input.slider = ui_Acc2InputSlider;
    g_radio_menu_acc2_input.value_label = ui_Acc2InputValueLabel;
    lv_obj_add_event_cb(ui_Acc2InputSlider, ui_event_RadioMenuSlider, LV_EVENT_VALUE_CHANGED, &g_radio_menu_acc2_input);
    ui_update_radio_menu_item(&g_radio_menu_acc2_input, acc2_in_init);

    int acc2_out_init = lv_subject_get_int(&radio_acc2_output_level_subject);
    create_radio_menu_row(sec_radio, "ACC2 AF Out", 0, 9, acc2_out_init, &ui_Acc2OutputSlider, &ui_Acc2OutputValueLabel);
    g_radio_menu_acc2_output.slider = ui_Acc2OutputSlider;
    g_radio_menu_acc2_output.value_label = ui_Acc2OutputValueLabel;
    lv_obj_add_event_cb(ui_Acc2OutputSlider, ui_event_RadioMenuSlider, LV_EVENT_VALUE_CHANGED, &g_radio_menu_acc2_output);
    ui_update_radio_menu_item(&g_radio_menu_acc2_output, acc2_out_init);

    // Enhanced CAT page
    lv_obj_t *sec_cat = lv_menu_section_create(ui_MenuPageCat);
    lv_obj_set_style_bg_color(ui_MenuPageCat, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(sec_cat, ui_sx(20), LV_PART_MAIN | LV_STATE_DEFAULT);

    // XVTR offset mix with icon
    lv_obj_t *row_xvtr = create_switch(sec_cat, NULL, "XVTR Offset Mix", false);
    ui_PlaceholderButton1 = lv_obj_get_child(row_xvtr, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_PlaceholderButton1, ui_event_PlaceholderButton1_clicked, LV_EVENT_VALUE_CHANGED, NULL);

    // Transverter (UIXD) with icon
    lv_obj_t *row_transverter = create_switch(sec_cat, NULL, "Transverter", false);
    ui_TransverterButton = lv_obj_get_child(row_transverter, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_TransverterButton, ui_event_TransverterButton_clicked, LV_EVENT_VALUE_CHANGED, NULL);

    // Panel Data (UIDE) with icon - controls data updates from panel
    lv_obj_t *row_panel_data = create_switch(sec_cat, NULL, "Panel Data", true); // Default: enabled
    ui_PanelDataButton = lv_obj_get_child(row_panel_data, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_PanelDataButton, ui_event_PanelDataButton_clicked, LV_EVENT_VALUE_CHANGED, NULL);

    // CAT polling with icon
    lv_obj_t *row_poll = create_switch(sec_cat, NULL, "CAT Polling", false);
    ui_PlaceholderButton2 = lv_obj_get_child(row_poll, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_PlaceholderButton2, ui_event_PollingButton_clicked, LV_EVENT_VALUE_CHANGED, NULL);

    // AI mode with icon (special button handling)
    lv_obj_t *row_ai = create_button(sec_cat, NULL, "AI Mode", "Unknown");
    ui_PlaceholderButton3 = lv_obj_get_child(row_ai, -1); // Get the button from the container
    ui_PlaceholderLabel3 = lv_obj_get_child(ui_PlaceholderButton3, 0); // Get the label from the button
    lv_obj_add_event_cb(ui_PlaceholderButton3, ui_event_AIModeButton_clicked, LV_EVENT_CLICKED, NULL);

    // Enhanced Meter page
    lv_obj_t *sec_meter = lv_menu_section_create(ui_MenuPageMeter);
    lv_obj_set_style_bg_color(ui_MenuPageMeter, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(sec_meter, ui_sx(20), LV_PART_MAIN | LV_STATE_DEFAULT);

    // S-meter averaging with icon
    lv_obj_t *row_savg = create_switch(sec_meter, NULL, "S-Meter Averaging", true);
    ui_SMeterAveragingSwitch = lv_obj_get_child(row_savg, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_SMeterAveragingSwitch, ui_event_SMeterAveragingSwitch, LV_EVENT_VALUE_CHANGED, NULL);

    // Peak hold toggle with icon
    lv_obj_t *row_phold = create_switch(sec_meter, NULL, "Peak Hold", false);
    ui_PeakHoldSwitch = lv_obj_get_child(row_phold, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_PeakHoldSwitch, ui_event_PeakHoldSwitch, LV_EVENT_VALUE_CHANGED, NULL);

    // Peak hold duration with icon
    lv_obj_t *row_phdur = create_slider(sec_meter, NULL, "Peak Duration", 10, 1000, 100);
    ui_PeakHoldDurationSlider = lv_obj_get_child(row_phdur, -1); // Get the slider from the container
    ui_PeakHoldDurationValueLabel = lv_obj_get_child(row_phdur, -2); // Get the value label from the container
    lv_obj_add_event_cb(ui_PeakHoldDurationSlider, ui_event_PeakHoldDurationSlider, LV_EVENT_VALUE_CHANGED, NULL);

    // PEP display toggle with icon
    lv_obj_t *row_pep = create_switch(sec_meter, NULL, "Peak Envelope Power", true);
    ui_PepToggleSwitch = lv_obj_get_child(row_pep, -1); // Get the switch from the container
    lv_obj_add_event_cb(ui_PepToggleSwitch, ui_event_PepToggleSwitch, LV_EVENT_VALUE_CHANGED, NULL);

    // Enhanced System page
    lv_obj_t *sec_sys = lv_menu_section_create(ui_MenuPageSystem);
    lv_obj_set_style_bg_color(ui_MenuPageSystem, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(sec_sys, ui_sx(20), LV_PART_MAIN | LV_STATE_DEFAULT);

    // Reboot button with icon
    lv_obj_t *row_reboot = create_button(sec_sys, NULL, "System", "Restart");
    ui_RebootButton = lv_obj_get_child(row_reboot, -1); // Get the button from the container
    lv_obj_set_style_bg_color(ui_RebootButton, lv_color_hex(COLOR_RED_LIGHT), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_RebootButton, lv_color_hex(0xFF6666), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_add_event_cb(ui_RebootButton, ui_event_RebootButton, LV_EVENT_CLICKED, NULL);

    // Enhanced Antennas page
    lv_obj_t *sec_antennas = lv_menu_section_create(ui_MenuPageAntennas);
    lv_obj_set_style_bg_color(ui_MenuPageAntennas, lv_color_hex(COLOR_BG_DARK), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(sec_antennas, ui_sx(20), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_remove_flag(sec_antennas, LV_OBJ_FLAG_SCROLLABLE);  // Prevent unwanted scrolling

    // Create container for antenna button grid
    lv_obj_t *antenna_grid = lv_obj_create(sec_antennas);
    lv_obj_set_size(antenna_grid, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_remove_flag(antenna_grid, LV_OBJ_FLAG_SCROLLABLE);  // Prevent unwanted scrolling
    lv_obj_set_style_bg_opa(antenna_grid, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(antenna_grid, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(antenna_grid, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_gap(antenna_grid, ui_sx(10), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    // Set flex layout for grid (2 rows x 4 columns)
    lv_obj_set_flex_flow(antenna_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(antenna_grid, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    // Create 8 antenna buttons with dynamic labels from antenna_control
    for (int i = 0; i < 8; i++) {
        lv_obj_t *btn = lv_button_create(antenna_grid);
        lv_obj_set_size(btn, ui_sx(110), ui_sy(60)); // Touch-friendly size
        lv_obj_set_style_radius(btn, 8, LV_PART_MAIN | LV_STATE_DEFAULT);

        // Default state: Darker gray background, no border (unavailable)
        lv_obj_set_style_bg_color(btn, lv_color_hex(COLOR_BG_DARK_GRAY_BUTTON), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, 30, LV_PART_MAIN | LV_STATE_DEFAULT);

        // Pressed state: Darker gray
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x444444), LV_PART_MAIN | LV_STATE_PRESSED);

        // Available state: Blue background, no border (supports current frequency)
        lv_obj_set_style_bg_color(btn, lv_color_hex(COLOR_BLUE_PRIMARY), LV_PART_MAIN | LV_STATE_USER_1);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_USER_1);
        lv_obj_set_style_shadow_width(btn, 3, LV_PART_MAIN | LV_STATE_USER_1);
        lv_obj_set_style_shadow_opa(btn, 50, LV_PART_MAIN | LV_STATE_USER_1);

        // Active state: Green background, no border (currently selected)
        lv_obj_set_style_bg_color(btn, lv_color_hex(COLOR_EMERALD_GREEN), LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_shadow_width(btn, 6, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_shadow_opa(btn, 80, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(COLOR_EMERALD_GREEN), LV_PART_MAIN | LV_STATE_CHECKED);

        lv_obj_t *label = lv_label_create(btn);
        // Get antenna name from antenna_control (uses fallback chain: API -> NVS -> hardcoded)
        const char *antenna_name = antenna_get_name(i + 1);
        lv_label_set_text(label, antenna_name);
        lv_obj_set_style_text_font(label, ui_btn_small_med_font(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(label);

        // Store button and label references for dynamic updates
        g_antenna_buttons[i].button = btn;
        g_antenna_buttons[i].label = label;

        // Store antenna ID in user data for event handler
        lv_obj_set_user_data(btn, (void*)(uintptr_t)(i + 1));
        lv_obj_add_event_cb(btn, ui_event_AntennaButton_clicked, LV_EVENT_CLICKED, NULL);

        // Don't add LV_OBJ_FLAG_CHECKABLE - we manage states manually for single-selection
    }

    // Create antenna switch enable/disable toggle at the bottom of the page
    lv_obj_t *row_antenna_switch = create_switch(sec_antennas, NULL, "Enable", false);
    ui_AntennaSwitchButton = lv_obj_get_child(row_antenna_switch, -1); // Get the switch from container
    lv_obj_add_event_cb(ui_AntennaSwitchButton, ui_event_AntennaSwitchButton_clicked, LV_EVENT_VALUE_CHANGED, NULL);

    // NVS save task DISABLED due to ESP32-S3 RGB LCD hardware limitation
    // Flash writes cause display corruption regardless of timing/batching
    // See: https://github.com/espressif/esp-idf/issues/10010
    // Settings will not persist across reboots
    ESP_LOGW("UI_Screen2", "NVS save disabled - settings won't persist (display stability)");

    // Apply settings and control sync immediately
    ui_update_polling_controls();
    ui_load_saved_settings();

    // Initialize macro management UI
    ui_macro_populate_list();

    // Subscribe to messages (LVGL 9 native observers only - legacy removed)
    // Transverter state updates
    lv_subject_add_observer_obj(&radio_transverter_state_subject, ui_transverter_observer_cb, ui_Screen2, NULL);
    lv_subject_add_observer_obj(&radio_transverter_xo_subject, ui_transverter_observer_cb, ui_Screen2, NULL);
    lv_subject_add_observer_obj(&radio_transverter_ex056_subject, ui_transverter_observer_cb, ui_Screen2, NULL);

    // CAT polling state updates
    lv_subject_add_observer_obj(&radio_cat_polling_state_subject, ui_polling_observer_cb, ui_Screen2, NULL);

    // Display refresh notifications
    lv_subject_add_observer_obj(&radio_force_refresh_subject, ui_display_refresh_observer_cb, ui_Screen2, NULL);

    // Backlight level updates
    lv_subject_add_observer_obj(&radio_backlight_subject, ui_backlight_observer_cb, ui_Screen2, NULL);

    // CW menu updates
    lv_subject_add_observer_obj(&radio_cw_sidetone_volume_subject, ui_cw_sidetone_observer_cb, ui_Screen2, NULL);
    lv_subject_add_observer_obj(&radio_cw_pitch_hz_subject, ui_cw_pitch_observer_cb, ui_Screen2, NULL);
    lv_subject_add_observer_obj(&radio_fm_mic_gain_subject, ui_radio_menu_observer_cb, ui_Screen2, &g_radio_menu_fm_mic_gain);
    lv_subject_add_observer_obj(&radio_usb_input_level_subject, ui_radio_menu_observer_cb, ui_Screen2, &g_radio_menu_usb_input);
    lv_subject_add_observer_obj(&radio_usb_output_level_subject, ui_radio_menu_observer_cb, ui_Screen2, &g_radio_menu_usb_output);
    lv_subject_add_observer_obj(&radio_acc2_input_level_subject, ui_radio_menu_observer_cb, ui_Screen2, &g_radio_menu_acc2_input);
    lv_subject_add_observer_obj(&radio_acc2_output_level_subject, ui_radio_menu_observer_cb, ui_Screen2, &g_radio_menu_acc2_output);

    // XVTR Offset Mix toggle updates
    lv_subject_add_observer_obj(&radio_xvtr_offset_mix_subject, ui_xvtr_offset_mix_observer_cb, ui_Screen2, NULL);

    // Transverter toggle updates
    lv_subject_add_observer_obj(&radio_transverter_enabled_subject, ui_transverter_enabled_observer_cb, ui_Screen2, NULL);

    // Antenna status updates
    ESP_LOGD("UI_Screen2", "Registering antenna observers");
    lv_subject_add_observer_obj(&radio_antenna_state_subject, ui_antenna_state_observer_cb, ui_Screen2, NULL);
    lv_subject_add_observer_obj(&radio_antenna_names_subject, ui_antenna_names_observer_cb, ui_Screen2, NULL);
    ESP_LOGD("UI_Screen2", "Antenna observers registered successfully");

    // Request CW menu values once UI observers are ready
    cat_request_cw_menu_update();
    
    // Check WebSocket connection status during init
    bool ws_connected = websocket_client_is_connected();
    ESP_LOGI("UI_Screen2", "WebSocket connection status during init: %s", ws_connected ? "CONNECTED" : "DISCONNECTED");
    
    // If WebSocket is connected, actively request antenna status and subscribe to events
    if (ws_connected) {
        ESP_LOGD("UI_Screen2", "Requesting antenna status from server");
        
        // Request current antenna status
        esp_err_t status_ret = websocket_client_get_status();
        if (status_ret == ESP_OK) {
            ESP_LOGI("UI_Screen2", "Antenna status request sent successfully");
        } else {
            ESP_LOGW("UI_Screen2", "Failed to send antenna status request: %s", esp_err_to_name(status_ret));
        }
        
        // Request antenna names/configuration
        esp_err_t names_ret = websocket_client_get_relay_names();
        if (names_ret == ESP_OK) {
            ESP_LOGI("UI_Screen2", "Antenna names request sent successfully");
        } else {
            ESP_LOGW("UI_Screen2", "Failed to send antenna names request: %s", esp_err_to_name(names_ret));
        }
        
        // Subscribe to real-time events
        esp_err_t sub_ret = websocket_client_subscribe_events();
        if (sub_ret == ESP_OK) {
            ESP_LOGI("UI_Screen2", "Event subscription request sent successfully");
        } else {
            ESP_LOGW("UI_Screen2", "Failed to send event subscription request: %s", esp_err_to_name(sub_ret));
        }
    } else {
        ESP_LOGW("UI_Screen2", "WebSocket not connected - cannot request antenna status");
    }
    
    // Note: Legacy test message for LVGL messaging verification removed
    // Antenna state is now managed via observer pattern (radio_antenna_state_subject)
    ESP_LOGD("UI_Screen2", "Menu UI initialization complete");

    // Initialize background NVS save task and queue (DISABLED - causes heap corruption)
    ESP_LOGW("UI_Screen2", "NVS save task creation DISABLED - settings will not persist");

    // Mark deferred build as complete since we built everything immediately
    ui_MenuDeferredBuilt = true;

    ESP_LOGI("UI_Screen2", "Menu UI initialized successfully");
}

// Helper function to update antenna button states based on WebSocket status
static void ui_update_antenna_button_states(int current_antenna, const uint8_t* available_antennas, int available_count)
{
    // Smart update: check if anything actually changed
    bool state_changed = false;
    if (!g_antenna_cache.cache_valid || 
        g_antenna_cache.current_antenna != current_antenna ||
        g_antenna_cache.available_count != available_count ||
        memcmp(g_antenna_cache.available_antennas, available_antennas, sizeof(g_antenna_cache.available_antennas)) != 0) {
        state_changed = true;
    }
    
    if (!state_changed) {
        ESP_LOGV("UI_Antennas", "Antenna state unchanged - skipping UI update");
        return;
    }
    
    ESP_LOGD("UI_Antennas", "Antenna state changed: current=%d->%d, count=%d->%d", 
             g_antenna_cache.current_antenna, current_antenna, 
             g_antenna_cache.available_count, available_count);
    
    // Find the antenna grid container - it's in the Antennas menu page
    if (!ui_MenuPageAntennas) {
        ESP_LOGW("UI_Antennas", "Antennas menu page not available - ui_MenuPageAntennas is NULL");
        return;
    }
    
    // Navigate to the antenna grid: MenuPage -> Section -> AntennaGrid
    lv_obj_t *section = lv_obj_get_child(ui_MenuPageAntennas, 0);
    if (!section) {
        ESP_LOGW("UI_Antennas", "Antenna section not found");
        return;
    }
    
    lv_obj_t *antenna_grid = lv_obj_get_child(section, 0);
    if (!antenna_grid) {
        ESP_LOGW("UI_Antennas", "Antenna grid not found");
        return;
    }
    
    uint32_t child_count = lv_obj_get_child_cnt(antenna_grid);
    ESP_LOGI("UI_Antennas", "=== UPDATING ANTENNA BUTTONS ===");
    ESP_LOGI("UI_Antennas", "Grid buttons: %lu, Current: %d, Available count: %d", child_count, current_antenna, available_count);
    ESP_LOGI("UI_Antennas", "Available antennas: [%d,%d,%d,%d,%d,%d,%d,%d]",
             available_antennas[0], available_antennas[1], available_antennas[2], available_antennas[3],
             available_antennas[4], available_antennas[5], available_antennas[6], available_antennas[7]);
    
    // Update each antenna button (1-8)
    for (uint32_t i = 0; i < child_count && i < 8; i++) {
        lv_obj_t *btn = lv_obj_get_child(antenna_grid, i);
        if (!btn) {
            ESP_LOGW("UI_Antennas", "Button %lu is NULL - skipping", i);
            continue;
        }
        
        int antenna_id = i + 1; // Buttons are 0-indexed, antennas are 1-indexed
        ESP_LOGV("UI_Antennas", "Processing button %lu (antenna_id=%d)", i, antenna_id);
        
        // Clear all states first (no invalidate yet to reduce redraws)
        lv_obj_remove_state(btn, (lv_state_t)(LV_STATE_CHECKED | LV_STATE_USER_1));
        
        // Check if this antenna is currently active
        if (current_antenna == antenna_id) {
            // Currently selected antenna - green background
            lv_obj_add_state(btn, LV_STATE_CHECKED);
            ESP_LOGI("UI_Antennas", "Button %d: CURRENT ANTENNA (green)", antenna_id);
        } else {
            // Check if this antenna is available for the current frequency
            bool is_available = false;
            for (int j = 0; j < available_count; j++) {
                if (available_antennas[j] == antenna_id) {
                    is_available = true;
                    ESP_LOGD("UI_Antennas", "Match found! Antenna %d is available", antenna_id);
                    break;
                }
            }
            
            if (is_available) {
                // Available antenna - blue background
                lv_obj_add_state(btn, LV_STATE_USER_1);
                ESP_LOGI("UI_Antennas", "Button %d: AVAILABLE ANTENNA (blue)", antenna_id);
            } else {
                // Unavailable antenna - gray background (default state)
                ESP_LOGI("UI_Antennas", "Button %d: UNAVAILABLE ANTENNA (gray)", antenna_id);
            }
        }
    }
    
    // Single invalidate call for the entire grid - more efficient than per-button invalidates
    lv_obj_invalidate(antenna_grid);
    
    // Update cache with new state
    g_antenna_cache.current_antenna = current_antenna;
    g_antenna_cache.available_count = available_count;
    memcpy(g_antenna_cache.available_antennas, available_antennas, sizeof(g_antenna_cache.available_antennas));
    g_antenna_cache.cache_valid = true;
    
    ESP_LOGI("UI_Antennas", "Antenna button states updated and display refreshed");
}

// Event handler for antenna selection buttons
static void ui_event_AntennaButton_clicked(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if(event_code == LV_EVENT_CLICKED) {
        // Get antenna ID from user data
        uintptr_t antenna_id = (uintptr_t)lv_obj_get_user_data(target);
        
        // Only allow clicking on available (blue) antennas
        // Green (current) and gray (unavailable) buttons should not respond to clicks
        bool is_available_button = lv_obj_has_state(target, LV_STATE_USER_1);
        bool is_current_button = lv_obj_has_state(target, LV_STATE_CHECKED);
        
        if (is_current_button) {
            ESP_LOGD("UI_Antennas", "Antenna button %lu clicked but already active (green) - ignoring", antenna_id);
            return; // Already selected, do nothing
        }
        
        if (!is_available_button) {
            ESP_LOGD("UI_Antennas", "Antenna button %lu clicked but unavailable (gray) - ignoring", antenna_id);
            return; // Not available for current frequency, do nothing
        }
        
        ESP_LOGI("UI_Antennas", "Available antenna button %lu clicked - switching", antenna_id);
        
        // Check WebSocket connection status before sending command
        bool ws_connected = websocket_client_is_connected();
        ESP_LOGI("UI_Antennas", "WebSocket connection status: %s", ws_connected ? "CONNECTED" : "DISCONNECTED");
        
        if (!ws_connected) {
            ESP_LOGW("UI_Antennas", "WebSocket not connected - attempting immediate reconnection for antenna %lu switch", antenna_id);
            
            esp_err_t reconnect_ret = websocket_client_trigger_reconnection();
            if (reconnect_ret != ESP_OK) {
                ESP_LOGE("UI_Antennas", "Failed to trigger WebSocket reconnection: %s", esp_err_to_name(reconnect_ret));
                // TODO: Show visual feedback to user about connection issue
                return;
            }
            
            // Give the reconnection a moment, but don't block the UI
            ESP_LOGI("UI_Antennas", "Reconnection attempt initiated - antenna switch will be retried on next connection");
            // TODO: Queue the antenna switch request to be sent once reconnected
            return;
        }
        
        // Optimistic UI update: handle both previous and new antenna selection
        // Only do optimistic update if we have valid cache to work with
        if (g_antenna_cache.cache_valid && ui_MenuPageAntennas) {
            // Find and update the previously selected antenna button (green → blue if still available)
            int prev_antenna = g_antenna_cache.current_antenna;
            if (prev_antenna > 0 && prev_antenna <= 8 && prev_antenna != (int)antenna_id) {
                // Pre-compute availability for performance (avoid doing inside UI traversal)
                bool prev_is_available = false;
                for (int i = 0; i < g_antenna_cache.available_count; i++) {
                    if (g_antenna_cache.available_antennas[i] == prev_antenna) {
                        prev_is_available = true;
                        break;
                    }
                }
                
                // Get the antenna grid with minimal traversals for ESP32-S3 performance
                lv_obj_t *section = lv_obj_get_child(ui_MenuPageAntennas, 0);
                if (section) {
                    lv_obj_t *antenna_grid = lv_obj_get_child(section, 0);
                    if (antenna_grid) {
                        uint32_t child_count = lv_obj_get_child_cnt(antenna_grid);
                        if (prev_antenna <= (int)child_count) {
                            lv_obj_t *prev_btn = lv_obj_get_child(antenna_grid, prev_antenna - 1);
                            if (prev_btn && lv_obj_has_state(prev_btn, LV_STATE_CHECKED)) {
                                // Batch UI state changes for better performance
                                lv_obj_remove_state(prev_btn, LV_STATE_CHECKED); // Clear green
                                if (prev_is_available) {
                                    lv_obj_add_state(prev_btn, LV_STATE_USER_1); // Set blue (available)
                                }
                                // If not available, it becomes gray (default state)
                                
                                ESP_LOGD("UI_Antennas", "Previous antenna %d: green → %s", 
                                        prev_antenna, prev_is_available ? "blue" : "gray");
                            }
                        }
                    }
                }
            }
            
            // Update the clicked button (blue → green)
            lv_obj_remove_state(target, LV_STATE_USER_1); // Clear blue (available)
            lv_obj_add_state(target, LV_STATE_CHECKED);   // Add green (selected)
            
            // Update cache with new current antenna for next optimistic update
            g_antenna_cache.current_antenna = (int)antenna_id;
            ESP_LOGD("UI_Antennas", "Optimistic update: antenna %lu is now selected (green)", antenna_id);
        } else {
            // Cache invalid - only update clicked button, server update will fix everything
            lv_obj_remove_state(target, LV_STATE_USER_1); // Clear blue (available)
            lv_obj_add_state(target, LV_STATE_CHECKED);   // Add green (selected)
            ESP_LOGD("UI_Antennas", "Cache invalid - limited optimistic update for antenna %lu", antenna_id);
        }
        
        // Use dedicated antenna selection command for cleaner server logic
        esp_err_t ret = websocket_client_select_antenna_default((uint8_t)antenna_id);
        if (ret != ESP_OK) {
            ESP_LOGE("UI_Antennas", "Failed to select antenna %lu via WebSocket: %s", antenna_id, esp_err_to_name(ret));
            // Revert ALL optimistic updates on failure - invalidate cache and trigger full refresh
            g_antenna_cache.cache_valid = false;
            ESP_LOGW("UI_Antennas", "WebSocket command failed - invalidating cache, server update will restore correct state");
            return;
        }
        
        ESP_LOGI("UI_Antennas", "Antenna %lu selection command sent - optimistic UI updated", antenna_id);
        // Server response will confirm or correct the selection via WebSocket status events
    }
}

void ui_Screen2_screen_destroy(void) {
    if(ui_Screen2) lv_obj_delete(ui_Screen2);

    // NULL screen variables
    ui_Screen2 = NULL;
    // ui_TextArea1 = NULL; // Removed
    // ui_DebugLabel = NULL; // Removed
    ui_ReturnToScreen1Button = NULL;
    ui_ReturnToScreen1BtnLabel = NULL;
    ui_SettingsPageTitle = NULL;
    ui_BrightnessLabel = NULL;
    ui_BrightnessSlider = NULL;
    ui_PlaceholderButton1 = NULL;
    ui_PlaceholderButton2 = NULL;
    ui_PlaceholderButton3 = NULL;
    ui_PlaceholderLabel3 = NULL;
    ui_PeakHoldLabel = NULL;
    ui_PeakHoldSwitch = NULL;
    ui_PeakHoldDurationLabel = NULL;
    ui_PeakHoldDurationSlider = NULL;
    ui_PeakHoldDurationValueLabel = NULL;
    ui_BrightnessValueLabel = NULL;
    ui_PepToggleLabel = NULL;
    ui_PepToggleSwitch = NULL;
    ui_SMeterAveragingSwitch = NULL;
    ui_RebootButton = NULL;
}


void ui_event_SMeterAveragingSwitch(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t*)lv_event_get_target(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        bool is_checked = lv_obj_has_state(target, LV_STATE_CHECKED);

        // Update current settings
        current_settings.smeter_averaging_enabled = is_checked;
        current_settings.settings_changed = true;

        // Update Screen1 averaging behavior via observer
        lv_subject_set_int(&radio_smeter_averaging_subject, is_checked ? 1 : 0);

        ESP_LOGI("UI_Screen2", "S-Meter Averaging toggled: %s", is_checked ? "ON" : "OFF");
    }
}

void ui_event_RebootButton(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        ESP_LOGI("UI_Screen2", "Reboot button pressed - restarting device");

        // Give a moment for the log to be sent
        vTaskDelay(pdMS_TO_TICKS(100));

        // Reboot the ESP32
        esp_restart();
    }
}

// ============================================================================
// SIDEBAR COLLAPSE/EXPAND FOR FULL-SCREEN PAGES
// ============================================================================

static void ui_sidebar_set_collapsed(bool collapsed)
{
    if (ui_Menu == NULL) return;

    lv_menu_t *menu_priv = (lv_menu_t *)ui_Menu;
    if (menu_priv->sidebar == NULL) return;

    g_sidebar_collapsed = collapsed;

    if (collapsed) {
        lv_obj_add_flag(menu_priv->sidebar, LV_OBJ_FLAG_HIDDEN);
        // Show expand button on screen edge
        if (g_sidebar_expand_btn) {
            lv_obj_remove_flag(g_sidebar_expand_btn, LV_OBJ_FLAG_HIDDEN);
        }
        ESP_LOGI(TAG, "Sidebar collapsed");
    } else {
        lv_obj_remove_flag(menu_priv->sidebar, LV_OBJ_FLAG_HIDDEN);
        // Hide expand button
        if (g_sidebar_expand_btn) {
            lv_obj_add_flag(g_sidebar_expand_btn, LV_OBJ_FLAG_HIDDEN);
        }
        // Update header chevron to point left (collapse direction)
        if (g_sidebar_header_chevron) {
            lv_label_set_text(g_sidebar_header_chevron, LV_SYMBOL_LEFT);
        }
        ESP_LOGI(TAG, "Sidebar expanded");
    }
}

static void ui_sidebar_toggle(void)
{
    ui_sidebar_set_collapsed(!g_sidebar_collapsed);
}

// Callback for sidebar collapse/expand button clicks
static void ui_sidebar_toggle_cb(lv_event_t *e)
{
    (void)e;
    ui_sidebar_toggle();
}

// Async callback to refresh macro list after sidebar toggle (for macros page only)
static void ui_macro_async_refresh_cb(void *user_data)
{
    (void)user_data;
    ui_macro_refresh_list();
}

static void ui_macro_sidebar_toggle_cb(lv_event_t *e)
{
    (void)e;
    ui_sidebar_toggle();

    // Schedule refresh asynchronously to avoid lock issues
    // (we're in LVGL event context which holds the lock)
    lv_async_call(ui_macro_async_refresh_cb, NULL);
}

// ============================================================================
// MACRO LIST - KENWOOD TS-590SG STYLE TABLE UI
// ============================================================================

// Kenwood-style colors
#define MACRO_COLOR_ROW_BG       0x404040  // Row background
#define MACRO_COLOR_ROW_SELECTED 0xFFBC1F  // Selected row (golden yellow - matches menu selection)
#define MACRO_COLOR_HEADER_BG    0x252525  // Header background
#define MACRO_COLOR_FKEY_ACTIVE  0x4DAAFF  // Assigned F-key (light blue)
#define MACRO_COLOR_FKEY_EMPTY   0x666666  // Unassigned F-key (gray)
#define MACRO_EDIT_OVERLAY_BG    0x1a1a1a  // Dark overlay background for editing

// Column widths - scaled for multi-resolution support
#define MACRO_COL_ID_W      ui_sx(28)
#define MACRO_COL_FKEY_W    ui_sx(42)
#define MACRO_COL_NAME_W    ui_sx(120)
#define MACRO_COL_CMD_W     0  // Flex (remaining space)
#define MACRO_NAV_BTN_W     ui_sx(50)
#define MACRO_ROW_H         ui_sy(40)

// ============================================================================
// SORTING: Assigned macros first (F1-F6 order), then unassigned by ID
// ============================================================================

static void ui_macro_sort_cache(void)
{
    // Simple bubble sort - small list so performance is fine
    // Sort key: assigned F-keys first (1-6), then unassigned (7+) by ID
    for (int i = 0; i < ui_macro_count - 1; i++) {
        for (int j = 0; j < ui_macro_count - i - 1; j++) {
            int key_a = ui_macro_cache[j].fkey > 0 ? ui_macro_cache[j].fkey : (100 + ui_macro_cache[j].id);
            int key_b = ui_macro_cache[j + 1].fkey > 0 ? ui_macro_cache[j + 1].fkey : (100 + ui_macro_cache[j + 1].id);
            if (key_a > key_b) {
                ui_macro_item_t tmp = ui_macro_cache[j];
                ui_macro_cache[j] = ui_macro_cache[j + 1];
                ui_macro_cache[j + 1] = tmp;
            }
        }
    }
}

// ============================================================================
// F-KEY ASSIGNMENT POPUP
// ============================================================================

static uint8_t fkey_popup_macro_id = 0;  // Macro being assigned

static void ui_macro_fkey_btn_cb(lv_event_t *e)
{
    lv_obj_t *btn = (lv_obj_t*)lv_event_get_target(e);
    uint8_t fkey = (uintptr_t)lv_obj_get_user_data(btn);
    ESP_LOGI(TAG, "Assigning macro %d to F%d", fkey_popup_macro_id, fkey);

    // Clear any existing assignment for this F-key
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].fkey == fkey) {
            ui_macro_cache[i].fkey = 0;
        }
    }

    // Assign to selected macro
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id == fkey_popup_macro_id) {
            ui_macro_cache[i].fkey = fkey;
            break;
        }
    }

    // Update assignment cache
    ui_macro_fkey_assignments[fkey - 1] = fkey_popup_macro_id;

    // Send MXA command to panel: MXA<fkey>,<macro_id>;
    char cmd_buf[16];
    snprintf(cmd_buf, sizeof(cmd_buf), "MXA%d,%02d;", fkey, fkey_popup_macro_id);
    uart_write_message(cmd_buf);

    ui_macro_hide_fkey_popup();
    ui_macro_sort_cache();
    ui_macro_refresh_list();
}

static void ui_macro_fkey_clear_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Clearing F-key assignment for macro %d", fkey_popup_macro_id);

    // Find which F-key this macro had and clear it
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id == fkey_popup_macro_id && ui_macro_cache[i].fkey > 0) {
            uint8_t old_fkey = ui_macro_cache[i].fkey;
            ui_macro_cache[i].fkey = 0;
            ui_macro_fkey_assignments[old_fkey - 1] = 0;

            // Send MXA command to clear: MXA<fkey>,00;
            char cmd_buf[16];
            snprintf(cmd_buf, sizeof(cmd_buf), "MXA%d,00;", old_fkey);
            uart_write_message(cmd_buf);
            break;
        }
    }

    ui_macro_hide_fkey_popup();
    ui_macro_sort_cache();
    ui_macro_refresh_list();
}

static void ui_macro_fkey_popup_close_cb(lv_event_t *e)
{
    ui_macro_hide_fkey_popup();
}

void ui_macro_show_fkey_popup(uint8_t macro_id)
{
    if (macro_editor.fkey_popup) {
        lv_obj_delete(macro_editor.fkey_popup);
    }

    fkey_popup_macro_id = macro_id;

    // Create modal background
    macro_editor.fkey_popup = lv_obj_create(lv_screen_active());
    lv_obj_set_size(macro_editor.fkey_popup, ui_sx(300), ui_sy(280));
    lv_obj_center(macro_editor.fkey_popup);
    lv_obj_set_style_bg_color(macro_editor.fkey_popup, lv_color_hex(MACRO_COLOR_HEADER_BG), 0);
    lv_obj_set_style_border_color(macro_editor.fkey_popup, lv_color_hex(COLOR_BLUE_PRIMARY), 0);
    lv_obj_set_style_border_width(macro_editor.fkey_popup, 2, 0);
    lv_obj_set_style_radius(macro_editor.fkey_popup, 8, 0);
    lv_obj_set_style_pad_all(macro_editor.fkey_popup, ui_sx(16), 0);
    lv_obj_set_flex_flow(macro_editor.fkey_popup, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(macro_editor.fkey_popup, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(macro_editor.fkey_popup, ui_sy(12), 0);

    // Title
    lv_obj_t *title = lv_label_create(macro_editor.fkey_popup);
    lv_label_set_text(title, "Assign to:");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, ui_font30(), 0);

    // 2x3 grid of F-key buttons
    lv_obj_t *grid = lv_obj_create(macro_editor.fkey_popup);
    lv_obj_set_size(grid, ui_sx(220), ui_sy(130));
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, ui_sx(6), 0);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(grid, ui_sx(10), 0);

    for (int i = 1; i <= 6; i++) {
        lv_obj_t *btn = lv_button_create(grid);
        lv_obj_set_size(btn, ui_sx(95), ui_sy(42));

        // Highlight if this F-key is already assigned to this macro
        bool is_current = false;
        for (int j = 0; j < ui_macro_count; j++) {
            if (ui_macro_cache[j].id == macro_id && ui_macro_cache[j].fkey == i) {
                is_current = true;
                break;
            }
        }

        lv_obj_set_style_bg_color(btn, lv_color_hex(is_current ? COLOR_EMERALD_GREEN : COLOR_BLUE_PRIMARY), 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_user_data(btn, (void*)(uintptr_t)i);
        lv_obj_add_event_cb(btn, ui_macro_fkey_btn_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *lbl = lv_label_create(btn);
        char fkey_txt[4];
        snprintf(fkey_txt, sizeof(fkey_txt), "F%d", i);
        lv_label_set_text(lbl, fkey_txt);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(lbl);
    }

    // Bottom button row (Clear and Close)
    lv_obj_t *button_row = lv_obj_create(macro_editor.fkey_popup);
    lv_obj_set_size(button_row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(button_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(button_row, 0, 0);
    lv_obj_set_style_pad_all(button_row, 0, 0);
    lv_obj_set_flex_flow(button_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(button_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(button_row, ui_sx(10), 0);
    lv_obj_set_style_pad_bottom(button_row, ui_sy(8), 0);

    // Clear button
    lv_obj_t *clear_btn = lv_button_create(button_row);
    lv_obj_set_size(clear_btn, ui_sx(125), ui_sy(38));
    lv_obj_set_style_bg_color(clear_btn, lv_color_hex(COLOR_BG_LIGHT), 0);
    lv_obj_set_style_radius(clear_btn, 6, 0);
    lv_obj_add_event_cb(clear_btn, ui_macro_fkey_clear_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *clear_lbl = lv_label_create(clear_btn);
    lv_label_set_text(clear_lbl, "Clear");
    lv_obj_set_style_text_color(clear_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(clear_lbl, ui_font16(), 0);
    lv_obj_center(clear_lbl);

    // Close button
    lv_obj_t *close_btn = lv_button_create(button_row);
    lv_obj_set_size(close_btn, ui_sx(125), ui_sy(38));
    lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x404040), 0);
    lv_obj_set_style_radius(close_btn, 6, 0);
    lv_obj_add_event_cb(close_btn, ui_macro_fkey_popup_close_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *close_lbl = lv_label_create(close_btn);
    lv_label_set_text(close_lbl, "Close");
    lv_obj_set_style_text_color(close_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(close_lbl, ui_font16(), 0);
    lv_obj_center(close_lbl);
}

void ui_macro_hide_fkey_popup(void)
{
    if (macro_editor.fkey_popup) {
        lv_obj_delete(macro_editor.fkey_popup);
        macro_editor.fkey_popup = NULL;
    }
    fkey_popup_macro_id = 0;
}

// ============================================================================
// DELETE CONFIRMATION POPUP
// ============================================================================

static uint8_t delete_confirm_macro_id = 0;
static lv_obj_t *delete_confirm_popup = NULL;

static void ui_macro_delete_confirm_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Delete confirmed for macro %d", delete_confirm_macro_id);

    // Send MXD command to panel
    char cmd_buf[16];
    snprintf(cmd_buf, sizeof(cmd_buf), "MXD%02d;", delete_confirm_macro_id);
    uart_write_message(cmd_buf);

    // Clear F-key assignment if any
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id == delete_confirm_macro_id && ui_macro_cache[i].fkey > 0) {
            ui_macro_fkey_assignments[ui_macro_cache[i].fkey - 1] = 0;
        }
    }

    // Remove from cache
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id == delete_confirm_macro_id) {
            for (int j = i; j < ui_macro_count - 1; j++) {
                ui_macro_cache[j] = ui_macro_cache[j + 1];
            }
            ui_macro_count--;
            break;
        }
    }

    // Close popup and refresh
    if (delete_confirm_popup) {
        lv_obj_delete(delete_confirm_popup);
        delete_confirm_popup = NULL;
    }
    delete_confirm_macro_id = 0;
    ui_macro_selected_index = -1;
    ui_macro_refresh_list();
}

static void ui_macro_delete_cancel_cb(lv_event_t *e)
{
    if (delete_confirm_popup) {
        lv_obj_delete(delete_confirm_popup);
        delete_confirm_popup = NULL;
    }
    delete_confirm_macro_id = 0;
}

void ui_macro_show_delete_confirm(uint8_t macro_id)
{
    if (delete_confirm_popup) {
        lv_obj_delete(delete_confirm_popup);
    }

    delete_confirm_macro_id = macro_id;

    delete_confirm_popup = lv_obj_create(lv_screen_active());
    lv_obj_set_size(delete_confirm_popup, ui_sx(200), ui_sy(100));
    lv_obj_center(delete_confirm_popup);
    lv_obj_set_style_bg_color(delete_confirm_popup, lv_color_hex(MACRO_COLOR_HEADER_BG), 0);
    lv_obj_set_style_border_color(delete_confirm_popup, lv_color_hex(COLOR_RED_LIGHT), 0);
    lv_obj_set_style_border_width(delete_confirm_popup, 2, 0);
    lv_obj_set_style_radius(delete_confirm_popup, 8, 0);
    lv_obj_set_style_pad_all(delete_confirm_popup, ui_sx(10), 0);
    lv_obj_set_flex_flow(delete_confirm_popup, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(delete_confirm_popup, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *msg = lv_label_create(delete_confirm_popup);
    char msg_txt[32];
    snprintf(msg_txt, sizeof(msg_txt), "Delete macro #%d?", macro_id);
    lv_label_set_text(msg, msg_txt);
    lv_obj_set_style_text_color(msg, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t *btn_row = lv_obj_create(delete_confirm_popup);
    lv_obj_set_size(btn_row, LV_PCT(100), ui_sy(40));
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *yes_btn = lv_button_create(btn_row);
    lv_obj_set_size(yes_btn, ui_sx(70), ui_sy(32));
    lv_obj_set_style_bg_color(yes_btn, lv_color_hex(COLOR_RED_LIGHT), 0);
    lv_obj_add_event_cb(yes_btn, ui_macro_delete_confirm_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *yes_lbl = lv_label_create(yes_btn);
    lv_label_set_text(yes_lbl, "Delete");
    lv_obj_set_style_text_color(yes_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(yes_lbl);

    lv_obj_t *no_btn = lv_button_create(btn_row);
    lv_obj_set_size(no_btn, ui_sx(70), ui_sy(32));
    lv_obj_set_style_bg_color(no_btn, lv_color_hex(COLOR_BG_LIGHT), 0);
    lv_obj_add_event_cb(no_btn, ui_macro_delete_cancel_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *no_lbl = lv_label_create(no_btn);
    lv_label_set_text(no_lbl, "Cancel");
    lv_obj_set_style_text_color(no_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(no_lbl);
}

// ============================================================================
// INLINE EDITING
// ============================================================================

static void ui_macro_inline_save(void);

static void ui_macro_keyboard_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        // Enter pressed - save changes
        ui_macro_inline_save();
    } else if (code == LV_EVENT_CANCEL) {
        // Cancel - end editing without saving
        ui_macro_end_inline_edit(false);
    }
}

void ui_macro_start_inline_edit(int cache_index, bool edit_name)
{
    if (cache_index < 0 || cache_index >= ui_macro_count) return;

    macro_editor.editing_macro_id = ui_macro_cache[cache_index].id;
    macro_editor.is_editing = true;
    macro_editor.editing_name = edit_name;
    ui_macro_selected_index = cache_index;

    ui_macro_item_t *macro = &ui_macro_cache[cache_index];

    // Highlight selected row before showing overlay
    if (macro->row_container) {
        lv_obj_set_style_bg_color(macro->row_container, lv_color_hex(MACRO_COLOR_ROW_SELECTED), 0);
    }

    // Create overlay panel that covers the macro content area
    if (!macro_editor.edit_overlay) {
        macro_editor.edit_overlay = lv_obj_create(ui_MenuPageMacros);
        lv_obj_set_size(macro_editor.edit_overlay, LV_PCT(100), LV_PCT(100));
        lv_obj_align(macro_editor.edit_overlay, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_bg_color(macro_editor.edit_overlay, lv_color_hex(MACRO_EDIT_OVERLAY_BG), 0);
        lv_obj_set_style_bg_opa(macro_editor.edit_overlay, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(macro_editor.edit_overlay, 0, 0);
        lv_obj_set_style_pad_all(macro_editor.edit_overlay, ui_sx(8), 0);
        lv_obj_set_style_radius(macro_editor.edit_overlay, 0, 0);
        lv_obj_remove_flag(macro_editor.edit_overlay, LV_OBJ_FLAG_SCROLLABLE);

        // Label showing what we're editing
        macro_editor.edit_label = lv_label_create(macro_editor.edit_overlay);
        lv_obj_set_style_text_color(macro_editor.edit_label, lv_color_hex(COLOR_SELECTIVE_YELLOW), 0);
        lv_obj_set_style_text_font(macro_editor.edit_label, ui_font16(), 0);
        lv_obj_align(macro_editor.edit_label, LV_ALIGN_TOP_LEFT, ui_sx(4), 0);

        // Textarea at top of overlay
        macro_editor.active_textarea = lv_textarea_create(macro_editor.edit_overlay);
        lv_textarea_set_one_line(macro_editor.active_textarea, true);
        lv_obj_set_size(macro_editor.active_textarea, LV_PCT(100), ui_sy(44));
        lv_obj_align(macro_editor.active_textarea, LV_ALIGN_TOP_MID, 0, ui_sy(28));
        lv_obj_set_style_bg_color(macro_editor.active_textarea, lv_color_hex(0x2a3a4a), 0);
        lv_obj_set_style_border_color(macro_editor.active_textarea, lv_color_hex(COLOR_SELECTIVE_YELLOW), 0);
        lv_obj_set_style_border_width(macro_editor.active_textarea, 2, 0);
        lv_obj_set_style_text_color(macro_editor.active_textarea, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(macro_editor.active_textarea, ui_font16(), 0);
        lv_obj_set_style_radius(macro_editor.active_textarea, 4, 0);

        // Large keyboard filling the rest
        macro_editor.keyboard = lv_keyboard_create(macro_editor.edit_overlay);
        lv_keyboard_set_mode(macro_editor.keyboard, LV_KEYBOARD_MODE_TEXT_UPPER);
        lv_obj_set_size(macro_editor.keyboard, LV_PCT(100), ui_sy(260));  // Large keyboard
        lv_obj_align(macro_editor.keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_event_cb(macro_editor.keyboard, ui_macro_keyboard_event_cb, LV_EVENT_READY, NULL);
        lv_obj_add_event_cb(macro_editor.keyboard, ui_macro_keyboard_event_cb, LV_EVENT_CANCEL, NULL);
    }

    // Update label text
    char label_txt[64];
    snprintf(label_txt, sizeof(label_txt), "Edit %s for macro #%d:",
             edit_name ? "NAME" : "COMMAND", macro->id);
    lv_label_set_text(macro_editor.edit_label, label_txt);

    // Populate textarea
    if (edit_name) {
        lv_textarea_set_text(macro_editor.active_textarea, macro->name);
        lv_textarea_set_max_length(macro_editor.active_textarea, MACRO_NAME_MAX - 1);
    } else {
        lv_textarea_set_text(macro_editor.active_textarea, macro->command);
        lv_textarea_set_max_length(macro_editor.active_textarea, MACRO_COMMAND_MAX - 1);
    }

    lv_keyboard_set_textarea(macro_editor.keyboard, macro_editor.active_textarea);
    lv_obj_remove_flag(macro_editor.edit_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void ui_macro_inline_save(void)
{
    if (!macro_editor.is_editing || ui_macro_selected_index < 0) return;

    const char *text = lv_textarea_get_text(macro_editor.active_textarea);
    if (!text || strlen(text) == 0) {
        ui_macro_end_inline_edit(false);
        return;
    }

    ui_macro_item_t *macro = &ui_macro_cache[ui_macro_selected_index];

    if (macro_editor.editing_name) {
        strncpy(macro->name, text, MACRO_NAME_MAX - 1);
        macro->name[MACRO_NAME_MAX - 1] = '\0';
    } else {
        strncpy(macro->command, text, MACRO_COMMAND_MAX - 1);
        macro->command[MACRO_COMMAND_MAX - 1] = '\0';
    }

    // Send MXW command: MXW<ID>,<name>,<cmd>|<cmd>|...;
    char cmd_buf[256];
    snprintf(cmd_buf, sizeof(cmd_buf), "MXW%02d,%s,%s;", macro->id, macro->name, macro->command);
    ESP_LOGI(TAG, "Saving macro: %s", cmd_buf);
    uart_write_message(cmd_buf);

    ui_macro_end_inline_edit(true);
}

void ui_macro_end_inline_edit(bool save)
{
    (void)save;  // Already saved if needed

    macro_editor.is_editing = false;
    macro_editor.editing_macro_id = 0;

    // Hide the overlay (keeps it for reuse)
    if (macro_editor.edit_overlay) {
        lv_obj_add_flag(macro_editor.edit_overlay, LV_OBJ_FLAG_HIDDEN);
    }

    // Reset row color
    if (ui_macro_selected_index >= 0 && ui_macro_selected_index < ui_macro_count) {
        ui_macro_item_t *macro = &ui_macro_cache[ui_macro_selected_index];
        if (macro->row_container) {
            lv_obj_set_style_bg_color(macro->row_container, lv_color_hex(MACRO_COLOR_ROW_BG), 0);
        }
    }
    ui_macro_selected_index = -1;

    ui_macro_refresh_list();
}

// ============================================================================
// CACHE MANAGEMENT
// ============================================================================

void ui_macro_set_cached(uint8_t id, const char *name, const char *cmd)
{
    if (id < 1 || id > 50) {
        ESP_LOGW(TAG, "Invalid macro ID: %d", id);
        return;
    }

    // Skip empty/undefined macros (empty name means slot is not defined)
    bool is_empty = (!name || name[0] == '\0');

    // Check if already cached
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id == id) {
            if (is_empty) {
                // Remove from cache - shift remaining items down
                for (int j = i; j < ui_macro_count - 1; j++) {
                    ui_macro_cache[j] = ui_macro_cache[j + 1];
                }
                ui_macro_count--;
                memset(&ui_macro_cache[ui_macro_count], 0, sizeof(ui_macro_item_t));
                ESP_LOGD(TAG, "Removed empty macro %d from cache", id);
            } else {
                // Update existing
                strncpy(ui_macro_cache[i].name, name, MACRO_NAME_MAX - 1);
                ui_macro_cache[i].name[MACRO_NAME_MAX - 1] = '\0';
                strncpy(ui_macro_cache[i].command, cmd, MACRO_COMMAND_MAX - 1);
                ui_macro_cache[i].command[MACRO_COMMAND_MAX - 1] = '\0';
            }
            return;
        }
    }

    // Don't add empty macros
    if (is_empty) {
        return;
    }

    // New macro - add to cache
    if (ui_macro_count < MAX_UI_MACROS) {
        ui_macro_cache[ui_macro_count].id = id;
        ui_macro_cache[ui_macro_count].fkey = 0;  // Not assigned initially
        strncpy(ui_macro_cache[ui_macro_count].name, name, MACRO_NAME_MAX - 1);
        ui_macro_cache[ui_macro_count].name[MACRO_NAME_MAX - 1] = '\0';
        strncpy(ui_macro_cache[ui_macro_count].command, cmd, MACRO_COMMAND_MAX - 1);
        ui_macro_cache[ui_macro_count].command[MACRO_COMMAND_MAX - 1] = '\0';
        ui_macro_cache[ui_macro_count].row_container = NULL;
        ui_macro_count++;
        ui_macro_sort_cache();
    }
}

void ui_macro_set_fkey_assignment(uint8_t fkey, uint8_t macro_id)
{
    if (fkey < 1 || fkey > MACRO_FKEY_SLOTS) return;

    // Clear old assignment for this F-key
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].fkey == fkey) {
            ui_macro_cache[i].fkey = 0;
        }
    }

    // Set new assignment
    ui_macro_fkey_assignments[fkey - 1] = macro_id;

    if (macro_id > 0) {
        for (int i = 0; i < ui_macro_count; i++) {
            if (ui_macro_cache[i].id == macro_id) {
                ui_macro_cache[i].fkey = fkey;
                break;
            }
        }
    }

    ui_macro_sort_cache();
}

// ============================================================================
// TABLE ROW CLICK HANDLERS
// ============================================================================

static void ui_macro_row_fkey_cb(lv_event_t *e)
{
    lv_obj_t *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
    uint8_t macro_id = (uintptr_t)lv_obj_get_user_data(target);
    ESP_LOGI(TAG, "F-key column clicked for macro %d", macro_id);
    ui_macro_show_fkey_popup(macro_id);
}

static void ui_macro_row_name_cb(lv_event_t *e)
{
    lv_obj_t *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
    int cache_idx = (intptr_t)lv_obj_get_user_data(target);
    ESP_LOGI(TAG, "Name column clicked for cache index %d", cache_idx);
    ui_macro_start_inline_edit(cache_idx, true);
}

static void ui_macro_row_cmd_cb(lv_event_t *e)
{
    lv_obj_t *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
    int cache_idx = (intptr_t)lv_obj_get_user_data(target);
    ESP_LOGI(TAG, "Command column clicked for cache index %d", cache_idx);
    ui_macro_start_inline_edit(cache_idx, false);
}

static void ui_macro_row_select_cb(lv_event_t *e)
{
    lv_obj_t *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
    int cache_idx = (intptr_t)lv_obj_get_user_data(target);

    // Deselect previous
    if (ui_macro_selected_index >= 0 && ui_macro_selected_index < ui_macro_count) {
        if (ui_macro_cache[ui_macro_selected_index].row_container) {
            lv_obj_set_style_bg_color(ui_macro_cache[ui_macro_selected_index].row_container,
                                       lv_color_hex(MACRO_COLOR_ROW_BG), 0);
        }
    }

    // Select new
    ui_macro_selected_index = cache_idx;
    if (cache_idx >= 0 && cache_idx < ui_macro_count) {
        if (ui_macro_cache[cache_idx].row_container) {
            lv_obj_set_style_bg_color(ui_macro_cache[cache_idx].row_container,
                                       lv_color_hex(MACRO_COLOR_ROW_SELECTED), 0);
        }
    }
}

// ============================================================================
// SIDE NAVIGATION BUTTON HANDLERS
// ============================================================================

static void ui_macro_nav_add_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Add new macro");

    // Find next available ID
    uint8_t new_id = 1;
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id >= new_id) {
            new_id = ui_macro_cache[i].id + 1;
        }
    }

    if (new_id > 50) {
        ESP_LOGW(TAG, "Max macros reached (50)");
        return;
    }

    // Create new macro with default values
    ui_macro_set_cached(new_id, "New Macro", "");

    // Immediately save to panel so it exists for F-key assignment
    char cmd_buf[256];
    snprintf(cmd_buf, sizeof(cmd_buf), "MXW%02d,New Macro,;", new_id);
    uart_write_message(cmd_buf);
    ESP_LOGI(TAG, "Created new macro %d on panel", new_id);

    ui_macro_refresh_list();

    // Start editing name of new macro
    for (int i = 0; i < ui_macro_count; i++) {
        if (ui_macro_cache[i].id == new_id) {
            ui_macro_start_inline_edit(i, true);
            break;
        }
    }
}

static void ui_macro_nav_up_cb(lv_event_t *e)
{
    if (macro_editor.table_container) {
        lv_coord_t y = lv_obj_get_scroll_y(macro_editor.table_container);
        lv_obj_scroll_to_y(macro_editor.table_container, y - MACRO_ROW_H * 2, LV_ANIM_ON);
    }
}

static void ui_macro_nav_down_cb(lv_event_t *e)
{
    if (macro_editor.table_container) {
        lv_coord_t y = lv_obj_get_scroll_y(macro_editor.table_container);
        lv_obj_scroll_to_y(macro_editor.table_container, y + MACRO_ROW_H * 2, LV_ANIM_ON);
    }
}

static void ui_macro_nav_refresh_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Refreshing macros from panel");
    // Query panel for latest macro data using MX protocol
    uart_write_message("MXA;");  // Get F-key assignments
    // Query all macros individually (no list command in MX protocol)
    for (int i = 1; i <= 50; i++) {
        char cmd[16];
        snprintf(cmd, sizeof(cmd), "MXR%02d;", i);
        uart_write_message(cmd);
    }
}

static void ui_macro_nav_back_cb(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "Back to settings menu");
    // Navigate back - the menu will handle this via its back mechanism
    // Sidebar state is preserved - user controls expand/collapse manually
    lv_menu_set_page(ui_Menu, NULL);  // Go to root
}

static void ui_macro_nav_delete_cb(lv_event_t *e)
{
    if (ui_macro_selected_index >= 0 && ui_macro_selected_index < ui_macro_count) {
        ui_macro_show_delete_confirm(ui_macro_cache[ui_macro_selected_index].id);
    } else {
        ESP_LOGW(TAG, "No macro selected for deletion");
    }
}

// ============================================================================
// MAIN LIST POPULATION - KENWOOD STYLE TABLE
// ============================================================================

void ui_macro_populate_list(void)
{
    ESP_LOGI(TAG, "Populating Kenwood-style macro table");

    lvgl_port_lock(0);

    // Sort cache before rendering
    ui_macro_sort_cache();

    // Main container - horizontal layout: table on left, nav buttons on right
    lv_obj_t *main_container = lv_menu_section_create(ui_MenuPageMacros);
    lv_obj_set_style_bg_color(ui_MenuPageMacros, lv_color_hex(0x303030), LV_PART_MAIN);
    lv_obj_set_style_pad_left(main_container, ui_sx(12), LV_PART_MAIN);  // Left padding for whole view
    lv_obj_set_style_pad_right(main_container, ui_sx(4), LV_PART_MAIN);
    lv_obj_set_style_pad_top(main_container, ui_sy(4), LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(main_container, ui_sy(4), LV_PART_MAIN);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(main_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_size(main_container, LV_PCT(100), LV_SIZE_CONTENT);

    // ===== LEFT: Table container =====
    // Use more width when sidebar is collapsed (full screen mode)
    lv_obj_t *table_area = lv_obj_create(main_container);
    lv_obj_set_size(table_area, g_sidebar_collapsed ? LV_PCT(92) : LV_PCT(88), ui_sy(310));
    lv_obj_set_style_bg_opa(table_area, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(table_area, 0, 0);
    lv_obj_set_style_pad_all(table_area, 0, 0);
    lv_obj_set_flex_flow(table_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(table_area, LV_DIR_VER);
    macro_editor.table_container = table_area;

    // Header row with column labels
    lv_obj_t *header = lv_obj_create(table_area);
    lv_obj_set_size(header, LV_PCT(100), ui_sy(32));
    lv_obj_set_style_bg_color(header, lv_color_hex(MACRO_COLOR_HEADER_BG), 0);
    lv_obj_set_style_radius(header, 4, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_left(header, ui_sx(8), 0);
    lv_obj_set_style_pad_right(header, ui_sx(4), 0);
    lv_obj_set_style_pad_top(header, ui_sy(4), 0);
    lv_obj_set_style_pad_bottom(header, ui_sy(4), 0);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    // Column header: # (ID)
    lv_obj_t *hdr_id = lv_label_create(header);
    lv_label_set_text(hdr_id, "#");
    lv_obj_set_style_text_color(hdr_id, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(hdr_id, ui_font16(), 0);
    lv_obj_set_width(hdr_id, MACRO_COL_ID_W);

    // Column header: KEY
    lv_obj_t *hdr_fkey = lv_label_create(header);
    lv_label_set_text(hdr_fkey, "KEY");
    lv_obj_set_style_text_color(hdr_fkey, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(hdr_fkey, ui_font16(), 0);
    lv_obj_set_width(hdr_fkey, MACRO_COL_FKEY_W);

    // Column header: NAME (wider when sidebar collapsed)
    const int hdr_name_w = g_sidebar_collapsed ? ui_sx(160) : MACRO_COL_NAME_W;
    lv_obj_t *hdr_name = lv_label_create(header);
    lv_label_set_text(hdr_name, "NAME");
    lv_obj_set_style_text_color(hdr_name, lv_color_hex(COLOR_SELECTIVE_YELLOW), 0);
    lv_obj_set_style_text_font(hdr_name, ui_font16(), 0);
    lv_obj_set_width(hdr_name, hdr_name_w);

    // Column header: COMMAND (flex)
    lv_obj_t *hdr_cmd = lv_label_create(header);
    lv_label_set_text(hdr_cmd, "COMMAND");
    lv_obj_set_style_text_color(hdr_cmd, lv_color_hex(0x888888), 0);
    lv_obj_set_style_text_font(hdr_cmd, ui_font16(), 0);
    lv_obj_set_flex_grow(hdr_cmd, 1);

    // Add sample default macros if empty
    if (ui_macro_count == 0) {
        ui_macro_set_cached(1, "20M FT8", "FA00014074000;MD2;DA1;");
        ui_macro_cache[0].fkey = 1;  // Assign to F1
        ui_macro_fkey_assignments[0] = 1;

        ui_macro_set_cached(2, "10M FT8", "FA00028074000;MD2;DA1;");
        ui_macro_cache[1].fkey = 2;  // Assign to F2
        ui_macro_fkey_assignments[1] = 2;

        ui_macro_set_cached(3, "30M CW", "FA00010105000;MD3;");
        ui_macro_cache[2].fkey = 3;  // Assign to F3
        ui_macro_fkey_assignments[2] = 3;

        ui_macro_set_cached(4, "40M SSB", "FA00007200000;MD2;");
        // Not assigned to F-key

        ui_macro_sort_cache();
    }

    // Table rows
    for (int i = 0; i < ui_macro_count; i++) {
        ui_macro_item_t *macro = &ui_macro_cache[i];

        lv_obj_t *row = lv_obj_create(table_area);
        lv_obj_set_size(row, LV_PCT(100), MACRO_ROW_H);
        lv_obj_set_style_bg_color(row, lv_color_hex(MACRO_COLOR_ROW_BG), 0);
        lv_obj_set_style_radius(row, 0, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row, lv_color_hex(0x505050), 0);
        lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
        lv_obj_set_style_pad_left(row, ui_sx(8), 0);
        lv_obj_set_style_pad_right(row, ui_sx(4), 0);
        lv_obj_set_style_pad_top(row, ui_sy(4), 0);
        lv_obj_set_style_pad_bottom(row, ui_sy(4), 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_user_data(row, (void*)(intptr_t)i);
        lv_obj_add_event_cb(row, ui_macro_row_select_cb, LV_EVENT_CLICKED, NULL);
        macro->row_container = row;

        // Column 1: ID
        lv_obj_t *id_lbl = lv_label_create(row);
        char id_txt[4];
        snprintf(id_txt, sizeof(id_txt), "%d", macro->id);
        lv_label_set_text(id_lbl, id_txt);
        lv_obj_set_style_text_color(id_lbl, lv_color_hex(0x888888), 0);
        lv_obj_set_style_text_font(id_lbl, ui_font16(), 0);
        lv_obj_set_width(id_lbl, MACRO_COL_ID_W);

        // Column 2: F-key button
        lv_obj_t *fkey_btn = lv_button_create(row);
        lv_obj_set_size(fkey_btn, MACRO_COL_FKEY_W, MACRO_ROW_H - ui_sy(8));
        lv_obj_set_style_radius(fkey_btn, 4, 0);
        lv_obj_set_style_pad_all(fkey_btn, 0, 0);
        lv_obj_set_user_data(fkey_btn, (void*)(uintptr_t)macro->id);
        lv_obj_add_event_cb(fkey_btn, ui_macro_row_fkey_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *fkey_lbl = lv_label_create(fkey_btn);
        if (macro->fkey > 0 && macro->fkey <= 6) {
            char fkey_txt[4];
            snprintf(fkey_txt, sizeof(fkey_txt), "F%d", macro->fkey);
            lv_label_set_text(fkey_lbl, fkey_txt);
            lv_obj_set_style_bg_color(fkey_btn, lv_color_hex(MACRO_COLOR_FKEY_ACTIVE), 0);
            lv_obj_set_style_text_color(fkey_lbl, lv_color_hex(0xFFFFFF), 0);
        } else {
            lv_label_set_text(fkey_lbl, "--");
            lv_obj_set_style_bg_color(fkey_btn, lv_color_hex(MACRO_COLOR_FKEY_EMPTY), 0);
            lv_obj_set_style_text_color(fkey_lbl, lv_color_hex(0x999999), 0);
        }
        lv_obj_set_style_text_font(fkey_lbl, ui_font16(), 0);
        lv_obj_center(fkey_lbl);

        // Column 3: Name (clickable button with visible background)
        // Wider name column when sidebar is collapsed for more editing space
        const int name_width = g_sidebar_collapsed ? ui_sx(160) : MACRO_COL_NAME_W;
        lv_obj_t *name_btn = lv_button_create(row);
        lv_obj_set_size(name_btn, name_width, MACRO_ROW_H - ui_sy(8));
        lv_obj_set_style_bg_color(name_btn, lv_color_hex(0x3a3a3a), 0);
        lv_obj_set_style_bg_color(name_btn, lv_color_hex(0x4a4a4a), LV_STATE_PRESSED);
        lv_obj_set_style_radius(name_btn, 4, 0);
        lv_obj_set_style_border_width(name_btn, 1, 0);
        lv_obj_set_style_border_color(name_btn, lv_color_hex(0x555555), 0);
        lv_obj_set_style_pad_left(name_btn, ui_sx(6), 0);
        lv_obj_set_style_pad_right(name_btn, ui_sx(2), 0);
        lv_obj_set_user_data(name_btn, (void*)(intptr_t)i);
        lv_obj_add_event_cb(name_btn, ui_macro_row_name_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *name_lbl = lv_label_create(name_btn);
        lv_label_set_text(name_lbl, macro->name);
        lv_label_set_long_mode(name_lbl, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(name_lbl, name_width - ui_sx(12));
        lv_obj_set_style_text_color(name_lbl, lv_color_hex(COLOR_SELECTIVE_YELLOW), 0);
        lv_obj_set_style_text_font(name_lbl, ui_font16(), 0);
        lv_obj_align(name_lbl, LV_ALIGN_LEFT_MID, 0, 0);

        // Column 4: Command (flex, clickable button with visible background)
        lv_obj_t *cmd_btn = lv_button_create(row);
        lv_obj_set_flex_grow(cmd_btn, 1);
        lv_obj_set_height(cmd_btn, MACRO_ROW_H - ui_sy(8));
        lv_obj_set_style_bg_color(cmd_btn, lv_color_hex(0x2a2a2a), 0);
        lv_obj_set_style_bg_color(cmd_btn, lv_color_hex(0x3a3a3a), LV_STATE_PRESSED);
        lv_obj_set_style_radius(cmd_btn, 4, 0);
        lv_obj_set_style_border_width(cmd_btn, 1, 0);
        lv_obj_set_style_border_color(cmd_btn, lv_color_hex(0x444444), 0);
        lv_obj_set_style_pad_left(cmd_btn, ui_sx(6), 0);
        lv_obj_set_style_pad_right(cmd_btn, ui_sx(2), 0);
        lv_obj_set_user_data(cmd_btn, (void*)(intptr_t)i);
        lv_obj_add_event_cb(cmd_btn, ui_macro_row_cmd_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *cmd_lbl = lv_label_create(cmd_btn);
        lv_label_set_text(cmd_lbl, strlen(macro->command) > 0 ? macro->command : "(tap to edit)");
        lv_label_set_long_mode(cmd_lbl, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(cmd_lbl, LV_PCT(100));
        lv_obj_set_style_text_color(cmd_lbl, lv_color_hex(strlen(macro->command) > 0 ? 0xBBBBBB : 0x666666), 0);
        lv_obj_set_style_text_font(cmd_lbl, ui_font16(), 0);
        lv_obj_align(cmd_lbl, LV_ALIGN_LEFT_MID, 0, 0);
    }

    // ===== RIGHT: Navigation buttons (6 buttons) =====
    lv_obj_t *nav_panel = lv_obj_create(main_container);
    lv_obj_set_size(nav_panel, MACRO_NAV_BTN_W, ui_sy(310));  // 6 buttons @ 48px + gaps
    lv_obj_set_style_bg_opa(nav_panel, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(nav_panel, 0, 0);
    lv_obj_set_style_pad_all(nav_panel, 0, 0);
    lv_obj_set_flex_flow(nav_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(nav_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(nav_panel, ui_sy(4), 0);
    lv_obj_remove_flag(nav_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Helper to create nav button
    auto create_nav_btn = [](lv_obj_t *parent, const char *symbol, uint32_t color,
                             lv_event_cb_t cb) -> lv_obj_t* {
        lv_obj_t *btn = lv_button_create(parent);
        lv_obj_set_size(btn, MACRO_NAV_BTN_W - ui_sx(4), ui_sy(48));
        lv_obj_set_style_bg_color(btn, lv_color_hex(color), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(color - 0x202020), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, symbol);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(lbl, ui_symbol_font_sm(), 0);
        lv_obj_center(lbl);

        return btn;
    };

    // Navigation buttons
    create_nav_btn(nav_panel, LV_SYMBOL_LEFT, 0x555555, ui_macro_nav_back_cb);
    create_nav_btn(nav_panel, LV_SYMBOL_REFRESH, COLOR_SELECTIVE_YELLOW, ui_macro_nav_refresh_cb);
    create_nav_btn(nav_panel, LV_SYMBOL_PLUS, COLOR_EMERALD_GREEN, ui_macro_nav_add_cb);
    create_nav_btn(nav_panel, LV_SYMBOL_TRASH, COLOR_RED_LIGHT, ui_macro_nav_delete_cb);
    create_nav_btn(nav_panel, LV_SYMBOL_UP, COLOR_BLUE_PRIMARY, ui_macro_nav_up_cb);
    create_nav_btn(nav_panel, LV_SYMBOL_DOWN, COLOR_BLUE_PRIMARY, ui_macro_nav_down_cb);

    lvgl_port_unlock();
    ESP_LOGI(TAG, "Macro table populated with %d items", ui_macro_count);
}

// Async callback for LVGL task - runs with proper stack space
static void macro_refresh_async_cb(void *user_data) {
    (void)user_data;
    ESP_LOGD(TAG, "Debounced macro refresh executing on LVGL task");
    ui_macro_refresh_list();
}

// Timer callback for debounced macro refresh - schedules work on LVGL task
static void macro_refresh_timer_cb(TimerHandle_t timer) {
    (void)timer;
    ESP_LOGD(TAG, "Debounced macro refresh triggered");
    // Don't call ui_macro_refresh_list() directly - timer task has small stack
    // Schedule on LVGL task which has proper stack space for UI work
    lv_async_call(macro_refresh_async_cb, NULL);
}

void ui_macro_request_refresh(void)
{
    // Create timer on first use
    if (macro_refresh_timer == NULL) {
        macro_refresh_timer = xTimerCreate(
            "macro_refresh",
            pdMS_TO_TICKS(MACRO_REFRESH_DEBOUNCE_MS),
            pdFALSE,  // One-shot timer
            NULL,
            macro_refresh_timer_cb
        );
        if (macro_refresh_timer == NULL) {
            ESP_LOGW(TAG, "Failed to create macro refresh timer, using direct refresh");
            ui_macro_refresh_list();
            return;
        }
    }

    // Reset timer - this debounces rapid calls
    if (xTimerIsTimerActive(macro_refresh_timer)) {
        xTimerReset(macro_refresh_timer, 0);
    } else {
        xTimerStart(macro_refresh_timer, 0);
    }
}

void ui_macro_refresh_list(void)
{
    ESP_LOGD(TAG, "Refreshing macro list");

    lvgl_port_lock(0);

    // Clean up editor state
    macro_editor.edit_overlay = NULL;
    macro_editor.keyboard = NULL;
    macro_editor.active_textarea = NULL;
    macro_editor.edit_label = NULL;
    macro_editor.table_container = NULL;
    macro_editor.fkey_popup = NULL;

    // Remove all children from macros page
    lv_obj_clean(ui_MenuPageMacros);

    // Rebuild list (will call lvgl_port_unlock internally but we need symmetry)
    lvgl_port_unlock();

    ui_macro_populate_list();
}
