/**
 * @file ui_power_popup.h
 * @brief Power/Carrier level adjustment popup for RRC panel integration
 *
 * This component provides a popup overlay for adjusting radio parameters
 * (RF Power, TX Monitor Level) via the RRC panel's buttons and encoders.
 * It communicates with the panel using UI meta commands (UIPC, UIML, UIMN).
 */

#ifndef UI_POWER_POPUP_H
#define UI_POWER_POPUP_H

#include "lvgl.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UI Control types (must match RRC_Interface UIControl enum)
 */
typedef enum {
    UI_CONTROL_NONE = 0,
    UI_CONTROL_POWER = 1,
    UI_CONTROL_CARRIER_LEVEL = 2,
    UI_CONTROL_AF_GAIN = 3,
    UI_CONTROL_RF_GAIN = 4,
    UI_CONTROL_MIC_GAIN = 5,
    UI_CONTROL_NR_LEVEL = 6,     // NR1: level 1-10
    UI_CONTROL_NB_LEVEL = 7,
    UI_CONTROL_NR2_SPEED = 8,    // NR2: SPAC speed 0-9 (2-20ms)
    UI_CONTROL_PROC_INPUT_LEVEL = 9,   // Speech processor input level 0-100
    UI_CONTROL_PROC_OUTPUT_LEVEL = 10, // Speech processor output level 0-100
    UI_CONTROL_NOTCH_FREQUENCY = 11,   // Manual notch frequency 0-127
    UI_CONTROL_IF_SHIFT = 12,          // IF shift 0-9999 Hz (CW/CW-R only)
    UI_CONTROL_RIT_XIT_OFFSET = 13,    // RIT/XIT offset -9999 to +9999 Hz
    UI_CONTROL_DATA_MODE = 14,         // Data mode toggle 0=OFF, 1=ON
    UI_CONTROL_CW_CARRIER_LEVEL = 15,  // CW carrier output level 0-100%
} ui_control_type_t;

/**
 * @brief Initialize the popup component
 * @param parent_screen The screen to attach the popup to (typically ui_Screen1)
 * @note Call this once during screen initialization
 */
void ui_power_popup_init(lv_obj_t *parent_screen);

/**
 * @brief Show the popup for a specific control type
 * @param type The control type to display
 * @param initial_value The initial value to show
 */
void ui_power_popup_show(ui_control_type_t type, int initial_value);

/**
 * @brief Hide/dismiss the popup
 */
void ui_power_popup_hide(void);

/**
 * @brief Update the displayed value (called from CAT parser when UIPC/UIML received)
 * @param value The new value to display
 */
void ui_power_popup_set_value(int value);

/**
 * @brief Check if popup is currently visible
 * @return true if visible, false otherwise
 */
bool ui_power_popup_is_visible(void);

/**
 * @brief Get the current control type being displayed
 * @return The active control type, or UI_CONTROL_NONE if hidden
 */
ui_control_type_t ui_power_popup_get_type(void);

/**
 * @brief Cleanup popup resources (call during screen destroy)
 */
void ui_power_popup_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // UI_POWER_POPUP_H
