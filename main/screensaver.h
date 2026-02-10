/**
 * @file screensaver.h
 * @brief LCD screensaver to prevent image sticking/ghosting
 *
 * Activates screensaver (blank screen + backlight off) after configurable
 * period of inactivity. Tracks both touch input and radio data activity.
 */
#ifndef SCREENSAVER_H
#define SCREENSAVER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Screensaver timeout options
 */
typedef enum {
    SCREENSAVER_DISABLED = 0,     ///< Screensaver disabled
    SCREENSAVER_5_MIN = 5,        ///< Activate after 5 minutes
    SCREENSAVER_10_MIN = 10,      ///< Activate after 10 minutes
    SCREENSAVER_15_MIN = 15,      ///< Activate after 15 minutes
    SCREENSAVER_30_MIN = 30,      ///< Activate after 30 minutes
} screensaver_timeout_t;

/**
 * @brief Initialize screensaver module
 *
 * Creates LVGL timer to check inactivity and manage screensaver state.
 * Must be called after LVGL initialization.
 *
 * @return ESP_OK on success
 */
esp_err_t screensaver_init(void);

/**
 * @brief Set screensaver timeout
 *
 * @param timeout Timeout period or SCREENSAVER_DISABLED
 */
void screensaver_set_timeout(screensaver_timeout_t timeout);

/**
 * @brief Get current screensaver timeout setting
 *
 * @return Current timeout value
 */
screensaver_timeout_t screensaver_get_timeout(void);

/**
 * @brief Check if screensaver is currently active
 *
 * @return true if screensaver is showing, false otherwise
 */
bool screensaver_is_active(void);

/**
 * @brief Manually trigger activity to wake screensaver
 *
 * Call this when radio data is received to reset inactivity timer.
 * Touch input is handled automatically by LVGL.
 */
void screensaver_trigger_activity(void);

/**
 * @brief Notify screensaver of external backlight change
 *
 * Call this when backlight level is changed externally (e.g., UIBL command)
 * to update saved backlight level and wake screensaver if active.
 *
 * @param new_level New backlight level (0-255)
 */
void screensaver_update_backlight(uint8_t new_level);

#ifdef __cplusplus
}
#endif

#endif // SCREENSAVER_H
