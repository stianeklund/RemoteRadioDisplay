#pragma once

/**
 * @file ui_scale.h
 * @brief Resolution-independent UI scaling for multi-target displays
 *
 * Design reference: 800x480 (ESP32-S3 Sunton board).
 * On S3: ui_sx(x) == x, ui_sy(y) == y (identity, optimized away).
 * On P4 (1280x720): ui_sx(100) == 160, ui_sy(100) == 150.
 *
 * All arguments are typically compile-time constants, so these resolve
 * at compile time with zero runtime cost.
 *
 * Font accessors return the correct font for the current target.
 * Both screens are 5" diagonal; P4 has ~1.6x higher DPI, so all
 * fonts scale by the same factor to maintain physical size.
 */

#include "gfx/lcd_config.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// Scale a horizontal (X-axis) design value from 800px reference to actual H_RES
static inline lv_coord_t ui_sx(int x) {
    return (lv_coord_t)((int32_t)x * H_RES / 800);
}

// Scale a vertical (Y-axis) design value from 480px reference to actual V_RES
static inline lv_coord_t ui_sy(int y) {
    return (lv_coord_t)((int32_t)y * V_RES / 480);
}

// --- P4 font declarations (all guarded) ---
#if CONFIG_IDF_TARGET_ESP32P4
LV_FONT_DECLARE(ui_font_Font16_P4);
LV_FONT_DECLARE(ui_font_Font18_P4);
LV_FONT_DECLARE(ui_font_Font30_P4);
LV_FONT_DECLARE(ui_font_Font48_P4);
LV_FONT_DECLARE(ui_font_Frequency_P4);
LV_FONT_DECLARE(ui_font_FrequencyFont_P4);
LV_FONT_DECLARE(ui_font_FrequencyFontSmall_P4);
LV_FONT_DECLARE(ui_font_ButtonFont_P4);
LV_FONT_DECLARE(ui_font_ButtonFontLargeBold_P4);
LV_FONT_DECLARE(ui_font_ButtonFontSmallMedium_P4);
LV_FONT_DECLARE(ui_font_ButtonSmallRegular_P4);
LV_FONT_DECLARE(ui_font_MeterFont_P4);
LV_FONT_DECLARE(ui_font_UTCTime_P4);
LV_FONT_DECLARE(ui_font_Glyphs24_P4);
LV_FONT_DECLARE(ui_font_Glyphs30_P4);
LV_FONT_DECLARE(ui_font_Glyphs58_P4);
LV_FONT_DECLARE(ui_font_Glyphs70_P4);
#endif

// --- Font accessor functions ---
// Each returns S3 original or P4 scaled variant based on target.

#define UI_FONT_ACCESSOR(func_name, s3_font, p4_font) \
    static inline const lv_font_t *func_name(void) { \
        _Pragma("GCC diagnostic push") \
        _Pragma("GCC diagnostic ignored \"-Wunused-function\"") \
        extern const lv_font_t s3_font; \
        _Pragma("GCC diagnostic pop") \
        return IF_P4(&p4_font, &s3_font); \
    }

// Target selection helper
#if CONFIG_IDF_TARGET_ESP32P4
#define IF_P4(p4_val, s3_val) (p4_val)
#else
#define IF_P4(p4_val, s3_val) (s3_val)
#endif

// Text fonts
UI_FONT_ACCESSOR(ui_font16,              ui_font_Font16,              ui_font_Font16_P4)
UI_FONT_ACCESSOR(ui_font18,              ui_font_Font18,              ui_font_Font18_P4)
UI_FONT_ACCESSOR(ui_font30,              ui_font_Font30,              ui_font_Font30_P4)
UI_FONT_ACCESSOR(ui_font48,              ui_font_Font48,              ui_font_Font48_P4)
UI_FONT_ACCESSOR(ui_frequency_font,      ui_font_Frequency,           ui_font_Frequency_P4)
UI_FONT_ACCESSOR(ui_freq_font,           ui_font_FrequencyFont,       ui_font_FrequencyFont_P4)
UI_FONT_ACCESSOR(ui_freq_small_font,     ui_font_FrequencyFontSmall,  ui_font_FrequencyFontSmall_P4)

// Button fonts
UI_FONT_ACCESSOR(ui_btn_font,            ui_font_ButtonFont,            ui_font_ButtonFont_P4)
UI_FONT_ACCESSOR(ui_btn_large_bold_font, ui_font_ButtonFontLargeBold,   ui_font_ButtonFontLargeBold_P4)
UI_FONT_ACCESSOR(ui_btn_small_med_font,  ui_font_ButtonFontSmallMedium, ui_font_ButtonFontSmallMedium_P4)
UI_FONT_ACCESSOR(ui_btn_small_reg_font,  ui_font_ButtonSmallRegular,    ui_font_ButtonSmallRegular_P4)

// Meter / UTC
UI_FONT_ACCESSOR(ui_meter_font,          ui_font_MeterFont,  ui_font_MeterFont_P4)
UI_FONT_ACCESSOR(ui_utc_font,            ui_font_UTCTime,    ui_font_UTCTime_P4)

// Glyph fonts
UI_FONT_ACCESSOR(ui_glyph24_font,        ui_font_Glyphs24,   ui_font_Glyphs24_P4)
UI_FONT_ACCESSOR(ui_glyph30_font,        ui_font_Glyphs30,   ui_font_Glyphs30_P4)
UI_FONT_ACCESSOR(ui_glyph58_font,        ui_font_Glyphs58,   ui_font_Glyphs58_P4)
UI_FONT_ACCESSOR(ui_glyph70_font,        ui_font_Glyphs70,   ui_font_Glyphs70_P4)

// Built-in Montserrat fonts for LV_SYMBOL_* rendering (custom fonts lack 0xF000+ glyphs)
static inline const lv_font_t *ui_symbol_font_sm(void) {
    return IF_P4(&lv_font_montserrat_24, &lv_font_montserrat_14);
}
static inline const lv_font_t *ui_symbol_font_lg(void) {
    return IF_P4(&lv_font_montserrat_36, &lv_font_montserrat_24);
}

#ifdef __cplusplus
}
#endif
