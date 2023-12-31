// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.3.6
// Project name: Sunton_SquareLine_Project

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
ui_Screen1 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen1, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM );    /// Flags
lv_obj_set_style_text_font(ui_Screen1, &lv_font_montserrat_18, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_Screen1, 0, LV_PART_MAIN| LV_STATE_USER_4);
lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_USER_4 );
lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN| LV_STATE_USER_4);

ui_SignalPanel = lv_obj_create(ui_Screen1);
lv_obj_set_width( ui_SignalPanel, 797);
lv_obj_set_height( ui_SignalPanel, 125);
lv_obj_set_x( ui_SignalPanel, 0 );
lv_obj_set_y( ui_SignalPanel, -179 );
lv_obj_set_align( ui_SignalPanel, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_SignalPanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_SignalPanel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SMeterBar = lv_bar_create(ui_Screen1);
lv_bar_set_value(ui_SMeterBar,25,LV_ANIM_OFF);
lv_obj_set_width( ui_SMeterBar, 163);
lv_obj_set_height( ui_SMeterBar, 10);
lv_obj_set_x( ui_SMeterBar, -259 );
lv_obj_set_y( ui_SMeterBar, -208 );
lv_obj_set_align( ui_SMeterBar, LV_ALIGN_CENTER );
lv_obj_set_style_radius(ui_SMeterBar, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_SMeterBar, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_SMeterLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_SMeterLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SMeterLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SMeterLabel, -381 );
lv_obj_set_y( ui_SMeterLabel, -208 );
lv_obj_set_align( ui_SMeterLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SMeterLabel,"S");
lv_obj_set_style_text_align(ui_SMeterLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SMeterLabel, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Switch1 = lv_switch_create(ui_Screen1);
lv_obj_set_width( ui_Switch1, 50);
lv_obj_set_height( ui_Switch1, 25);
lv_obj_set_x( ui_Switch1, 249 );
lv_obj_set_y( ui_Switch1, 159 );
lv_obj_set_align( ui_Switch1, LV_ALIGN_CENTER );


ui_SwrMeterLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_SwrMeterLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SwrMeterLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SwrMeterLabel, -367 );
lv_obj_set_y( ui_SwrMeterLabel, -151 );
lv_obj_set_align( ui_SwrMeterLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SwrMeterLabel,"SWR");
lv_obj_set_style_text_align(ui_SwrMeterLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SwrMeterLabel, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SwrMeterBar = lv_bar_create(ui_Screen1);
lv_bar_set_value(ui_SwrMeterBar,25,LV_ANIM_OFF);
lv_obj_set_width( ui_SwrMeterBar, 160);
lv_obj_set_height( ui_SwrMeterBar, 10);
lv_obj_set_x( ui_SwrMeterBar, -255 );
lv_obj_set_y( ui_SwrMeterBar, -153 );
lv_obj_set_align( ui_SwrMeterBar, LV_ALIGN_CENTER );
lv_obj_set_style_radius(ui_SwrMeterBar, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_SwrMeterBar, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_SwrTextLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_SwrTextLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SwrTextLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SwrTextLabel, -266 );
lv_obj_set_y( ui_SwrTextLabel, -135 );
lv_obj_set_align( ui_SwrTextLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SwrTextLabel,"1   1.5    2     3        5  ");
lv_obj_set_style_text_font(ui_SwrTextLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SignalStrengthLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_SignalStrengthLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SignalStrengthLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SignalStrengthLabel, -235 );
lv_obj_set_y( ui_SignalStrengthLabel, -225 );
lv_obj_set_align( ui_SignalStrengthLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SignalStrengthLabel,"1  3  5  7  9   +20   +40   +60dB");
lv_obj_clear_flag( ui_SignalStrengthLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_SignalStrengthLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_font(ui_SignalStrengthLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ALC = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_ALC, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_ALC, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_ALC, -371 );
lv_obj_set_y( ui_ALC, -181 );
lv_obj_set_align( ui_ALC, LV_ALIGN_CENTER );
lv_label_set_text(ui_ALC,"ALC");
lv_obj_set_style_text_align(ui_ALC, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_ALC, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_AfGainBar = lv_bar_create(ui_Screen1);
lv_bar_set_value(ui_AfGainBar,25,LV_ANIM_OFF);
lv_obj_set_width( ui_AfGainBar, 105);
lv_obj_set_height( ui_AfGainBar, 10);
lv_obj_set_x( ui_AfGainBar, 338 );
lv_obj_set_y( ui_AfGainBar, -214 );
lv_obj_set_align( ui_AfGainBar, LV_ALIGN_CENTER );
lv_obj_set_style_radius(ui_AfGainBar, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_AfGainBar, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_AfGainLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_AfGainLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_AfGainLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_AfGainLabel, 264 );
lv_obj_set_y( ui_AfGainLabel, -214 );
lv_obj_set_align( ui_AfGainLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_AfGainLabel,"AF");
lv_obj_set_style_text_font(ui_AfGainLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_RfGainLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_RfGainLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_RfGainLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_RfGainLabel, 264 );
lv_obj_set_y( ui_RfGainLabel, -194 );
lv_obj_set_align( ui_RfGainLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_RfGainLabel,"RF");
lv_obj_set_style_text_font(ui_RfGainLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_RfGainBar = lv_bar_create(ui_Screen1);
lv_bar_set_value(ui_RfGainBar,25,LV_ANIM_OFF);
lv_obj_set_width( ui_RfGainBar, 105);
lv_obj_set_height( ui_RfGainBar, 10);
lv_obj_set_x( ui_RfGainBar, 338 );
lv_obj_set_y( ui_RfGainBar, -194 );
lv_obj_set_align( ui_RfGainBar, LV_ALIGN_CENTER );
lv_obj_set_style_radius(ui_RfGainBar, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_RfGainBar, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_VfoAValue = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_VfoAValue, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_VfoAValue, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_VfoAValue, 20 );
lv_obj_set_y( ui_VfoAValue, -50 );
lv_obj_set_align( ui_VfoAValue, LV_ALIGN_CENTER );
lv_label_set_text(ui_VfoAValue,"14.012.000");
lv_obj_set_style_text_align(ui_VfoAValue, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_VfoAValue, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ModeDropDown = lv_dropdown_create(ui_Screen1);
lv_dropdown_set_options( ui_ModeDropDown, "USB\nLSB\nCW\nDATA\nFM\nAM" );
lv_dropdown_set_selected_highlight( ui_ModeDropDown, false);
lv_obj_set_width( ui_ModeDropDown, 70);
lv_obj_set_height( ui_ModeDropDown, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_ModeDropDown, 260 );
lv_obj_set_y( ui_ModeDropDown, -52 );
lv_obj_set_align( ui_ModeDropDown, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ModeDropDown, LV_OBJ_FLAG_SCROLL_WITH_ARROW | LV_OBJ_FLAG_SCROLL_ONE );   /// Flags
lv_obj_clear_flag( ui_ModeDropDown, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_ModeDropDown, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_align(ui_ModeDropDown, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_ModeDropDown, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_ModeDropDown, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_ModeDropDown, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ModeDropDown, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_ModeDropDown, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_ModeDropDown, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_ModeDropDown, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_ModeDropDown, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_ModeDropDown, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_ModeDropDown, 0, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_align(lv_dropdown_get_list(ui_ModeDropDown), LV_TEXT_ALIGN_LEFT,  LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(lv_dropdown_get_list(ui_ModeDropDown), &lv_font_montserrat_16,  LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(lv_dropdown_get_list(ui_ModeDropDown), 0,  LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_align(lv_dropdown_get_list(ui_ModeDropDown), LV_TEXT_ALIGN_RIGHT,  LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(lv_dropdown_get_list(ui_ModeDropDown), &lv_font_montserrat_16,  LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_radius(lv_dropdown_get_list(ui_ModeDropDown), 0,  LV_PART_SELECTED| LV_STATE_DEFAULT);

ui_VfoBValue = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_VfoBValue, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_VfoBValue, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_VfoBValue, 19 );
lv_obj_set_y( ui_VfoBValue, 3 );
lv_obj_set_align( ui_VfoBValue, LV_ALIGN_CENTER );
lv_label_set_text(ui_VfoBValue,"14.012.000");
lv_obj_set_style_text_align(ui_VfoBValue, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_VfoBValue, &lv_font_montserrat_38, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_VfoAuttonBackground = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_VfoAuttonBackground, 43);
lv_obj_set_height( ui_VfoAuttonBackground, 40);
lv_obj_set_x( ui_VfoAuttonBackground, -129 );
lv_obj_set_y( ui_VfoAuttonBackground, -50 );
lv_obj_set_align( ui_VfoAuttonBackground, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_VfoAuttonBackground, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_VfoAuttonBackground, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_radius(ui_VfoAuttonBackground, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_VfoALabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_VfoALabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_VfoALabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_VfoALabel, -129 );
lv_obj_set_y( ui_VfoALabel, -51 );
lv_obj_set_align( ui_VfoALabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_VfoALabel,"A");
lv_obj_set_style_text_align(ui_VfoALabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_VfoALabel, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_VfoALabel, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_VfoALabel, lv_color_hex(0x1A92F7), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_VfoALabel, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_border_color(ui_VfoALabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_border_opa(ui_VfoALabel, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_radius(ui_VfoALabel, 0, LV_PART_MAIN| LV_STATE_USER_1);
lv_obj_set_style_bg_color(ui_VfoALabel, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_USER_1 );
lv_obj_set_style_bg_opa(ui_VfoALabel, 255, LV_PART_MAIN| LV_STATE_USER_1);

ui_VfoBButtonBackground = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_VfoBButtonBackground, 43);
lv_obj_set_height( ui_VfoBButtonBackground, 40);
lv_obj_set_x( ui_VfoBButtonBackground, -129 );
lv_obj_set_y( ui_VfoBButtonBackground, -6 );
lv_obj_set_align( ui_VfoBButtonBackground, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_VfoBButtonBackground, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_VfoBButtonBackground, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_VfoBButtonBackground, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_radius(ui_VfoBButtonBackground, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_VfoBButtonBackground, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_VfoBButtonBackground, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_VfoBButtonBackground, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_grad_color(ui_VfoBButtonBackground, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_border_color(ui_VfoBButtonBackground, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_border_opa(ui_VfoBButtonBackground, 255, LV_PART_MAIN| LV_STATE_CHECKED);

ui_VfoBLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_VfoBLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_VfoBLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_VfoBLabel, -129 );
lv_obj_set_y( ui_VfoBLabel, -6 );
lv_obj_set_align( ui_VfoBLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_VfoBLabel,"B");
lv_obj_add_flag( ui_VfoBLabel, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_set_style_text_align(ui_VfoBLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_VfoBLabel, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_AntButton = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_AntButton, 53);
lv_obj_set_height( ui_AntButton, 30);
lv_obj_set_x( ui_AntButton, -364 );
lv_obj_set_y( ui_AntButton, -74 );
lv_obj_set_align( ui_AntButton, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_AntButton, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_AntButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Ant1Label = lv_label_create(ui_AntButton);
lv_obj_set_width( ui_Ant1Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Ant1Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Ant1Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_Ant1Label,"ANT 1");
lv_obj_clear_flag( ui_Ant1Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_Ant1Label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Ant1Label, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Ant2Label = lv_label_create(ui_AntButton);
lv_obj_set_width( ui_Ant2Label, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Ant2Label, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Ant2Label, LV_ALIGN_CENTER );
lv_label_set_text(ui_Ant2Label,"ANT 2");
lv_obj_add_flag( ui_Ant2Label, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_Ant2Label, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_Ant2Label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Ant2Label, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_UtcTime = lv_textarea_create(ui_Screen1);
lv_obj_set_width( ui_UtcTime, 221);
lv_obj_set_height( ui_UtcTime, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_UtcTime, 99 );
lv_obj_set_y( ui_UtcTime, -196 );
lv_obj_set_align( ui_UtcTime, LV_ALIGN_CENTER );
if ("0123456789:"=="") lv_textarea_set_accepted_chars(ui_UtcTime, NULL);
else lv_textarea_set_accepted_chars(ui_UtcTime, "0123456789:");
lv_textarea_set_placeholder_text(ui_UtcTime,"14:30:33");
lv_textarea_set_one_line(ui_UtcTime,true);
lv_obj_clear_flag( ui_UtcTime, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_color(ui_UtcTime, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_UtcTime, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_UtcTime, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_UtcTime, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_UtcTime, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_UtcTime, lv_color_hex(0x292831), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_UtcTime, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_UtcTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_UtcTime, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_UtcTime, lv_color_hex(0xFFFFFF), LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_UtcTime, 255, LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_UtcTime, LV_TEXT_ALIGN_LEFT, LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_UtcTime, 0, LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_UtcTime, lv_color_hex(0x292831), LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_UtcTime, 255, LV_PART_SELECTED| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_UtcTime, lv_color_hex(0x000000), LV_PART_SELECTED | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_UtcTime, 0, LV_PART_SELECTED| LV_STATE_DEFAULT);

lv_obj_set_style_border_color(ui_UtcTime, lv_color_hex(0x000000), LV_PART_CURSOR | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_UtcTime, 0, LV_PART_CURSOR| LV_STATE_DEFAULT);

lv_obj_set_style_text_color(ui_UtcTime, lv_color_hex(0xFFFFFF), LV_PART_TEXTAREA_PLACEHOLDER | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_UtcTime, 255, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_UtcTime, LV_TEXT_ALIGN_LEFT, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_UtcTime, &lv_font_montserrat_48, LV_PART_TEXTAREA_PLACEHOLDER| LV_STATE_DEFAULT);

ui_IfFilter = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_IfFilter, 53);
lv_obj_set_height( ui_IfFilter, 30);
lv_obj_set_x( ui_IfFilter, -364 );
lv_obj_set_y( ui_IfFilter, -37 );
lv_obj_set_align( ui_IfFilter, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_IfFilter, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_IfFilter, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_IfFilterALabel = lv_label_create(ui_IfFilter);
lv_obj_set_width( ui_IfFilterALabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_IfFilterALabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_IfFilterALabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_IfFilterALabel,"FIL A");
lv_obj_clear_flag( ui_IfFilterALabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_IfFilterALabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_IfFilterALabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_IfFilterBLabel = lv_label_create(ui_IfFilter);
lv_obj_set_width( ui_IfFilterBLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_IfFilterBLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_IfFilterBLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_IfFilterBLabel,"FIL B");
lv_obj_add_flag( ui_IfFilterBLabel, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_IfFilterBLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_IfFilterBLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_align(ui_IfFilterBLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_IfFilterBLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PwrValueLabel2 = lv_label_create(ui_IfFilter);
lv_obj_set_width( ui_PwrValueLabel2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_PwrValueLabel2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PwrValueLabel2, 68 );
lv_obj_set_y( ui_PwrValueLabel2, 3 );
lv_obj_set_align( ui_PwrValueLabel2, LV_ALIGN_CENTER );
lv_label_set_text(ui_PwrValueLabel2,"W");
lv_obj_set_style_text_font(ui_PwrValueLabel2, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PwrValueLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_PwrValueLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_PwrValueLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PwrValueLabel, -253 );
lv_obj_set_y( ui_PwrValueLabel, -225 );
lv_obj_set_align( ui_PwrValueLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_PwrValueLabel,"---10---25---50-----100W");
lv_obj_add_flag( ui_PwrValueLabel, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_set_style_text_color(ui_PwrValueLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_PwrValueLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_PwrValueLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_PwrLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_PwrLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_PwrLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_PwrLabel, -368 );
lv_obj_set_y( ui_PwrLabel, -208 );
lv_obj_set_align( ui_PwrLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_PwrLabel,"PWR");
lv_obj_add_flag( ui_PwrLabel, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_PwrLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_PwrLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_RxTxLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_RxTxLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_RxTxLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_RxTxLabel, -102 );
lv_obj_set_y( ui_RxTxLabel, -218 );
lv_obj_set_align( ui_RxTxLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_RxTxLabel,"RX");
lv_obj_add_flag( ui_RxTxLabel, LV_OBJ_FLAG_CLICKABLE );   /// Flags
lv_obj_set_scrollbar_mode(ui_RxTxLabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_RxTxLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_RxTxLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_RxTxLabel, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_RxTxLabel, lv_color_hex(0x2B9B25), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_RxTxLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_color(ui_RxTxLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_text_opa(ui_RxTxLabel, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_text_font(ui_RxTxLabel, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_RxTxLabel, lv_color_hex(0xD90202), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_RxTxLabel, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_pad_left(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_pad_right(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_pad_top(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_pad_bottom(ui_RxTxLabel, 1, LV_PART_MAIN| LV_STATE_CHECKED);

ui_SplitLabel = lv_label_create(ui_Screen1);
lv_obj_set_width( ui_SplitLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SplitLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SplitLabel, -82 );
lv_obj_set_y( ui_SplitLabel, -92 );
lv_obj_set_align( ui_SplitLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SplitLabel,"Split");
lv_obj_clear_flag( ui_SplitLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_SplitLabel, LV_SCROLLBAR_MODE_OFF);

ui_PreAmpButton = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_PreAmpButton, 53);
lv_obj_set_height( ui_PreAmpButton, 30);
lv_obj_set_x( ui_PreAmpButton, -364 );
lv_obj_set_y( ui_PreAmpButton, 0 );
lv_obj_set_align( ui_PreAmpButton, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_PreAmpButton, LV_OBJ_FLAG_CHECKABLE );   /// Flags
lv_obj_clear_flag( ui_PreAmpButton, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_PreAmpButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_PreAmpButton, lv_color_hex(0x292831), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_PreAmpButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_PreAmpButton, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_PreAmpButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_PreAmpButton, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_radius(ui_PreAmpButton, 0, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);
lv_obj_set_style_bg_color(ui_PreAmpButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED|LV_STATE_PRESSED );
lv_obj_set_style_bg_opa(ui_PreAmpButton, 255, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);

ui_PreAmpButtonLabel = lv_label_create(ui_PreAmpButton);
lv_obj_set_width( ui_PreAmpButtonLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_PreAmpButtonLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_PreAmpButtonLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_PreAmpButtonLabel,"PRE");
lv_obj_clear_flag( ui_PreAmpButtonLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_PreAmpButtonLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_PreAmpButtonLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_AttButton = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_AttButton, 53);
lv_obj_set_height( ui_AttButton, 30);
lv_obj_set_x( ui_AttButton, -364 );
lv_obj_set_y( ui_AttButton, 35 );
lv_obj_set_align( ui_AttButton, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_AttButton, LV_OBJ_FLAG_CHECKABLE );   /// Flags
lv_obj_clear_flag( ui_AttButton, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_AttButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_AttButton, lv_color_hex(0x292831), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_AttButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_AttButton, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_AttButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_AttButton, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_radius(ui_AttButton, 0, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);
lv_obj_set_style_bg_color(ui_AttButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED|LV_STATE_PRESSED );
lv_obj_set_style_bg_opa(ui_AttButton, 255, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);

ui_AttButtonLabel = lv_label_create(ui_AttButton);
lv_obj_set_width( ui_AttButtonLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_AttButtonLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_AttButtonLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_AttButtonLabel,"ATT");
lv_obj_clear_flag( ui_AttButtonLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_AttButtonLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_AttButtonLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_AtTuneButton = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_AtTuneButton, 53);
lv_obj_set_height( ui_AtTuneButton, 30);
lv_obj_set_x( ui_AtTuneButton, -363 );
lv_obj_set_y( ui_AtTuneButton, 70 );
lv_obj_set_align( ui_AtTuneButton, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_AtTuneButton, LV_OBJ_FLAG_CHECKABLE );   /// Flags
lv_obj_clear_flag( ui_AtTuneButton, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_AtTuneButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_AtTuneButton, lv_color_hex(0x292831), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_AtTuneButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_AtTuneButton, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_AtTuneButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_AtTuneButton, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_radius(ui_AtTuneButton, 0, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);
lv_obj_set_style_bg_color(ui_AtTuneButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED|LV_STATE_PRESSED );
lv_obj_set_style_bg_opa(ui_AtTuneButton, 255, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);

ui_AtTuneButtonLabel = lv_label_create(ui_AtTuneButton);
lv_obj_set_width( ui_AtTuneButtonLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_AtTuneButtonLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_AtTuneButtonLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_AtTuneButtonLabel,"AT");
lv_obj_clear_flag( ui_AtTuneButtonLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_AtTuneButtonLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_AtTuneButtonLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ProcButton = lv_btn_create(ui_Screen1);
lv_obj_set_width( ui_ProcButton, 53);
lv_obj_set_height( ui_ProcButton, 30);
lv_obj_set_x( ui_ProcButton, -363 );
lv_obj_set_y( ui_ProcButton, 104 );
lv_obj_set_align( ui_ProcButton, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_ProcButton, LV_OBJ_FLAG_CHECKABLE );   /// Flags
lv_obj_clear_flag( ui_ProcButton, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_ProcButton, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_ProcButton, lv_color_hex(0x292831), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_ProcButton, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_radius(ui_ProcButton, 0, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_ProcButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_ProcButton, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_radius(ui_ProcButton, 0, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);
lv_obj_set_style_bg_color(ui_ProcButton, lv_color_hex(0x2095F6), LV_PART_MAIN | LV_STATE_CHECKED|LV_STATE_PRESSED );
lv_obj_set_style_bg_opa(ui_ProcButton, 255, LV_PART_MAIN| LV_STATE_CHECKED|LV_STATE_PRESSED);

ui_ProcButtonLabel = lv_label_create(ui_ProcButton);
lv_obj_set_width( ui_ProcButtonLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_ProcButtonLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_ProcButtonLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_ProcButtonLabel,"PROC");
lv_obj_clear_flag( ui_ProcButtonLabel, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_align(ui_ProcButtonLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_ProcButtonLabel, &lv_font_montserrat_14, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_Switch1, ui_event_Switch1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ModeDropDown, ui_event_ModeDropDown, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_AntButton, ui_event_AntButton, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_IfFilter, ui_event_IfFilter, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_RxTxLabel, ui_event_RxTxLabel, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_PreAmpButton, ui_event_PreAmpButton, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_AttButton, ui_event_AttButton, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_AtTuneButton, ui_event_AtTuneButton, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ProcButton, ui_event_ProcButton, LV_EVENT_ALL, NULL);

}
