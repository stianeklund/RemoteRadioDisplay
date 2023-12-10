// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.3.6
// Project name: Sunton_SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
lv_obj_t *ui_Screen1;
lv_obj_t *ui_SignalPanel;
lv_obj_t *ui_SMeterBar;
lv_obj_t *ui_SMeterLabel;
void ui_event_Switch1( lv_event_t * e);
lv_obj_t *ui_Switch1;
lv_obj_t *ui_SwrMeterLabel;
lv_obj_t *ui_SwrMeterBar;
lv_obj_t *ui_SwrTextLabel;
lv_obj_t *ui_SignalStrengthLabel;
lv_obj_t *ui_ALC;
lv_obj_t *ui_AfGainBar;
lv_obj_t *ui_AfGainLabel;
lv_obj_t *ui_RfGainLabel;
lv_obj_t *ui_RfGainBar;
lv_obj_t *ui_VfoAValue;
void ui_event_ModeDropDown( lv_event_t * e);
lv_obj_t *ui_ModeDropDown;
lv_obj_t *ui_VfoBValue;
lv_obj_t *ui_VfoAuttonBackground;
lv_obj_t *ui_VfoALabel;
lv_obj_t *ui_VfoBButtonBackground;
lv_obj_t *ui_VfoBLabel;
void ui_event_AntButton( lv_event_t * e);
lv_obj_t *ui_AntButton;
lv_obj_t *ui_Ant1Label;
lv_obj_t *ui_Ant2Label;
lv_obj_t *ui_UtcTime;
void ui_event_IfFilter( lv_event_t * e);
lv_obj_t *ui_IfFilter;
lv_obj_t *ui_IfFilterALabel;
lv_obj_t *ui_IfFilterBLabel;
lv_obj_t *ui_PwrValueLabel2;
lv_obj_t *ui_PwrValueLabel;
lv_obj_t *ui_PwrLabel;
void ui_event_RxTxLabel( lv_event_t * e);
lv_obj_t *ui_RxTxLabel;
lv_obj_t *ui_SplitLabel;
void ui_event_PreAmpButton( lv_event_t * e);
lv_obj_t *ui_PreAmpButton;
lv_obj_t *ui_PreAmpButtonLabel;
void ui_event_AttButton( lv_event_t * e);
lv_obj_t *ui_AttButton;
lv_obj_t *ui_AttButtonLabel;
void ui_event_AtTuneButton( lv_event_t * e);
lv_obj_t *ui_AtTuneButton;
lv_obj_t *ui_AtTuneButtonLabel;
void ui_event_ProcButton( lv_event_t * e);
lv_obj_t *ui_ProcButton;
lv_obj_t *ui_ProcButtonLabel;


// SCREEN: ui_creen2
// void ui_Screen2_screen_init(void);
// lv_obj_t *ui_Screen2;
lv_obj_t *ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Switch1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      _ui_flag_modify( ui_VfoAuttonBackground, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
      _ui_flag_modify( ui_VfoBButtonBackground, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      _ui_flag_modify( ui_VfoAuttonBackground, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      _ui_flag_modify( ui_VfoBButtonBackground, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
}
}
void ui_event_ModeDropDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      changeMode( e );
}
}
void ui_event_AntButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      changeAntenna( e );
      _ui_flag_modify( ui_Ant2Label, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
      _ui_flag_modify( ui_Ant1Label, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
}
}
void ui_event_IfFilter( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      changeFilter( e );
      _ui_flag_modify( ui_IfFilterALabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
      _ui_flag_modify( ui_IfFilterBLabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
}
}
void ui_event_RxTxLabel( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_checked_set_text_value( ui_RxTxLabel, target, "RX", "TX");
      _ui_state_modify( ui_RxTxLabel, LV_STATE_CHECKED, _UI_MODIFY_STATE_TOGGLE);
      _ui_flag_modify( ui_SignalStrengthLabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
      _ui_flag_modify( ui_PwrValueLabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
      _ui_flag_modify( ui_SMeterLabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
      _ui_flag_modify( ui_PwrLabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
}
}
void ui_event_PreAmpButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      TogglePreAmp( e );
}
}
void ui_event_AttButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      TogglePreAmp( e );
}
}
void ui_event_AtTuneButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      AtTune( e );
}
}
void ui_event_ProcButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      TogglePreAmp( e );
}
}

///////////////////// SCREENS ////////////////////

void ui_init( void )
{
lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);
ui_Screen1_screen_init();
// ui_Screen2_screen_init();
ui____initial_actions0 = lv_obj_create(NULL);
lv_disp_load_scr( ui_Screen1);
}
