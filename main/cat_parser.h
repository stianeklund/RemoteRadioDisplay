#ifndef CAT_PARSER_H
#define CAT_PARSER_H 

#include "esp_err.h"
#include <stdint.h>
#include "esp_log.h"
#include <stdbool.h>
#include "cat_shared_types.h" // Include the shared types

// Consolidated VFO update payload
typedef struct {
    uint32_t active_freq;     // Frequency for main display
    uint32_t inactive_freq;   // Frequency for small label
    int8_t active_vfo;        // Which VFO is active (0=A, 1=B, 2=MEM)
} vfo_update_t;

// Notch filter mode enumeration
typedef enum {
    NOTCH_OFF = 0,
    NOTCH_AUTO = 1,
    NOTCH_MANUAL_NORMAL = 2,
    NOTCH_MANUAL_WIDE = 3
} notch_mode_t;

// Structure to hold USB/LSB filter width data
typedef struct {
    int low_cut;
    int high_cut;
} filter_usb_width_t;

// TS-590 Power Calibration Table
typedef struct {
    int raw_value;
    float watts;
} power_cal_point_t;

typedef struct {
    int num_points;
    power_cal_point_t points[6];
} power_cal_table_t;

#define TS590_PWR_CAL { 6, { \
    {  0,  0.0f }, \
    {  3,  5.0f }, \
    {  6, 10.0f }, \
    { 12, 25.0f }, \
    { 18, 50.0f }, \
    { 30,100.0f }  \
    } }

// Structure to hold PEP (Peak Envelope Power) data
typedef struct {
    int current_power_raw;      // Current instantaneous power (0-30 raw)
    int pep_power_raw;          // Peak envelope power (0-30 raw)
    float current_power_watts;  // Current power in watts
    float pep_power_watts;      // Peak power in watts
    uint32_t pep_timestamp;     // Timestamp when peak was recorded
    bool enabled;               // PEP tracking enabled flag
} pep_data_t;

// kenwood_if_data_t is now defined in cat_shared_types.h
// kenwood_xi_data_t is now defined in cat_shared_types.h

bool cat_get_transmit_status(void); // Getter for transmit status
bool cat_get_split_status(void); // Getter for split status
int cat_get_rx_vfo_function(void); // Getter for RX VFO function
esp_err_t parse_command(void);
void parse_if_command(const char* response);
void parse_ri_command(const char* response);
void parse_rm_command(const char* response);
void parse_sm_command(const char* response);
void parse_ai_command(const char* response);
// uint32_t parse_frequency(const char* frequency);
esp_err_t parse_frequency(const char *freq_str, uint32_t *frequency);
int parse_mode(const char* response);
int parse_filter(const char* response);
bool parse_rit_status(const char* response);
int parse_rit_frequency(const char* response);
bool parse_xit_status(const char* response);
int parse_xit_frequency(const char* response);
bool parse_split_operation_status(const char* response);  // SP cmd: split freq operation, NOT split mode
uint32_t parse_tx_frequency(const char* response);
int parse_agc(const char* response);
int parse_power(const char* response);
int parse_preamp(const char* response);
int parse_att(const char* response);
bool parse_nb_status(const char* response);
int parse_nr_status(const char* response); // Returns NR level: 0 (OFF), 1 (NR1), 2 (NR2), -1 (error)
bool parse_bc_status(const char* response);
notch_mode_t parse_notch_status(const char* response);
int parse_notch_frequency(const char* response);
int parse_cw_speed(const char* response);
bool parse_vox_status(const char* response);
int parse_vox_gain(const char* response);
void parse_af_gain(const char* response);
void parse_rf_gain(const char* response);
void parse_xi_command(const char* response);
void parse_cat_command(const char* response);
void parse_xo_command(const char* response);
void parse_ex_command(const char* response);
void parse_mr_command(const char* response);
// Memory channel accessor
memory_channel_data_t* get_memory_channel_data(void);
// Transverter helper functions
bool transverter_is_enabled(void);
uint64_t apply_transverter_offset_for_display(uint64_t radio_frequency);
uint64_t compute_radio_frequency_from_display(uint64_t display_frequency);
transverter_state_t* get_transverter_state(void);
// Request fresh VFO frequencies from radio (sends FA; and FB; commands)
void cat_request_transverter_display_refresh(void);
// Request fresh transverter configuration from radio (XO and EX056 commands)
void cat_request_transverter_config_update(void);
// Request filter mode settings (EX028 for SSB, EX029 for SSB-DATA)
void cat_request_filter_mode_update(void);
// Request CW menu settings (EX006, EX040)
void cat_request_cw_menu_update(void);
// cat_parser_init_activity_monitor is removed
bool cat_check_and_reset_activity_flag(void); // Check and reset CAT activity flag

// Initialize CAT parser subsystem (call early in app_main)
esp_err_t cat_parser_init(void);

// PEP (Peak Envelope Power) functions
void pep_init(void);
void pep_enable(bool enabled);

// Initialize UI message bridge (called from LVGL context, e.g., ui_init)
void cat_ui_bridge_start(void);
void pep_update_power(int power_value);
pep_data_t* pep_get_data(void);
void pep_reset(void);
float convert_raw_power_to_watts(int raw_power);

#endif // CAT_PARSER_H 
