#ifndef CAT_SHARED_TYPES_H
#define CAT_SHARED_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// Structure to hold all data from the IF command
// THIS IS THE SINGLE SOURCE OF TRUTH
typedef struct {
    long long vfo_frequency;
    int rit_xit_frequency;
    bool rit_on;
    bool xit_on;
    int memory_bank_channel;
    bool tx_rx; // true if transmitting, false if receiving
    int mode;
    int function; // e.g., VFO A/B, MR
    bool scan_on;
    bool split_on;
    bool tone_on;
    int tone_number;
    int shift_status;
    int p15_value;
    int memory_channel;
} kenwood_if_data_t;

// Structure to hold data from the XI command
typedef struct {
    uint32_t transmit_frequency;
    int      transmission_mode;
    bool     data_mode_on;
} kenwood_xi_data_t;

// Structure for Antenna Tuner status
typedef struct {
    bool rx_at_in;
    bool tx_at_in;
    bool tuning_in_progress;
} ui_at_status_t;

// Structure for transverter offset data from XO command
typedef struct {
    uint64_t offset_frequency;  // Offset frequency in Hz (11 digits)
    bool direction_plus;        // true = Plus (+), false = Minus (-)
    bool valid;                 // true if XO command data is valid
} transverter_xo_data_t;

// Structure for transverter enable status from EX056 command  
typedef struct {
    bool enabled;               // true if transverter is enabled
    bool power_down_mode;       // true if in power-down mode (EX056=2)
    bool valid;                 // true if EX056 command data is valid
} transverter_ex056_data_t;

// Combined transverter state
typedef struct {
    transverter_xo_data_t xo_data;
    transverter_ex056_data_t ex056_data;
} transverter_state_t;

// Structure to hold data from the MR (Memory Read) command
typedef struct {
    uint16_t channel;           // Memory channel number (0-999)
    uint32_t frequency;         // Frequency in Hz
    uint8_t mode;               // Operating mode (MD enum: 1=LSB, 2=USB, etc.)
    bool data_mode;             // DATA mode flag (DA: 0=OFF, 1=ON)
    uint8_t tone_mode;          // Tone mode (0=OFF, 1=TONE, 2=CTCSS, 3=Cross)
    uint8_t tone_freq_index;    // Tone frequency index (TN)
    uint8_t ctcss_freq_index;   // CTCSS frequency index (CN)
    uint8_t filter;             // Filter A/B (0=A, 1=B)
    bool fm_narrow;             // FM narrow flag
    bool lockout;               // Channel lockout flag
    bool split;                 // Split operation flag
    char name[9];               // Memory name (up to 8 ASCII chars + null)
    bool valid;                 // true if data has been populated
} memory_channel_data_t;

#endif // CAT_SHARED_TYPES_H
