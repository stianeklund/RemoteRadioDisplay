#pragma once
/**
 * @file cat_state.hpp
 * @brief Consolidated radio state with thread-safe atomic access
 *
 * This module provides a centralized RadioState structure that consolidates
 * previously scattered global variables from cat_parser.cpp. All fields use
 * std::atomic for lock-free thread-safe access between the CAT parser task
 * (Core 0) and LVGL task (Core 1).
 */

#include <cstdint>
#include <atomic>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Consolidated radio state with atomic fields for thread safety
 *
 * ESP32-S3 natively supports atomic operations on 32-bit values.
 * Using std::atomic provides lock-free access without mutex overhead.
 */
struct RadioState {
    // === Transceiver status ===
    std::atomic<bool> is_transmitting{false};    // TX/RX status
    std::atomic<bool> tfset_active{false};       // TF-Set mode engaged
    std::atomic<bool> split_enabled{false};      // Split mode active

    // === VFO state ===
    std::atomic<int8_t> rx_vfo_function{0};      // 0=A, 1=B, 2=MEM
    std::atomic<int8_t> tx_vfo_function{0};      // 0=A, 1=B, 2=MEM
    std::atomic<uint32_t> vfo_a_hz{0};           // VFO-A frequency (raw radio-side)
    std::atomic<uint32_t> vfo_b_hz{0};           // VFO-B frequency (raw radio-side)

    // === Filter state ===
    std::atomic<uint8_t> highcut_index{0};       // Current high-cut filter index
    std::atomic<uint8_t> lowcut_index{0};        // Current low-cut filter index
    std::atomic<uint8_t> ssb_filter_mode{0};     // EX028: 0=Hi/Lo Cut, 1=Width/Shift (SSB)
    std::atomic<uint8_t> ssb_data_filter_mode{0};// EX029: 0=Hi/Lo Cut, 1=Width/Shift (SSB-DATA)

    // === Antenna state ===
    std::atomic<int8_t> antenna_select{0};       // 0=ANT1, 1=ANT2
    std::atomic<bool> rx_ant_used{false};        // RX antenna in use
    std::atomic<bool> drive_out_on{false};       // Drive output enabled

    // === Display communication control ===
    std::atomic<bool> display_communication_enabled{true}; // UIDE command: enable/disable display comms
};

/**
 * @brief Get the singleton RadioState instance
 * @return Reference to the global RadioState
 */
RadioState& radio_state();

// === C-compatible getter functions ===
// These maintain API compatibility with existing code

bool radio_get_transmit_status(void);
bool radio_get_split_status(void);
int radio_get_rx_vfo_function(void);
int radio_get_tx_vfo_function(void);
uint32_t radio_get_vfo_a_hz(void);
uint32_t radio_get_vfo_b_hz(void);
bool radio_get_tfset_active(void);

// === C-compatible setter functions ===
void radio_set_transmit_status(bool tx);
void radio_set_split_status(bool split);
void radio_set_rx_vfo_function(int vfo);
void radio_set_tx_vfo_function(int vfo);
void radio_set_vfo_a_hz(uint32_t freq);
void radio_set_vfo_b_hz(uint32_t freq);
void radio_set_tfset_active(bool active);
void radio_set_highcut_index(uint8_t idx);
void radio_set_lowcut_index(uint8_t idx);
void radio_set_ssb_filter_mode(uint8_t mode);
void radio_set_ssb_data_filter_mode(uint8_t mode);
uint8_t radio_get_ssb_filter_mode(void);
uint8_t radio_get_ssb_data_filter_mode(void);
void radio_set_antenna_select(int ant);
void radio_set_rx_ant_used(bool used);
void radio_set_drive_out_on(bool on);

#ifdef __cplusplus
}
#endif
