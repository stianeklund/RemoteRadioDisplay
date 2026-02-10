#include "cat_state.hpp"

/**
 * @brief Singleton accessor for RadioState
 *
 * Uses static local variable for thread-safe initialization (C++11 magic statics).
 * The RadioState is allocated in internal SRAM for fast atomic access.
 */
RadioState& radio_state() {
    static RadioState state;
    return state;
}

// === C-compatible getter implementations ===

bool radio_get_transmit_status(void) {
    return radio_state().is_transmitting.load(std::memory_order_acquire);
}

bool radio_get_split_status(void) {
    return radio_state().split_enabled.load(std::memory_order_acquire);
}

int radio_get_rx_vfo_function(void) {
    return radio_state().rx_vfo_function.load(std::memory_order_acquire);
}

int radio_get_tx_vfo_function(void) {
    return radio_state().tx_vfo_function.load(std::memory_order_acquire);
}

uint32_t radio_get_vfo_a_hz(void) {
    return radio_state().vfo_a_hz.load(std::memory_order_acquire);
}

uint32_t radio_get_vfo_b_hz(void) {
    return radio_state().vfo_b_hz.load(std::memory_order_acquire);
}

bool radio_get_tfset_active(void) {
    return radio_state().tfset_active.load(std::memory_order_acquire);
}

// === C-compatible setter implementations ===

void radio_set_transmit_status(bool tx) {
    radio_state().is_transmitting.store(tx, std::memory_order_release);
}

void radio_set_split_status(bool split) {
    radio_state().split_enabled.store(split, std::memory_order_release);
}

void radio_set_rx_vfo_function(int vfo) {
    radio_state().rx_vfo_function.store(static_cast<int8_t>(vfo), std::memory_order_release);
}

void radio_set_tx_vfo_function(int vfo) {
    radio_state().tx_vfo_function.store(static_cast<int8_t>(vfo), std::memory_order_release);
}

void radio_set_vfo_a_hz(uint32_t freq) {
    radio_state().vfo_a_hz.store(freq, std::memory_order_release);
}

void radio_set_vfo_b_hz(uint32_t freq) {
    radio_state().vfo_b_hz.store(freq, std::memory_order_release);
}

void radio_set_tfset_active(bool active) {
    radio_state().tfset_active.store(active, std::memory_order_release);
}

void radio_set_highcut_index(uint8_t idx) {
    radio_state().highcut_index.store(idx, std::memory_order_release);
}

void radio_set_lowcut_index(uint8_t idx) {
    radio_state().lowcut_index.store(idx, std::memory_order_release);
}

void radio_set_ssb_filter_mode(uint8_t mode) {
    radio_state().ssb_filter_mode.store(mode, std::memory_order_release);
}

void radio_set_ssb_data_filter_mode(uint8_t mode) {
    radio_state().ssb_data_filter_mode.store(mode, std::memory_order_release);
}

uint8_t radio_get_ssb_filter_mode(void) {
    return radio_state().ssb_filter_mode.load(std::memory_order_acquire);
}

uint8_t radio_get_ssb_data_filter_mode(void) {
    return radio_state().ssb_data_filter_mode.load(std::memory_order_acquire);
}

void radio_set_antenna_select(int ant) {
    radio_state().antenna_select.store(static_cast<int8_t>(ant), std::memory_order_release);
}

void radio_set_rx_ant_used(bool used) {
    radio_state().rx_ant_used.store(used, std::memory_order_release);
}

void radio_set_drive_out_on(bool on) {
    radio_state().drive_out_on.store(on, std::memory_order_release);
}
