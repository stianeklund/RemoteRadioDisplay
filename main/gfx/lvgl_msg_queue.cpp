/**
 * @file lvgl_msg_queue.cpp
 * @brief LVGL message queue - DEPRECATED
 *
 * This file previously provided a FreeRTOS queue for dispatching LVGL messages
 * from non-LVGL tasks. It has been superseded by the LVGL 9 native observer
 * pattern implemented in radio/radio_subject_updater.cpp.
 *
 * The functions are retained as stubs for API compatibility but do nothing.
 * New code should use radio_subject_set_*_async() functions instead.
 */

#include "lvgl_msg_queue.h"
#include "radio/radio_subject_updater.h"

void lvgl_msg_queue_init(void) {
    // No-op: Legacy message queue no longer used
    // Subject updates are handled by radio_subject_updater
}

bool lvgl_post_message(uint32_t id, const void *data, size_t len) {
    // No-op: Legacy message posting no longer supported
    // Use radio_subject_set_*_async() functions instead
    (void)id;
    (void)data;
    (void)len;
    return false;
}

void lvgl_drain_messages(void) {
    // Only drain subject updates (the new observer pattern)
    radio_subject_drain_updates();
}
