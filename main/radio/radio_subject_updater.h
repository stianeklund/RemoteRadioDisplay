/**
 * @file radio_subject_updater.h
 * @brief Thread-safe subject update API for non-LVGL tasks
 *
 * This module provides functions to safely update lv_subject_t instances
 * from tasks other than the LVGL task. Updates are queued and processed
 * in the LVGL task context during lvgl_drain_messages().
 *
 * Usage:
 *   // From CAT parser task:
 *   radio_subject_set_int_async(&radio_s_meter_subject, sm_value);
 *
 *   // For POINTER subjects with data copy:
 *   radio_subject_set_pointer_async(&radio_if_data_subject, &if_data, sizeof(if_data));
 *
 *   // For POINTER subjects with buffer notification (data already in buffer):
 *   radio_subject_notify_async(&radio_vfo_consolidated_subject);
 */
#ifndef RADIO_SUBJECT_UPDATER_H
#define RADIO_SUBJECT_UPDATER_H

#include "lvgl.h"
#include "esp_err.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Queue Item Types
// ============================================================================

/** Subject update type */
typedef enum {
    SUBJECT_UPDATE_INT,      /**< Update INT subject value */
    SUBJECT_UPDATE_FLOAT,    /**< Update FLOAT subject value */
    SUBJECT_UPDATE_POINTER,  /**< Update POINTER subject (copy data to buffer) */
    SUBJECT_UPDATE_NOTIFY,   /**< Notify POINTER subject (data already in buffer) */
} subject_update_type_t;

// ============================================================================
// Async Update Functions (Thread-Safe)
// ============================================================================

/**
 * @brief Queue an INT subject update from a non-LVGL task
 *
 * @param subject Pointer to the subject to update
 * @param value New value
 * @return true if queued successfully, false if queue full
 */
bool radio_subject_set_int_async(lv_subject_t* subject, int32_t value);

/**
 * @brief Queue a FLOAT subject update from a non-LVGL task
 *
 * @param subject Pointer to the subject to update
 * @param value New value
 * @return true if queued successfully, false if queue full
 */
bool radio_subject_set_float_async(lv_subject_t* subject, float value);

/**
 * @brief Queue a POINTER subject update with data copy
 *
 * Data is copied to an intermediate buffer and then to the subject's
 * static buffer during dispatch in the LVGL task.
 *
 * @param subject Pointer to the subject to update
 * @param data Pointer to source data
 * @param len Size of data in bytes
 * @return true if queued successfully, false if queue full or data too large
 */
bool radio_subject_set_pointer_async(lv_subject_t* subject, const void* data, size_t len);

/**
 * @brief Queue a subject notification (no value change, just notify observers)
 *
 * Use this when you've already updated the subject's static buffer
 * and just need to notify observers.
 *
 * @param subject Pointer to the subject to notify
 * @return true if queued successfully, false if queue full
 */
bool radio_subject_notify_async(lv_subject_t* subject);

// ============================================================================
// Direct Update Functions (LVGL Task Only)
// ============================================================================

/**
 * @brief Set INT subject value directly (must be called from LVGL task)
 *
 * @param subject Pointer to the subject to update
 * @param value New value
 */
void radio_subject_set_int_direct(lv_subject_t* subject, int32_t value);

/**
 * @brief Set FLOAT subject value directly (must be called from LVGL task)
 *
 * @param subject Pointer to the subject to update
 * @param value New value
 */
void radio_subject_set_float_direct(lv_subject_t* subject, float value);

/**
 * @brief Set POINTER subject data directly (must be called from LVGL task)
 *
 * @param subject Pointer to the subject to update
 * @param data Pointer to new data (will be set as subject's pointer value)
 */
void radio_subject_set_pointer_direct(lv_subject_t* subject, void* data);

/**
 * @brief Notify subject observers directly (must be called from LVGL task)
 *
 * @param subject Pointer to the subject to notify
 */
void radio_subject_notify_direct(lv_subject_t* subject);

// ============================================================================
// Queue Management
// ============================================================================

/**
 * @brief Process pending subject updates
 *
 * Called from lvgl_drain_messages() to dispatch queued subject updates
 * in the LVGL task context.
 *
 * @return Number of updates processed
 */
int radio_subject_drain_updates(void);

/**
 * @brief Get number of pending subject updates
 *
 * @return Number of items in the update queue
 */
int radio_subject_pending_count(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_SUBJECT_UPDATER_H
