/**
 * @file radio_subject_updater.cpp
 * @brief Thread-safe subject update implementation using FreeRTOS queue
 *
 * Updates from non-LVGL tasks are queued and processed during
 * radio_subject_drain_updates() called from the LVGL task.
 */

#include "radio_subject_updater.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "RADIO_SUBJECT_UPD";

// Maximum size for inline payload (larger payloads use heap)
#define INLINE_PAYLOAD_SIZE 48

// Queue item structure
typedef struct {
    lv_subject_t* subject;
    subject_update_type_t type;
    uint8_t flags;  // bit0: heap_payload
    union {
        int32_t int_value;
        float float_value;
        uint8_t inline_bytes[INLINE_PAYLOAD_SIZE];
        void* heap_ptr;
    } payload;
    uint16_t payload_len;  // For POINTER type
} subject_update_item_t;

// Queue handle
static QueueHandle_t s_update_queue = NULL;
static bool s_initialized = false;

// Lazy initialization of queue
static bool ensure_queue_initialized(void)
{
    if (s_update_queue != NULL) {
        return true;
    }

    // Create queue with depth 64 - same as lvgl_msg_queue
    s_update_queue = xQueueCreate(64, sizeof(subject_update_item_t));
    if (s_update_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create subject update queue");
        return false;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Subject update queue initialized (depth=64)");
    return true;
}

// ============================================================================
// Async Update Functions (Thread-Safe)
// ============================================================================

bool radio_subject_set_int_async(lv_subject_t* subject, int32_t value)
{
    if (!ensure_queue_initialized() || subject == NULL) {
        return false;
    }

    subject_update_item_t item = {0};
    item.subject = subject;
    item.type = SUBJECT_UPDATE_INT;
    item.flags = 0;
    item.payload.int_value = value;

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        ESP_LOGD(TAG, "Subject update queue full (INT)");
        return false;
    }
    return true;
}

bool radio_subject_set_float_async(lv_subject_t* subject, float value)
{
#if LV_USE_FLOAT
    if (!ensure_queue_initialized() || subject == NULL) {
        return false;
    }

    subject_update_item_t item = {0};
    item.subject = subject;
    item.type = SUBJECT_UPDATE_FLOAT;
    item.flags = 0;
    item.payload.float_value = value;

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        ESP_LOGD(TAG, "Subject update queue full (FLOAT)");
        return false;
    }
    return true;
#else
    (void)subject;
    (void)value;
    ESP_LOGW(TAG, "FLOAT subjects not supported (LV_USE_FLOAT=0)");
    return false;
#endif
}

bool radio_subject_set_pointer_async(lv_subject_t* subject, const void* data, size_t len)
{
    if (!ensure_queue_initialized() || subject == NULL) {
        return false;
    }

    if (data == NULL || len == 0) {
        // Just notify without data
        return radio_subject_notify_async(subject);
    }

    subject_update_item_t item = {0};
    item.subject = subject;
    item.type = SUBJECT_UPDATE_POINTER;
    item.payload_len = (uint16_t)len;

    if (len <= INLINE_PAYLOAD_SIZE) {
        // Inline copy
        item.flags = 0;
        memcpy(item.payload.inline_bytes, data, len);
    } else {
        // Heap allocation for large payloads (use PSRAM if available)
        void* p = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (p == NULL) {
            // Fallback to internal RAM
            p = heap_caps_malloc(len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (p == NULL) {
            ESP_LOGE(TAG, "Failed to allocate %u bytes for subject payload", (unsigned)len);
            return false;
        }
        memcpy(p, data, len);
        item.flags = 0x01;  // Heap payload flag
        item.payload.heap_ptr = p;
    }

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        // Free heap payload if queue is full
        if (item.flags & 0x01) {
            heap_caps_free(item.payload.heap_ptr);
        }
        ESP_LOGD(TAG, "Subject update queue full (POINTER)");
        return false;
    }
    return true;
}

bool radio_subject_notify_async(lv_subject_t* subject)
{
    if (!ensure_queue_initialized() || subject == NULL) {
        return false;
    }

    subject_update_item_t item = {0};
    item.subject = subject;
    item.type = SUBJECT_UPDATE_NOTIFY;
    item.flags = 0;

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        ESP_LOGD(TAG, "Subject update queue full (NOTIFY)");
        return false;
    }
    return true;
}

// ============================================================================
// Direct Update Functions (LVGL Task Only)
// ============================================================================

void radio_subject_set_int_direct(lv_subject_t* subject, int32_t value)
{
    if (subject != NULL) {
        lv_subject_set_int(subject, value);
    }
}

void radio_subject_set_float_direct(lv_subject_t* subject, float value)
{
#if LV_USE_FLOAT
    if (subject != NULL) {
        lv_subject_set_float(subject, value);
    }
#else
    (void)subject;
    (void)value;
#endif
}

void radio_subject_set_pointer_direct(lv_subject_t* subject, void* data)
{
    if (subject != NULL) {
        lv_subject_set_pointer(subject, data);
    }
}

void radio_subject_notify_direct(lv_subject_t* subject)
{
    if (subject != NULL) {
        lv_subject_notify(subject);
    }
}

// ============================================================================
// Queue Management
// ============================================================================

int radio_subject_drain_updates(void)
{
    if (s_update_queue == NULL) {
        return 0;
    }

    subject_update_item_t item;
    int processed = 0;
    const int MAX_PER_DRAIN = 32;  // Limit to prevent blocking LVGL

    while (processed < MAX_PER_DRAIN &&
           xQueueReceive(s_update_queue, &item, 0) == pdTRUE) {

        if (item.subject == NULL) {
            // Skip invalid items
            if ((item.flags & 0x01) && item.payload.heap_ptr) {
                heap_caps_free(item.payload.heap_ptr);
            }
            continue;
        }

        switch (item.type) {
            case SUBJECT_UPDATE_INT:
                lv_subject_set_int(item.subject, item.payload.int_value);
                break;

            case SUBJECT_UPDATE_FLOAT:
#if LV_USE_FLOAT
                lv_subject_set_float(item.subject, item.payload.float_value);
#endif
                break;

            case SUBJECT_UPDATE_POINTER: {
                // Get pointer to the static buffer from the subject
                void* dest_buf = (void*)lv_subject_get_pointer(item.subject);
                if (dest_buf != NULL && item.payload_len > 0) {
                    const void* src;
                    if (item.flags & 0x01) {
                        src = item.payload.heap_ptr;
                    } else {
                        src = item.payload.inline_bytes;
                    }
                    memcpy(dest_buf, src, item.payload_len);
                }
                // Notify observers that data changed
                lv_subject_notify(item.subject);
                break;
            }

            case SUBJECT_UPDATE_NOTIFY:
                lv_subject_notify(item.subject);
                break;
        }

        // Free heap payload if used
        if ((item.flags & 0x01) && item.payload.heap_ptr) {
            heap_caps_free(item.payload.heap_ptr);
        }

        processed++;
    }

    // Log if queue still has items (will be processed next cycle)
    if (processed >= MAX_PER_DRAIN && uxQueueMessagesWaiting(s_update_queue) > 0) {
        ESP_LOGD(TAG, "Subject update batch limit, %d pending",
                 (int)uxQueueMessagesWaiting(s_update_queue));
    }

    return processed;
}

int radio_subject_pending_count(void)
{
    if (s_update_queue == NULL) {
        return 0;
    }
    return (int)uxQueueMessagesWaiting(s_update_queue);
}
