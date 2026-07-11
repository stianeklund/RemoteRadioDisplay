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
#include "esp_timer.h"
#include <atomic>
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
    int64_t queued_at_us;
} subject_update_item_t;

// Queue handle
static QueueHandle_t s_update_queue = NULL;
static bool s_initialized = false;
static std::atomic<uint32_t> s_high_watermark{0};
static std::atomic<uint32_t> s_dropped{0};
static std::atomic<uint32_t> s_processed{0};
static std::atomic<uint64_t> s_total_age_us{0};
static std::atomic<uint64_t> s_max_age_us{0};

static void record_queue_success(void)
{
    const uint32_t depth = static_cast<uint32_t>(uxQueueMessagesWaiting(s_update_queue));
    uint32_t previous = s_high_watermark.load(std::memory_order_relaxed);
    while (depth > previous &&
           !s_high_watermark.compare_exchange_weak(previous, depth, std::memory_order_relaxed)) {}
}

static void record_item_age(int64_t queued_at_us)
{
    if (queued_at_us <= 0) return;
    const int64_t age = esp_timer_get_time() - queued_at_us;
    if (age < 0) return;
    const uint64_t age_us = static_cast<uint64_t>(age);
    s_total_age_us.fetch_add(age_us, std::memory_order_relaxed);
    s_processed.fetch_add(1, std::memory_order_relaxed);
    uint64_t previous = s_max_age_us.load(std::memory_order_relaxed);
    while (age_us > previous &&
           !s_max_age_us.compare_exchange_weak(previous, age_us, std::memory_order_relaxed)) {}
}

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
    item.queued_at_us = esp_timer_get_time();

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        s_dropped.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGD(TAG, "Subject update queue full (INT)");
        return false;
    }
    record_queue_success();
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
    item.queued_at_us = esp_timer_get_time();

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        s_dropped.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGD(TAG, "Subject update queue full (FLOAT)");
        return false;
    }
    record_queue_success();
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
    item.queued_at_us = esp_timer_get_time();

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
        s_dropped.fetch_add(1, std::memory_order_relaxed);
        // Free heap payload if queue is full
        if (item.flags & 0x01) {
            heap_caps_free(item.payload.heap_ptr);
        }
        ESP_LOGD(TAG, "Subject update queue full (POINTER)");
        return false;
    }
    record_queue_success();
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
    item.queued_at_us = esp_timer_get_time();

    if (xQueueSendToBack(s_update_queue, &item, 0) != pdPASS) {
        s_dropped.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGD(TAG, "Subject update queue full (NOTIFY)");
        return false;
    }
    record_queue_success();
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

        record_item_age(item.queued_at_us);

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

radio_subject_queue_stats_t radio_subject_reset_queue_stats(void)
{
    radio_subject_queue_stats_t result = {};
    result.high_watermark = s_high_watermark.exchange(0, std::memory_order_relaxed);
    result.dropped = s_dropped.exchange(0, std::memory_order_relaxed);
    result.processed = s_processed.exchange(0, std::memory_order_relaxed);
    const uint64_t total_age = s_total_age_us.exchange(0, std::memory_order_relaxed);
    result.max_age_us = s_max_age_us.exchange(0, std::memory_order_relaxed);
    result.avg_age_us = result.processed > 0 ? total_age / result.processed : 0;
    result.current_depth = s_update_queue != NULL
        ? static_cast<uint32_t>(uxQueueMessagesWaiting(s_update_queue)) : 0;
    return result;
}
