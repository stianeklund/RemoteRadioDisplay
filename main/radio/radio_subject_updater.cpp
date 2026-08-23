/**
 * @file radio_subject_updater.cpp
 * @brief Thread-safe subject update implementation using FreeRTOS queue
 *
 * Updates from non-LVGL tasks are queued and processed during
 * radio_subject_drain_updates() called from the LVGL task.
 */

#include "radio_subject_updater.h"
#include "cat_shared_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <atomic>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "RADIO_SUBJECT_UPD";

// Maximum size for inline payload (larger payloads use heap).
// Sized so kenwood_if_data_t (56 B) stays inline: IF frames are queued at ~6.7 Hz
// while polling, and a PSRAM malloc/free per frame is a pointless fragmentation
// source. Cost of the bump is 16 B per slot, 1 KB over the 64-slot queue.
#define INLINE_PAYLOAD_SIZE 64

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

static_assert(sizeof(kenwood_if_data_t) <= INLINE_PAYLOAD_SIZE,
              "kenwood_if_data_t must stay inline: it is queued per IF frame and the "
              "heap path would malloc/free PSRAM at the poll rate");

// Queue handle
static QueueHandle_t s_update_queue = NULL;
static bool s_initialized = false;
static std::atomic<uint32_t> s_high_watermark{0};
static std::atomic<uint32_t> s_dropped{0};
static std::atomic<uint32_t> s_processed{0};
static std::atomic<uint64_t> s_total_age_us{0};
static std::atomic<uint64_t> s_max_age_us{0};

// Post-dequeue instrumentation. All of these are touched from the LVGL task only
// (drain timer plus display render events); atomics keep them consistent with the
// queue counters above and make the stats reset safe from any caller.
static std::atomic<uint64_t> s_apply_total_us{0};
static std::atomic<uint64_t> s_apply_max_us{0};
static std::atomic<uint32_t> s_apply_samples{0};
static std::atomic<uint64_t> s_render_total_us{0};
static std::atomic<uint64_t> s_render_max_us{0};
static std::atomic<uint32_t> s_render_samples{0};
static std::atomic<uint64_t> s_e2e_total_us{0};
static std::atomic<uint64_t> s_e2e_max_us{0};
static std::atomic<uint32_t> s_e2e_samples{0};

// Oldest enqueue timestamp applied to a subject but not yet carried through a
// refresh. Zero means nothing is waiting on the renderer.
static std::atomic<int64_t> s_pending_refresh_stamp_us{0};
// Open render sample, LVGL task only.
static int64_t s_render_start_us = 0;

static void record_queue_success(void)
{
    const uint32_t depth = static_cast<uint32_t>(uxQueueMessagesWaiting(s_update_queue));
    uint32_t previous = s_high_watermark.load(std::memory_order_relaxed);
    while (depth > previous &&
           !s_high_watermark.compare_exchange_weak(previous, depth, std::memory_order_relaxed)) {}
}

static void accumulate_sample(std::atomic<uint64_t>& total, std::atomic<uint64_t>& max,
                              std::atomic<uint32_t>& samples, uint64_t value_us)
{
    total.fetch_add(value_us, std::memory_order_relaxed);
    samples.fetch_add(1, std::memory_order_relaxed);
    uint64_t previous = max.load(std::memory_order_relaxed);
    while (value_us > previous &&
           !max.compare_exchange_weak(previous, value_us, std::memory_order_relaxed)) {}
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
    const int64_t apply_start_us = esp_timer_get_time();
    int64_t oldest_queued_us = 0;

    while (processed < MAX_PER_DRAIN &&
           xQueueReceive(s_update_queue, &item, 0) == pdTRUE) {

        record_item_age(item.queued_at_us);
        if (item.queued_at_us > 0 &&
            (oldest_queued_us == 0 || item.queued_at_us < oldest_queued_us)) {
            oldest_queued_us = item.queued_at_us;
        }

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

    if (processed > 0) {
        const int64_t apply_us = esp_timer_get_time() - apply_start_us;
        if (apply_us >= 0) {
            accumulate_sample(s_apply_total_us, s_apply_max_us, s_apply_samples,
                              static_cast<uint64_t>(apply_us));
        }
        // Keep the oldest outstanding stamp so the refresh hook reports worst-case
        // latency rather than the latest update that happened to sneak in.
        if (oldest_queued_us > 0) {
            int64_t pending = s_pending_refresh_stamp_us.load(std::memory_order_relaxed);
            while ((pending == 0 || oldest_queued_us < pending) &&
                   !s_pending_refresh_stamp_us.compare_exchange_weak(
                       pending, oldest_queued_us, std::memory_order_relaxed)) {}
        }
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

    const uint64_t apply_total = s_apply_total_us.exchange(0, std::memory_order_relaxed);
    result.apply_samples = s_apply_samples.exchange(0, std::memory_order_relaxed);
    result.apply_max_us = s_apply_max_us.exchange(0, std::memory_order_relaxed);
    result.apply_avg_us = result.apply_samples > 0 ? apply_total / result.apply_samples : 0;

    const uint64_t render_total = s_render_total_us.exchange(0, std::memory_order_relaxed);
    result.render_samples = s_render_samples.exchange(0, std::memory_order_relaxed);
    result.render_max_us = s_render_max_us.exchange(0, std::memory_order_relaxed);
    result.render_avg_us = result.render_samples > 0 ? render_total / result.render_samples : 0;

    const uint64_t e2e_total = s_e2e_total_us.exchange(0, std::memory_order_relaxed);
    result.e2e_samples = s_e2e_samples.exchange(0, std::memory_order_relaxed);
    result.e2e_max_us = s_e2e_max_us.exchange(0, std::memory_order_relaxed);
    result.e2e_avg_us = result.e2e_samples > 0 ? e2e_total / result.e2e_samples : 0;
    result.current_depth = s_update_queue != NULL
        ? static_cast<uint32_t>(uxQueueMessagesWaiting(s_update_queue)) : 0;
    return result;
}

// ============================================================================
// Render-Stage Instrumentation
// ============================================================================

void radio_subject_mark_render_start(void)
{
    s_render_start_us = esp_timer_get_time();
}

void radio_subject_mark_render_ready(void)
{
    if (s_render_start_us <= 0) {
        return;
    }
    const int64_t render_us = esp_timer_get_time() - s_render_start_us;
    s_render_start_us = 0;
    if (render_us >= 0) {
        accumulate_sample(s_render_total_us, s_render_max_us, s_render_samples,
                          static_cast<uint64_t>(render_us));
    }
}

void radio_subject_mark_refresh_ready(void)
{
    // LV_EVENT_REFR_READY fires on every refresh, including ones that drew nothing.
    // Only refreshes that had a pending subject update produce a sample.
    const int64_t pending = s_pending_refresh_stamp_us.exchange(0, std::memory_order_relaxed);
    if (pending <= 0) {
        return;
    }
    const int64_t e2e_us = esp_timer_get_time() - pending;
    if (e2e_us >= 0) {
        accumulate_sample(s_e2e_total_us, s_e2e_max_us, s_e2e_samples,
                          static_cast<uint64_t>(e2e_us));
    }
}
