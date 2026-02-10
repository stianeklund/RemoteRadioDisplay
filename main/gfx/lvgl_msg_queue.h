#ifndef LVGL_MSG_QUEUE_H
#define LVGL_MSG_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the LVGL message queue. Call once during LVGL init (before creating the LVGL task).
void lvgl_msg_queue_init(void);

// Post a message to be executed in the LVGL task context.
// If len <= 64, payload is copied inline. If len > 64 and data != NULL,
// a heap buffer is allocated and freed after dispatch.
// Returns true on success, false if the queue is full or parameters invalid.
bool lvgl_post_message(uint32_t id, const void *data, size_t len);

// Drain all pending messages. Must be called from the LVGL task with LVGL mutex held.
void lvgl_drain_messages(void);

#ifdef __cplusplus
}
#endif

#endif // LVGL_MSG_QUEUE_H

