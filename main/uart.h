#ifndef UART_H 
#define UART_H 

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t high_watermark;
    uint32_t dropped;
    uint32_t dequeued;
    uint32_t current_depth;
    uint64_t max_age_us;
    uint64_t avg_age_us;
} uart_pipeline_queue_stats_t;

esp_err_t init_uart(void);
void read_uart(void *pvParameters);
void cat_parser_task(void *pvParameters);
esp_err_t uart_write_message(const char *message);
void uart_write_message_handler(void *arg, void *data);
void uart_tx_task(void *pvParameters);

// Low-level raw write using the configured CAT UART port
esp_err_t uart_write_raw(const char *data, size_t len);

// Expose selected UART port for diagnostics/tests
int uart_get_port(void);

// Check if UART TX queue is initialized and ready for messages
bool uart_is_ready(void);
uart_pipeline_queue_stats_t uart_reset_tx_queue_stats(void);
uart_pipeline_queue_stats_t uart_reset_parser_queue_stats(void);
#endif // UART_H
