#ifndef UART_H 
#define UART_H 

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

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
#endif // UART_H
