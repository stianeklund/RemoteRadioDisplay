#ifndef UART_H 
#define UART_H 

#include "esp_err.h"
#include <stdint.h>

esp_err_t init_uart(void);
void read_uart(void);
#endif // UART_H
