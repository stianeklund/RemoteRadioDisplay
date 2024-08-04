#ifndef CAT_PARSER_H
#define CAT_PARSER_H 

#include "esp_err.h"
#include <stdint.h>

esp_err_t parse_command(void);
uint32_t parse_frequency(const char* frequency);
uint8_t* read_uart(void);
void parse_uart();

#endif // CAT_PARSER_H 

