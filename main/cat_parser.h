#ifndef CAT_PARSER_H
#define CAT_PARSER_H 

#include "esp_err.h"
#include <stdint.h>

esp_err_t parse_command(void);
uint32_t parse_frequency(const char* frequency);
void read_uart(void);

#endif // CAT_PARSER_H 

