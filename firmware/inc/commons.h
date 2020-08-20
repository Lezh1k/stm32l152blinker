#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

char* u16_to_str(uint16_t val);
void delay_ms(uint32_t ms);
uint32_t get_tick(void);

#endif // UTILS_H
