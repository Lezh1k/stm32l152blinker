#ifndef COMMONS_H
#define COMMONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define UNUSED(x) ((void)x)

char* u16_to_str(uint16_t val);
uint32_t get_tick(void);

void delay_init(void);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif // extern "C"
#endif // COMMONS_H
