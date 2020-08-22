#ifndef COMMONS_H
#define COMMONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define UNUSED(x) ((void)x)

char* u16_to_str(uint16_t val);
void delay_ms(uint32_t ms);
uint32_t get_tick(void);

#ifdef __cplusplus
}
#endif // extern "C"
#endif // COMMONS_H
