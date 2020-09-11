#include <stm32l1xx_tim.h>
#include "commons.h"

char*
u16_to_str(uint16_t val) {
  static char buff[6] = {0}; //65535 - max val + \0 at the end of string
  char *bv = &buff[5];
  *bv = 0; //zero end
  do {
    *(--bv) = (val % 10) + '0';
    val /= 10;
  } while (val && bv > buff);
  return bv;
}
///////////////////////////////////////////////////////

static volatile uint32_t current_sys_tick;
void delay_ms (uint32_t ms) {
  uint32_t start = current_sys_tick;
  while (current_sys_tick - start <= ms)
    ; // do nothing
}
///////////////////////////////////////////////////////

void
SysTick_Handler(void) {
  ++current_sys_tick; // 1ms tick
}
///////////////////////////////////////////////////////

uint32_t
get_tick() {
  return current_sys_tick;
}
///////////////////////////////////////////////////////

void
delay_init() {
  SysTick_Config(SystemCoreClock / 1000); //1 ms tick
}
///////////////////////////////////////////////////////
