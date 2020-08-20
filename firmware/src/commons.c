#include "commons.h"

char*
u16_to_str(uint16_t val) {
  static char buff[6] = {0}; //65535 - max val
  char *bv = &buff[5];
  *bv = 0; //zero end
  do {
    *(--bv) = (val % 10) + '0';
    val /= 10;
  } while (val && bv >= buff);
  return bv;
}
///////////////////////////////////////////////////////

static volatile uint32_t sys_tick_current;
void delay_ms (uint32_t ms) {
  uint32_t curr = sys_tick_current;
  while (sys_tick_current - curr <= ms)
    ; // do nothing
}
///////////////////////////////////////////////////////

void
SysTick_Handler(void) {
  ++sys_tick_current;
}
///////////////////////////////////////////////////////

uint32_t
get_tick() {
  return sys_tick_current;
}
