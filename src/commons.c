#include "commons.h"

char *
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
