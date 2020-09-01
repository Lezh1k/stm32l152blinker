#ifndef MODEM_HW_H
#define MODEM_HW_H

#include "modem.h"

modem_err_t modem_hw_write_byte(modem_t *m,
                                uint16_t timeout_ms,
                                char b);

modem_err_t modem_hw_read_byte(modem_t *m,
                               uint16_t timeout_ms,
                               uint8_t *out_byte);

modem_err_t modem_set_pwr(modem_t *modem,
                          bool on);

void modem_set_DTR(const modem_t *m,
                   bool high);

void modem_init_GPIO_and_USART(void);
void modem_USART_change_baud_rate(uint32_t br);


modem_t* modem_create_default(void);

// this function will:
// 1. Wait for "pb done" message
// 2. Set RTS/CTS (AT+IFC=2,2\r)
// 3. Set Usart 7 line mode (AT+CSUART=1\r)
// 4. Set DTR pin functionality (AT&D1)
// 5. Set baud rate temporarily to 4 000 000 baud (AT+IPR=4000000)
modem_err_t modem_prepare_to_work(modem_t *m);

const char *modem_ok_str(void);


#endif // MODEM_HW_H
