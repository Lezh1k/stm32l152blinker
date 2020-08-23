#ifndef MODEM_H
#define MODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum modem_err {
  ME_SUCCESS = 0,
  ME_CMD_ECHO_ERR,
  ME_UNEXPECTED_ANSWER,
  ME_TIMEOUT,
  ME_NOT_IMPLEMENTED,
} modem_err_t;


typedef struct modem modem_t;
// pf_read_byte(const modem_t *m, uint16_t timeout_ms, uint8_t *out_res);
typedef modem_err_t (*pf_read_byte)(modem_t*, uint16_t, uint8_t*);
// pf_write_byte(const modem_t *m, uint16_t timeout_ms, char byte);
typedef modem_err_t (*pf_write_byte)(modem_t*, uint16_t, char);
typedef void (*pf_delay_ms)(uint32_t);
typedef uint32_t (*pf_get_current_ms)(void);

// this one is to create modem handler struct
modem_t* modem_create_default(void);
modem_t* modem_create(pf_read_byte fn_read_byte,
                      pf_write_byte fn_write_byte,
                      pf_delay_ms fn_delay_ms,
                      pf_get_current_ms fn_get_current_ms);

modem_err_t modem_set_pwr(modem_t *modem,
                          bool on);

// this function will:
// 1. Wait for "pb done" message
// 2. Set RTS/CTS (AT+IFC=2,2\r)
// 3. Set Usart 7 line mode (AT+CSUART=1\r)
// 4. Set DTR pin functionality (AT&D1)
// 5. Set baud rate temporarily to 4 000 000 baud (AT+IPR=4000000)
modem_err_t modem_prepare_to_work(modem_t *m);

#ifdef __cplusplus
}
#endif // extern "C"
#endif // MODEM_H
