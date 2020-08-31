#ifndef MODEM_H
#define MODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define MODEM_PB_DONE_STR "\r\nPB DONE\r\n"
#define MODEM_OK_STR "\r\nOK\r\n"
#define MODEM_TIMEOUT_MS_INFINITY 0xffff
#define MODEM_USE_MAX_AVAILABLE_AT_BUFF 0

#define MODEM_MASK_ANY_DIGIT '-'

typedef enum modem_err {
  ME_SUCCESS = 0,
  ME_CMD_ECHO_ERR,
  ME_UNEXPECTED_ANSWER,
  ME_TIMEOUT,
  ME_NOT_IMPLEMENTED,
} modem_err_t;

typedef struct modem modem_t;

typedef struct modem_expected_answer {
  const char *prefix; //do we really need this?
  const char *str_exp_res;
  uint16_t max_len;
  // uint8_t _padding[2]; maybe compiller is smart enough to add padding.
} modem_expected_answer_t;

modem_expected_answer_t
modem_expected_answer(const char *prefix,
                      const char *str_exp_res,
                      uint16_t max_len);

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

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

// ... -> modem_expected_answer_t*[]
modem_err_t modem_exec_at_cmd(modem_t *m,
                              const char *cmd,
                              uint16_t timeout_ms,
                              int exp_ans_count, ...);

const char *modem_at_buff(const modem_t *m);

#ifdef __cplusplus
}
#endif // extern "C"
#endif // MODEM_H
