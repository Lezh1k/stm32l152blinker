#ifndef MODEM_H
#define MODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

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
  const char *prefix;
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

#define MODEM_AT_CMD_BUFF_LEN 64
typedef struct modem {
  pf_read_byte fn_read_byte;
  pf_write_byte fn_write_byte;

  pf_delay_ms fn_delay_ms;
  pf_get_current_ms fn_get_current_ms;

  char at_cmd_buff[MODEM_AT_CMD_BUFF_LEN];
} modem_t;

// this one is to create modem handler struct

modem_t* modem_create(pf_read_byte fn_read_byte,
                      pf_write_byte fn_write_byte,
                      pf_delay_ms fn_delay_ms,
                      pf_get_current_ms fn_get_current_ms);

// ... -> modem_expected_answer_t*[]
modem_err_t modem_exec_at_cmd(modem_t *m,
                              const char *cmd,
                              uint16_t timeout_ms,
                              int exp_ans_count, ...);

// used in HW. todo move to vptr
modem_err_t modem_read_at_str(modem_t *m,
                           uint16_t max_len,
                           uint16_t timeout_ms, uint32_t *read_n);


#ifdef MODEM_SPEED_TEST
void modem_speed_test(void);
#endif

#ifdef __cplusplus
}
#endif // extern "C"
#endif // MODEM_H
