#include <stddef.h>
#include <string.h>
#include <stdarg.h>

#include "modem.h"
#include "commons.h"

typedef struct modem_parse_cmd_res {
  modem_err_t code;
  char *str_answer;
} modem_parse_cmd_res_t;

modem_expected_answer_t
modem_expected_answer(const char *prefix,
                      const char *str_exp_res,
                      uint16_t max_len) {
  modem_expected_answer_t res;
  res.prefix = prefix;
  res.str_exp_res = str_exp_res;
  res.max_len = max_len;
  return res;
}
///////////////////////////////////////////////////////

// private functions

static modem_parse_cmd_res_t modem_parse_cmd_answer(const char *buff,
                                                    const char *cmd);

static modem_err_t modem_write_at_str(modem_t *m,
                                      const char *str,
                                      uint16_t timeout_ms);

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


modem_t*
modem_create(pf_read_byte fn_read_byte,
             pf_write_byte fn_write_byte,
             pf_delay_ms fn_delay_ms,
             pf_get_current_ms fn_get_current_ms) {
  static modem_t m_instance;
  m_instance.fn_read_byte = fn_read_byte;
  m_instance.fn_write_byte = fn_write_byte;
  m_instance.fn_delay_ms = fn_delay_ms;
  m_instance.fn_get_current_ms = fn_get_current_ms;
  return &m_instance;
}
///////////////////////////////////////////////////////


//modem response has this format : <cr><lf>response<cr><lf>
//but AT+CIPSEND has another response first: \r\n>\0
//so when this command is in use - need to set buff_size=3
modem_err_t
modem_read_at_str(modem_t *m,
                  uint16_t max_len,
                  uint16_t timeout_ms,
                  uint32_t *read_n) {
  char ch, *tb;
  bool cr = false;
  modem_err_t err;
  uint32_t start_ms = m->fn_get_current_ms();
  uint32_t current_ms;

  if (max_len == MODEM_USE_MAX_AVAILABLE_AT_BUFF)
    max_len = sizeof (m->at_cmd_buff);

  tb = m->at_cmd_buff;

  while (max_len &&
         ((current_ms = m->fn_get_current_ms()) - start_ms) <= timeout_ms) {
    err = m->fn_read_byte(m, 50, (uint8_t*) &ch);
    if (err == ME_TIMEOUT)
      continue;

    --max_len;
    *tb++ = ch;
    if (ch != '\n')
      continue;

    if (cr)
      break;

    cr = true;
  }
  *tb++ = 0;
  *read_n = (uint32_t)((ptrdiff_t)(tb - m->at_cmd_buff));
  return err;
}
///////////////////////////////////////////////////////

modem_err_t
modem_write_at_str(modem_t *m,
                   const char *str,
                   uint16_t timeout_ms) {
  modem_err_t err = ME_SUCCESS;
  uint32_t start_ms = m->fn_get_current_ms();
  uint32_t current_ms;

  while (*str &&
         ((current_ms = m->fn_get_current_ms()) - start_ms) <= timeout_ms) {
    err = m->fn_write_byte(m, timeout_ms, *str);
    if (err == ME_TIMEOUT)
      continue;
    if (err != ME_SUCCESS)
      break;
    ++str;
  }
  return err;
}
///////////////////////////////////////////////////////

modem_parse_cmd_res_t
modem_parse_cmd_answer(const char *buff,
                       const char *cmd) {
  modem_parse_cmd_res_t res;
  for (; *cmd; ++cmd, ++buff) {
    if (*cmd == *buff)
      continue;
    res.code = ME_CMD_ECHO_ERR;
    return res;
  }
  res.code = ME_SUCCESS;
  res.str_answer = (char*) buff; //cause we won't change anything.
  return res;
}
///////////////////////////////////////////////////////

modem_err_t
modem_exec_at_cmd(modem_t *m,
                  const char *cmd,
                  uint16_t timeout_ms,
                  int exp_ans_count, ...) {
  va_list va_lst;
  modem_err_t err = ME_SUCCESS;
  modem_parse_cmd_res_t mpr;
  modem_expected_answer_t ea;
  uint32_t read_n;

  err = modem_write_at_str(m, cmd, 1000);
  if (err != ME_SUCCESS)
    return err;

  va_start(va_lst, exp_ans_count);
  for (int i = 0; i < exp_ans_count; ++i) {
    ea = va_arg(va_lst, modem_expected_answer_t);
    err = modem_read_at_str(m,
                            ea.max_len,
                            timeout_ms,
                            &read_n);
    if (err != ME_SUCCESS)
      break;

    mpr = modem_parse_cmd_answer(m->at_cmd_buff,
                                 ea.prefix);

    if ((err = mpr.code) != ME_SUCCESS)
      break;

    const char *sa, *se;
    sa = mpr.str_answer;
    se = ea.str_exp_res;

    for (; *sa && *se; ) {
      if (*sa == *se) {
        ++sa; ++se;
        continue;
      }

      //check - as "\d+". not as just digit
      if (*sa >= '0' && *sa <= '9' && *se == MODEM_MASK_ANY_DIGIT) {
        ++sa; ++se;
        continue;
      }

      // if we received not digit, but we are still waiting
      // for digit - just increment expected ptr
      if (*se == MODEM_MASK_ANY_DIGIT) {
        ++se;
        continue;
      }

      err = ME_UNEXPECTED_ANSWER;
      break;
    }

    //great. we here if command passed without errors
    //and returned expected result
  }

  va_end(va_lst);
  return err;
}
///////////////////////////////////////////////////////

#ifdef MODEM_SPEED_TEST
void
modem_speed_test() {
  modem_t *m_modem;
  modem_err_t m_err;

  modem_socket_t m_sock;
  ms_error_t ms_err;

  SysTick_Config(SystemCoreClock / 1000); //1 ms tick. see commons.c
  LED_init();
  m_modem = modem_create_default();

  led_green_turn(true);
  m_err = modem_prepare_to_work(m_modem);
  if (m_err != ME_SUCCESS) {
    led_green_turn(false);
  }

  m_sock = ms_socket(m_modem);
  do {
    ms_err = ms_set_timeouts(&m_sock, 8000, 3000, 3000);
    if (ms_err != MSE_SUCCESS) {
      led_green_turn(false);
      break;
    }

    ms_err = ms_net_open(&m_sock);
    if (ms_err != MSE_SUCCESS) {
      led_green_turn(false);
      break;
    }

    // speed test
    for (int i = 0; i < 10; ++i) {
      ms_err = ms_tcp_connect(&m_sock, "212.42.115.163", 45223);
      if (ms_err != MSE_SUCCESS)
        break;

      for (int j = 0; j < 1024; ++j) {
        int sent = ms_send(&m_sock, (uint8_t*)"123456789012345678901234567890", 30);
        if (sent != 30) {
          ms_err = MSE_NETWORK_ERR_BASE;
          break;
        }
      }
      if (ms_err != MSE_SUCCESS)
        break;

      ms_err = ms_tcp_disconnect(&m_sock);
      delay_ms(2000);
    }

    if (ms_err != MSE_SUCCESS)
      break;

    ms_err = ms_net_close(&m_sock);
  } while (0);

}
#endif
