#include <stddef.h>
#include <string.h>
#include <stdarg.h>

#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "modem.h"
#include "commons.h"

#define MODEM_BR_4000000 4000000
#define MODEM_BR_115200 115200

#define MODEM_USART USART1
#define MODEM_GPIO_AF_USART GPIO_AF_USART1

#define MODEM_RCC_PERIPH_GPIO RCC_AHBPeriph_GPIOA
#define MODEM_RCC_PeriphClockCmd RCC_APB2PeriphClockCmd
#define MODEM_RCC_PERIPH_BLOCK RCC_APB2Periph_USART1

#define MODEM_USART_PORT GPIOA

//USART1:
//rts --> PA12
//cts --> PA11
//rx --> PA10
//tx --> PA9
#define MODEM_USART_RTS_PIN GPIO_Pin_12
#define MODEM_USART_CTS_PIN GPIO_Pin_11
#define MODEM_USART_RX_PIN GPIO_Pin_10
#define MODEM_USART_TX_PIN GPIO_Pin_9

#define MODEM_USART_RTS_PIN_SOURCE GPIO_PinSource12
#define MODEM_USART_CTS_PIN_SOURCE GPIO_PinSource11
#define MODEM_USART_RX_PIN_SOURCE GPIO_PinSource10
#define MODEM_USART_TX_PIN_SOURCE GPIO_PinSource9

#define MODEM_DTR_CHANGE_WAIT_MS 20
#define MODEM_TIMEOUT_MS_INFINITY 0xffff
#define MODEM_AT_CMD_BUFF_LEN 64

// TODO CHANGE THIS!
#define MODEM_DTR_PIN GPIO_Pin_8
#define MODEM_PWR_ON_PIN GPIO_Pin_7

///////////////////////////////////////////////////////

#define MODEM_PB_DONE_STR "\r\nPB DONE\r\n"
#define MODEM_OK_STR "\r\nOK\r\n"

typedef struct modem {
  pf_read_byte fn_read_byte;
  pf_write_byte fn_write_byte;

  pf_delay_ms fn_delay_ms;
  pf_get_current_ms fn_get_current_ms;

  char at_cmd_buff[MODEM_AT_CMD_BUFF_LEN];
} modem_t;

typedef struct modem_parse_cmd_res {
  modem_err_t code;
  char *str_answer;
} modem_parse_cmd_res_t;

typedef struct modem_expected_answer {
  const char *prefix; //do we really need this?
  const char *str_exp_res;
  uint16_t max_len;
  // uint8_t _padding[2]; maybe compiller is smart enough to add padding.
} modem_expected_answer_t;

static modem_expected_answer_t
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
static modem_err_t modem_write_byte(modem_t *m,
                                    uint16_t timeout_ms,
                                    char b);

static modem_err_t modem_read_byte(modem_t *m,
                                   uint16_t timeout_ms,
                                   uint8_t *out_byte);

static void modem_set_DTR(const modem_t *m,
                          bool high);

static void modem_init_GPIO_and_USART(void);

static void modem_USART_change_baud_rate(uint32_t br);

static modem_err_t modem_wait_for_pb_ready(modem_t *modem,
                                           uint32_t timeout_ms);

static uint32_t modem_read_at_str(modem_t *m,
                                  uint16_t max_len,
                                  uint16_t timeout_ms);

static modem_err_t modem_exec_at_cmd(modem_t *m,
                                     const char *cmd,
                                     uint16_t timeout_ms,
                                     int exp_ans_count, ...);

static modem_err_t modem_write_str(modem_t *m,
                                   const char *str,
                                   uint16_t timeout_ms);

static modem_parse_cmd_res_t modem_parse_cmd_answer(const char *buff,
                                                    const char *cmd);

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

modem_t*
modem_create_default(void) {
  modem_t *res = modem_create(modem_read_byte,
                              modem_write_byte,
                              delay_ms,
                              get_tick);
  modem_init_GPIO_and_USART();
  return res;
}
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

modem_err_t
modem_set_pwr(modem_t *modem,
              bool on) {
  UNUSED(modem); //WARNING!!! USE THIS!!!
  if (on) {
    GPIO_ResetBits(MODEM_USART_PORT, MODEM_PWR_ON_PIN);
  } else {
    GPIO_SetBits(MODEM_USART_PORT, MODEM_PWR_ON_PIN);
  }
  return ME_NOT_IMPLEMENTED;
}
///////////////////////////////////////////////////////

void
modem_set_DTR(const modem_t *m,
              bool high) {
  if (high) {
    GPIO_SetBits(MODEM_USART_PORT, MODEM_DTR_PIN);
  } else {
    GPIO_ResetBits(MODEM_USART_PORT, MODEM_DTR_PIN);
    m->fn_delay_ms(MODEM_DTR_CHANGE_WAIT_MS);
  }
}
///////////////////////////////////////////////////////

void
modem_USART_change_baud_rate(uint32_t br) {
  USART_InitTypeDef ucfg;
  USART_DeInit(MODEM_USART); //set default values to all registers
  //then init
  ucfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  ucfg.USART_Parity = USART_Parity_No;
  ucfg.USART_BaudRate = br;
  ucfg.USART_StopBits = 1;
  ucfg.USART_WordLength = USART_WordLength_8b;
  ucfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
  USART_OverSampling8Cmd(MODEM_USART, ENABLE);
  USART_Init(MODEM_USART, &ucfg);
  USART_Cmd(MODEM_USART, ENABLE); //turn on modem usart
}
///////////////////////////////////////////////////////

void
modem_init_GPIO_and_USART(void) {
  RCC_AHBPeriphClockCmd(MODEM_RCC_PERIPH_GPIO, ENABLE);
  GPIO_InitTypeDef ioCfg;
  ioCfg.GPIO_Pin = MODEM_USART_RTS_PIN | MODEM_USART_TX_PIN; //rts and tx pull up, AF
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_PP;
  ioCfg.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MODEM_USART_PORT, &ioCfg);

  ioCfg.GPIO_Pin = MODEM_USART_RX_PIN; //rx to floating, AF
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_OD;
  ioCfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MODEM_USART_PORT, &ioCfg);

  ioCfg.GPIO_Pin = MODEM_USART_CTS_PIN; //cts pull down, AF
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_OD;
  ioCfg.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(MODEM_USART_PORT, &ioCfg);

  //set AF
  GPIO_PinAFConfig(MODEM_USART_PORT, MODEM_USART_RTS_PIN_SOURCE, MODEM_GPIO_AF_USART);
  GPIO_PinAFConfig(MODEM_USART_PORT, MODEM_USART_CTS_PIN_SOURCE, MODEM_GPIO_AF_USART);
  GPIO_PinAFConfig(MODEM_USART_PORT, MODEM_USART_RX_PIN_SOURCE, MODEM_GPIO_AF_USART);
  GPIO_PinAFConfig(MODEM_USART_PORT, MODEM_USART_TX_PIN_SOURCE, MODEM_GPIO_AF_USART);

  //  config usart
  MODEM_RCC_PeriphClockCmd(MODEM_RCC_PERIPH_BLOCK, ENABLE);
  modem_USART_change_baud_rate(MODEM_BR_115200); // init baud rate is 115200
}
///////////////////////////////////////////////////////

modem_err_t
modem_read_byte(modem_t *m,
                uint16_t timeout_ms,
                uint8_t *out_byte) {
  uint32_t start = m->fn_get_current_ms();
  volatile bool is_time_out = false;
  while (!(MODEM_USART->SR & USART_SR_RXNE) && !is_time_out) {
    is_time_out = m->fn_get_current_ms() - start > timeout_ms;
  }

  if (is_time_out)
    return ME_TIMEOUT;

  *out_byte = (uint8_t) (MODEM_USART->DR & 0x00ff);
  return ME_SUCCESS;
}
///////////////////////////////////////////////////////

modem_err_t
modem_write_byte(modem_t *m,
                 uint16_t timeout_ms,
                 char b) {
  UNUSED(m);
  UNUSED(timeout_ms); //todo USE THIS!!!!!!!
  while(!(MODEM_USART->SR & USART_SR_TXE))
    ;
  MODEM_USART->DR = b;
  return ME_SUCCESS;
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
//but AT+CIPSEND has another response first: \r\n>\0
//so when this command is in use - need to set buff_size=3
uint32_t
modem_read_at_str(modem_t *m,
                  uint16_t max_len,
                  uint16_t timeout_ms) {
  char ch, *tb;
  bool cr = false;
  modem_err_t err;

  tb = m->at_cmd_buff;
  while (max_len--) {
    err = m->fn_read_byte(m, timeout_ms, (uint8_t*) &ch);
    if (err == ME_TIMEOUT)
      continue; // WARNING! Maybe it's better to return timeout here or break
    *tb++ = ch;

    if (ch != '\n')
      continue;
    if (cr)
      break;
    cr = true;
  }
  *tb++ = 0;
  return (uint32_t)((ptrdiff_t)(tb - m->at_cmd_buff));
}
///////////////////////////////////////////////////////

modem_err_t
modem_write_str(modem_t *m,
                const char *str,
                uint16_t timeout_ms) {
  modem_err_t err = ME_SUCCESS;
  while (*str) {
    err = m->fn_write_byte(m, timeout_ms, *str++);
    if (err != ME_SUCCESS)
      break;
  }
  return err;
}
///////////////////////////////////////////////////////

modem_err_t
modem_wait_for_pb_ready(modem_t *modem,
                        uint32_t timeout_ms) {
  do {
    int rr = modem_read_at_str(modem,
                               sizeof(modem->at_cmd_buff),
                               timeout_ms);
    UNUSED(rr);
  } while (strcmp(MODEM_PB_DONE_STR, modem->at_cmd_buff));
  return ME_SUCCESS;
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
  modem_expected_answer_t *ea;

  err = modem_write_str(m, cmd, 1000); //todo change this timeout to something else
  if (err != ME_SUCCESS)
    return err;

  va_start(va_lst, exp_ans_count);

  for (int i = 0; i < exp_ans_count; ++i) {
    ea = va_arg(va_lst, modem_expected_answer_t*);
    err = modem_read_at_str(m,
                            ea->max_len,
                            timeout_ms);
    if (err != ME_SUCCESS)
      break;

    mpr = modem_parse_cmd_answer(m->at_cmd_buff,
                                 ea->prefix);

    if ((err = mpr.code) != ME_SUCCESS)
      break;

    if (strcmp(mpr.str_answer, ea->str_exp_res)) {
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

modem_err_t
modem_prepare_to_work(modem_t *m) {
  modem_err_t err;
  uint32_t start_ms;

  modem_set_DTR(m, false);
  m->fn_delay_ms(MODEM_DTR_CHANGE_WAIT_MS);

  err = modem_wait_for_pb_ready(m, 10000);
  if (err != ME_SUCCESS)
    return err;

  do {
    // enable RTS/CTS
    err = modem_exec_at_cmd(m, "AT+IFC=2,2\r", MODEM_TIMEOUT_MS_INFINITY,
                            1, modem_expected_answer("AT+IFC=2,2\r", MODEM_OK_STR, sizeof (m->at_cmd_buff)));
    if (err != ME_SUCCESS)
      break;

    // enable 7-line usart mode
    err = modem_exec_at_cmd(m, "AT+CSUART=1\r", MODEM_TIMEOUT_MS_INFINITY,
                            1, modem_expected_answer("AT+CSUART=1\r", MODEM_OK_STR, sizeof (m->at_cmd_buff)));

    if (err != ME_SUCCESS)
      break;

    // set DTR pin mode (see specification)
    err = modem_exec_at_cmd(m, "AT&D1\r", MODEM_TIMEOUT_MS_INFINITY,
                            1, modem_expected_answer("AT&D1\r", MODEM_OK_STR, sizeof (m->at_cmd_buff)));

    if (err != ME_SUCCESS)
      break;

    // set baud rate to 4 000 000
    err = modem_exec_at_cmd(m, "AT+IPR=4000000\r", MODEM_TIMEOUT_MS_INFINITY,
                            1, modem_expected_answer("AT+IPR=4000000\r", MODEM_OK_STR, sizeof (m->at_cmd_buff)));

    if (err != ME_SUCCESS)
      break;

    modem_USART_change_baud_rate(MODEM_BR_4000000);

    start_ms = m->fn_get_current_ms();
    for (; m->fn_get_current_ms() - start_ms > 200; ) {
      err = modem_exec_at_cmd(m, "AT\r", 20,
                              1, modem_expected_answer("AT\r", MODEM_OK_STR, sizeof (m->at_cmd_buff)));
      if (err == ME_SUCCESS)
        break;
    } // for. we could get "OK" from modem on 4000000 baud rate

  } while (false);
  return err;
}
