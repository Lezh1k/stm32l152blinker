#include <stddef.h>
#include <string.h>

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

#define MODEM_DELAY_AFTER_DTR_CHANGE_MS 20

// TODO CHANGE THIS!
#define MODEM_DTR_PIN GPIO_Pin_8
#define MODEM_PWR_ON_PIN GPIO_Pin_7

///////////////////////////////////////////////////////

#define MODEM_PB_DONE_STR "\r\nPB DONE\r\n"
#define MODEM_OK_STR "\r\nOK\r\n"
#define MODEM_ERROR_STR "\r\nERROR\r\n"

typedef struct modem {
  pf_read_byte fn_read_byte;
  pf_write_byte fn_write_byte;

  pf_delay_ms fn_delay_ms;
  pf_get_current_ms fn_get_current_ms;

  char at_cmd_buff[64];
  // at+cipopen=10,"tcp","192.168.129.100",90901\r -> longest at sequence I guess.
  //  uint8_t _padding[?]; maybe compiller is smart enough to add padding.
  //  if not - now it's 80 bytes - aligned enough.
} modem_t;

// todo check if we need this
typedef struct modem_parse_cmd_res {
  modem_err_t code;
  char *str_answer;
} modem_parse_cmd_res_t;

typedef struct modem_expected_answer {
  const char *str;
  uint32_t max_len;
} modem_expected_answer_t;

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
                                  uint32_t timeout_ms);

static uint32_t modem_read_at_str_infinity(modem_t *m,
                                           uint16_t max_len);

static modem_err_t modem_exec_at_cmd(modem_t *m,
                                     modem_expected_answer_t *lst_expected_answers);

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

modem_err_t
modem_read_byte(modem_t *m,
                uint16_t timeout_ms,
                uint8_t *out_byte) {
  if (timeout_ms == 0) { //wait infinity
    while (!(MODEM_USART->SR & USART_SR_RXNE))
      ;
    *out_byte = (uint8_t) (MODEM_USART->DR & 0x00ff);
    return ME_SUCCESS;
  }

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
                  uint32_t timeout_ms) {
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
  *tb = 0; //*tb++
  return (uint32_t)((ptrdiff_t)(tb-m->at_cmd_buff));
}
///////////////////////////////////////////////////////

uint32_t
modem_read_at_str_infinity(modem_t *m,
                           uint16_t max_len) {
  return modem_read_at_str(m, max_len, 0);
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

void
modem_set_DTR(const modem_t *m,
              bool high) {
  if (high) {
    GPIO_SetBits(MODEM_USART_PORT, MODEM_DTR_PIN);
  } else {
    GPIO_ResetBits(MODEM_USART_PORT, MODEM_DTR_PIN);
    m->fn_delay_ms(MODEM_DELAY_AFTER_DTR_CHANGE_MS);
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


modem_err_t modem_exec_at_cmd(modem_t *m,
                                     modem_expected_answer_t *lst_expected_answers) {

}

modem_err_t
modem_prepare_to_work(modem_t *m) {
  modem_err_t err;
  err = modem_wait_for_pb_ready(m, 10000);
  if (err != ME_SUCCESS)
    return err;

  // refactoring!!!!
  modem_write_cmd("AT+IFC=2,2\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+IFC=2,2\r");
  if (mpr.code != ME_SUCCESS) {
    led_blue_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 1.1 7 line uart mode
  modem_write_cmd("AT+CSUART=0\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CSUART=0\r");
  if (mpr.code != ME_SUCCESS) {
    led_blue_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 1.1.1
  modem_write_cmd("AT+CSCLK=1\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CSCLK=1\r");
  if (mpr.code != ME_SUCCESS) {
    led_blue_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 1.2 DTR pin enable
  modem_write_cmd("AT&D1\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT&D1\r");
  if (mpr.code != ME_SUCCESS) {
    led_blue_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 2. Set modem baud/rate temporary
  modem_write_cmd("AT+IPR=4000000\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+IPR=4000000\r");
  if (mpr.code != ME_SUCCESS) {
    led_blue_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  modem_USART_change_baud_rate(MODEM_BR_4000000);
  led_blue_turn(false);

  // 3. Check that we can communicate
  for (;;) { //todo some counter and timeout
    modem_write_cmd("AT\r");
    rr = modem_read_str_timeout(buff, sizeof(buff), 1000);
    mpr = modem_parse_cmd_answer(buff, "AT\r");
    if (mpr.code == ME_SUCCESS &&
        strcmp(mpr.str_answer, MODEM_OK_STR) == 0) {
      break;
    }
  }
}
