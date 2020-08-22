#include <stddef.h>
#include <string.h>

#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "modem.h"
#include "commons.h"

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

// private functions
static void modem_write_byte(char b);
static uint8_t modem_read_byte(uint16_t timeout_ms);

static void modem_init_GPIO_and_USART(void);
static void modem_USART_change_baud_rate(uint32_t br);


static void modem_set_DTR(const modem_t *m, bool high);


static uint32_t modem_read_at_str_timeout(modem_t *m,
                                          uint16_t max_len,
                                          uint32_t timeout_ms);

static uint32_t modem_read_at_str(modem_t *m,
                                  uint16_t max_len);

// modem instance
static modem_t m_instance;

modem_t*
modem_create_default(char *data_buff,
                     uint16_t data_buff_len) {
  m_instance.data_buff = data_buff;
  m_instance.data_buff_len = data_buff_len;
  m_instance.fn_read_byte = modem_read_byte;
  m_instance.fn_write_byte = modem_write_byte;
  modem_init_GPIO_and_USART();
  return &m_instance;
}
///////////////////////////////////////////////////////

modem_t*
modem_create(pf_read_byte fn_read_byte,
             pf_write_byte fn_write_byte,
             char *data_buff,
             uint16_t data_buff_len) {
  m_instance.data_buff = data_buff;
  m_instance.data_buff_len = data_buff_len;
  m_instance.fn_read_byte = fn_read_byte;
  m_instance.fn_write_byte = fn_write_byte;
  return &m_instance;
}
///////////////////////////////////////////////////////

modem_err_t
modem_cmd(modem_t *modem,
          const char *cmd,
          const char **expected_answers) {
  return ME_NOT_IMPLEMENTED;
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

uint8_t
modem_read_byte(uint16_t timeout_ms) {
  if (timeout_ms == 0) { //wait infinity
    while (!(MODEM_USART->SR & USART_SR_RXNE))
      ;
    return (uint8_t) (MODEM_USART->DR & 0x00ff);
  }
  uint32_t start = get_tick();
  volatile bool is_time_out = false;

  while (!(MODEM_USART->SR & USART_SR_RXNE) && !is_time_out) {
    is_time_out = get_tick() - start > timeout_ms;
  }
  return is_time_out ? 0 : (uint8_t) (MODEM_USART->DR & 0x00ff);
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
//but AT+CIPSEND has another response first: \r\n>\0
//so when this command is in use - need to set buff_size=3
uint32_t
modem_read_at_str_timeout(modem_t *m,
                          uint16_t max_len,
                          uint32_t timeout_ms) {
  char ch, *tb;
  bool cr = false;

  tb = m->at_cmd_buff;
  while (max_len--) {
    ch = m->fn_read_byte(timeout_ms);
    if (!ch)
      continue;
    *tb++ = ch;
    if (ch != '\n')
      continue;
    if (cr)
      break;
    cr = true;
  }

  *tb++ = 0;
  return (uint32_t)((ptrdiff_t)(tb-m->at_cmd_buff));
}
///////////////////////////////////////////////////////

uint32_t
modem_read_at_str(modem_t *m, uint16_t max_len) {
  return modem_read_at_str_timeout(m, max_len, 0);
}
///////////////////////////////////////////////////////

void
modem_write_byte(char b) {
  while(!(MODEM_USART->SR & USART_SR_TXE))
    ;
  MODEM_USART->DR = b;
}
///////////////////////////////////////////////////////

void
modem_write_cmd(const char *cmd) {
  for (; *cmd; ++cmd)
    modem_write_byte(*cmd);
}
///////////////////////////////////////////////////////

void
modem_write_data(const char *buff,
                 uint32_t size) {
  while (size--)
    modem_write_byte(*buff++);
}
///////////////////////////////////////////////////////

void
modem_set_pwr(bool on) {
  if (on) {
    GPIO_ResetBits(MODEM_USART_PORT, MODEM_PWR_ON_PIN);
  } else {
    GPIO_SetBits(MODEM_USART_PORT, MODEM_PWR_ON_PIN);
  }
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
    int rr = modem_read_at_str_timeout(modem,
                                       sizeof(modem->at_cmd_buff),
                                       timeout_ms);
    //todo handle rr. maybe handle some timeout.
  } while (strcmp(MODEM_PB_DONE_STR, modem->at_cmd_buff));
  return ME_SUCCESS;
}

modem_err_t
modem_set_pwr(modem_t *modem, bool on) {
  return ME_NOT_IMPLEMENTED;
}
