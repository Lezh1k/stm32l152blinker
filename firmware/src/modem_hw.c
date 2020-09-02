#include <stm32l1xx.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_usart.h>
#include <stm32l1xx_rcc.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "commons.h"
#include "modem_hw.h"

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

// TODO CHANGE THIS!
#define MODEM_DTR_PIN GPIO_Pin_8
#define MODEM_PWR_ON_PIN GPIO_Pin_7

static modem_err_t modem_wait_for_pb_ready(modem_t *modem,
                                           uint32_t timeout_ms);

modem_t*
modem_create_default(void) {
  modem_t *res = modem_create(modem_hw_read_byte,
                              modem_hw_write_byte,
                              delay_ms,
                              get_tick);
  modem_init_GPIO_and_USART();
  return res;
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
modem_hw_read_byte(modem_t *m,
                   uint16_t timeout_ms,
                   uint8_t *out_byte) {
  uint32_t start_ms = m->fn_get_current_ms();
  uint32_t current_ms;
  volatile bool is_time_out = false;
  while (!(MODEM_USART->SR & USART_SR_RXNE) && !is_time_out) {
    current_ms = m->fn_get_current_ms();
    is_time_out = (current_ms - start_ms) > timeout_ms;
  }

  if (is_time_out)
    return ME_TIMEOUT;

  *out_byte = (uint8_t) (MODEM_USART->DR & 0x00ff);
  return ME_SUCCESS;
}
///////////////////////////////////////////////////////

modem_err_t
modem_hw_write_byte(modem_t *m,
                    uint16_t timeout_ms,
                    char b) {
  uint32_t start_ms = m->fn_get_current_ms();
  uint32_t current_ms;
  volatile bool is_time_out = false;

  while(!(MODEM_USART->SR & USART_SR_TXE) && !is_time_out) {
    current_ms = m->fn_get_current_ms();
    is_time_out = (current_ms - start_ms) > timeout_ms;
  }

  if (is_time_out)
    return ME_TIMEOUT;

  MODEM_USART->DR = b;
  return ME_SUCCESS;
}
///////////////////////////////////////////////////////

modem_err_t
modem_prepare_to_work(modem_t *m) {
  modem_err_t err;
  uint32_t start_ms;
  const char* init_commands[] = {
    "AT+IFC=2,2\r", //set CTS/RTS
//    "AT+CSUART=1\r", //set 7-line uart mode true
//    "AT&D1\r", //set DTR mode false
    "AT+IPR=4000000\r", //set modem baud rate
    NULL,
  };

  modem_set_DTR(m, false);
  m->fn_delay_ms(MODEM_DTR_CHANGE_WAIT_MS);

  err = modem_wait_for_pb_ready(m, 30000);
  if (err != ME_SUCCESS)
    return err;

  for (const char **cmd = init_commands; *cmd; ++cmd) {
    err = modem_exec_at_cmd(m, *cmd, MODEM_TIMEOUT_MS_INFINITY,
                            1,
                            modem_expected_answer(*cmd, modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF));
    if (err != ME_SUCCESS)
      return err;
  }

  modem_USART_change_baud_rate(MODEM_BR_4000000);
  start_ms = m->fn_get_current_ms();
  uint32_t curr_ms;
  for (; (curr_ms = m->fn_get_current_ms()) - start_ms <= 5000; ) {
    err = modem_exec_at_cmd(m, "AT\r", 100,
                            1, modem_expected_answer("AT\r", modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF));
    if (err == ME_SUCCESS)
      break;
  } // for. we got "OK" from modem on 4000000 baud rate

  return err;
}
///////////////////////////////////////////////////////

modem_err_t
modem_wait_for_pb_ready(modem_t *modem,
                        uint32_t timeout_ms) {
  modem_err_t err;
  do {
    uint32_t read_n;
    err = modem_read_at_str(modem,
                            sizeof(modem->at_cmd_buff),
                            timeout_ms,
                            &read_n);
    if (err == ME_TIMEOUT)
      break;
    UNUSED(read_n);
  } while (strcmp("\r\nPB DONE\r\n", modem->at_cmd_buff));
  return err;
}
///////////////////////////////////////////////////////

const char *
modem_ok_str() {
  return "\r\nOK\r\n";
}
///////////////////////////////////////////////////////
