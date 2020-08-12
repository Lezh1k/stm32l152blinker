#include <stddef.h>
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "modem.h"

#define MODEM_USART USART1
#define MODEM_GPIO_AF_USART GPIO_AF_USART1

#define GPIO_MODEM_TURN_ON_PORT GPIOA
#define GPIO_MODEM_TURN_ON_PIN GPIO_Pin_5

static void modem_write_byte(char b);

modem_parse_cmd_res_t modem_parse_cmd_answer(const char *buff,
                                             const char *cmd) {
  modem_parse_cmd_res_t res;
  for (; *cmd; ++cmd, ++buff) {
    if (*cmd == *buff)
      continue;
    res.code = MODEM_CMD_ECHO_ERR;
    return res;
  }
  res.code = MODEM_SUCCESS;
  res.str_answer = (char*) buff; //cause we won't change anything.
  return res;
}
///////////////////////////////////////////////////////

uint8_t modem_read_byte(uint32_t timeout) {
  if (timeout == 0) { //wait infinity
    while (!(MODEM_USART->SR & USART_SR_RXNE))
      ;
    return (uint8_t) (MODEM_USART->DR & 0x00ff);
  }

  while (!(MODEM_USART->SR & USART_SR_RXNE) && --timeout)
    ;
  return timeout ? (uint8_t) (MODEM_USART->DR & 0x00ff) : 0;
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
uint32_t
modem_read_str_timeout(char *buff,
                       uint32_t buff_size,
                       uint32_t timeout_between_symbols_ticks) {
  char ch;
  char *tb = buff;
  bool cr = false;
  uint32_t rn;

  for (rn = 0; rn < buff_size; ++rn) {
    ch = modem_read_byte(timeout_between_symbols_ticks);
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
  return (uint32_t)((ptrdiff_t)(tb-buff));
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
uint32_t modem_read_str(char *buff,
                        uint32_t buff_size) {
  return modem_read_str_timeout(buff, buff_size, 0);
}
///////////////////////////////////////////////////////

void modem_write_byte(char b) {
  while(!(MODEM_USART->SR & USART_SR_TXE))
    ;
  MODEM_USART->DR = b;
}
///////////////////////////////////////////////////////

void modem_write_cmd(const char *cmd) {
  for (; *cmd; ++cmd)
    modem_write_byte(*cmd);
}
///////////////////////////////////////////////////////

void modem_write_data(const char *buff,
                      uint32_t size) {
  while (size--)
    modem_write_byte(*buff++);
}
///////////////////////////////////////////////////////

void modem_turn(bool on) {
  if (on) {
    GPIO_ResetBits(GPIO_MODEM_TURN_ON_PORT, GPIO_MODEM_TURN_ON_PIN);
    return;
  }
  GPIO_SetBits(GPIO_MODEM_TURN_ON_PORT, GPIO_MODEM_TURN_ON_PIN);
}
///////////////////////////////////////////////////////

void modem_USART_change_baud_rate(uint32_t br) {
  USART_InitTypeDef ucfg;
//  USART_Cmd(MODEM_USART, DISABLE); //turn off usart
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

void modem_init_USART(void) {
  //config GPIO
  //USART1:
  //rts --> PA12
  //cts --> PA11
  //rx --> PA10
  //tx --> PA9
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitTypeDef ioCfg;
  ioCfg.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_9; //rts and tx pull up
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_PP;
  ioCfg.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &ioCfg);

  ioCfg.GPIO_Pin = GPIO_Pin_10; //rx to floating
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_OD;
  ioCfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &ioCfg);

  ioCfg.GPIO_Pin = GPIO_Pin_11; //cts pull down
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_OD;
  ioCfg.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &ioCfg);

  //set AF to GPIOA
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, MODEM_GPIO_AF_USART);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, MODEM_GPIO_AF_USART);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, MODEM_GPIO_AF_USART);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, MODEM_GPIO_AF_USART);

  //  with this doesn't work. WHY?
  //  GPIO_InitTypeDef ioCfgTurn;
  //  ioCfg.GPIO_Pin = GPIO_MODEM_TURN_ON_PIN;
  //  ioCfg.GPIO_Mode = GPIO_Mode_OUT;
  //  ioCfg.GPIO_Speed = GPIO_Speed_2MHz;
  //  GPIO_Init(GPIO_MODEM_TURN_ON_PORT, &ioCfgTurn);

  //  config usart
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  modem_USART_change_baud_rate(MODEM_BR_115200); // init baud rate is 115200
}
///////////////////////////////////////////////////////
