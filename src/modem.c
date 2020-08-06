#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "modem.h"

#define MODEM_USART USART1
#define MODEM_GPIO_AF_USART GPIO_AF_USART1

#define GPIO_MODEM_TURN_ON_PORT GPIOA
#define GPIO_MODEM_TURN_ON_PIN GPIO_Pin_5

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
  (void)timeout; //todo use this!!
  while (!(MODEM_USART->SR & USART_SR_RXNE))
    ;
  return (uint8_t) (MODEM_USART->DR & 0x00ff);
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
uint32_t modem_read_str(char *buff,
                        uint32_t buff_size) {
  uint32_t rn;
  bool cr = false;
  for (rn = 0; rn < buff_size; ++rn) {
    buff[rn] = modem_read_byte(200);
    if (buff[rn] != '\n')
      continue;
    if (cr)
      break;
    cr = true;
  }
  buff[++rn] = 0;
  return rn;
}
///////////////////////////////////////////////////////

void modem_write_cmd(const char *cmd) {
  for (; *cmd; ++cmd) {
    while(!(MODEM_USART->SR & USART_SR_TXE))
      ;
    MODEM_USART->DR = *cmd;
  }
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
  USART_Cmd(MODEM_USART, DISABLE); //turn off usart
  USART_DeInit(MODEM_USART); //set default values to all registers
  //then init
  ucfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  ucfg.USART_Parity = USART_Parity_No;
  ucfg.USART_BaudRate = br;
  ucfg.USART_StopBits = 1;
  ucfg.USART_WordLength = USART_WordLength_8b;
  ucfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;
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
  ioCfg.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_9;
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_PP;
  ioCfg.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &ioCfg);

  ioCfg.GPIO_Pin = GPIO_Pin_10;
  ioCfg.GPIO_Mode = GPIO_Mode_AF; //IN?
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_OD;
  ioCfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
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
