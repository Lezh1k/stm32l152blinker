#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_tim.h"
#include "misc.h"

#define GPIO_BLUE_LED_PIN GPIO_Pin_6
#define GPIO_GREEN_LED_PIN GPIO_Pin_7
#define GPIO_LED_PORT GPIOB

#define GPIO_MODEM_TURN_ON_PORT GPIOA
#define GPIO_MODEM_TURN_ON_PIN GPIO_Pin_5

static void init_TIM2(void);
static void init_LEDS(void);

static void modem_init_USART(void);
static void modem_turn(bool on);
static void modem_write_cmd(const char *cmd);

static inline uint8_t modem_read_byte(uint32_t timeout);
static uint32_t modem_read_str(char *buff, uint32_t buff_size);

#define MODEM_PB_DONE_STR "\r\nPB DONE\r\n"
//#define MODEM_PB_DONE_STR "\r\n01234\r\n"

int main(void) {
  char buff[128] = {0};
  int rr;
  init_LEDS();
  init_TIM2();

  modem_init_USART();
  modem_turn(true);

  do {
    rr = modem_read_str(buff, sizeof (buff));
    if (!rr)
      continue;
  } while (strcmp(MODEM_PB_DONE_STR, buff));
  GPIO_SetBits(GPIO_LED_PORT, GPIO_GREEN_LED_PIN);

  // init modem. set baud rate and CTS/RTS

  while(1) {
//    rr = modem_read_str(buff, sizeof(buff));
//    if (!rr)
//      continue;
//    buff[rr] = 0;
//    modem_write_cmd(buff);
  }
  return 0;
}
///////////////////////////////////////////////////////

uint8_t modem_read_byte(uint32_t timeout) {
  (void)timeout; //todo use this!!
  while (!(USART1->SR & USART_SR_RXNE))
    ;
  return (uint8_t) (USART1->DR & 0x00ff);
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
uint32_t modem_read_str(char *buff,
                        uint32_t buff_size) {
  uint32_t rn;
  for (rn = 0; rn < buff_size; ++rn) {
    buff[rn] = modem_read_byte(200);
    if (buff[rn] != '\n')
      continue;
    if (rn <= 1)
      continue;
    break;
  }
  buff[++rn] = 0;
  return rn;
}
///////////////////////////////////////////////////////

void modem_write_cmd(const char *cmd) {
  for (; *cmd; ++cmd) {
    while(!(USART1->SR & USART_SR_TXE))
      ;
    USART1->DR = *cmd;
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

void modem_init_USART(void) {
  //config GPIO
  //USART1:
  //rts --> PA12
  //cts --> PA11
  //rx --> PA10
  //tx --> PA9
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitTypeDef ioCfg;
  ioCfg.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9;
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_PP;
  ioCfg.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &ioCfg);

  //set AF to GPIOA
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

//  with this doesn't work. WHY?
//  GPIO_InitTypeDef ioCfgTurn;
//  ioCfg.GPIO_Pin = GPIO_MODEM_TURN_ON_PIN;
//  ioCfg.GPIO_Mode = GPIO_Mode_OUT;
//  ioCfg.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_Init(GPIO_MODEM_TURN_ON_PORT, &ioCfgTurn);

  //  config usart
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  USART_InitTypeDef ucfg;
  ucfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  ucfg.USART_Parity = USART_Parity_No;
  ucfg.USART_BaudRate = 115200; // init baudrate
//  ucfg.USART_BaudRate = 4000000;
  ucfg.USART_StopBits = 1;
  ucfg.USART_WordLength = USART_WordLength_8b;
  ucfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;
  USART_OverSampling8Cmd(USART1, ENABLE);
  USART_Init(USART1, &ucfg);

// todo enable interrupt if necessary
  USART_Cmd(USART1, ENABLE);
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
  static bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    if ((on = !on)) {
      GPIO_SetBits(GPIO_LED_PORT, GPIO_BLUE_LED_PIN);
    } else {
      GPIO_ResetBits(GPIO_LED_PORT, GPIO_BLUE_LED_PIN);
    }
  }
}
///////////////////////////////////////////////////////

void init_LEDS(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitTypeDef cfg = {
    .GPIO_Pin = GPIO_BLUE_LED_PIN | GPIO_GREEN_LED_PIN,
    .GPIO_Mode = GPIO_Mode_OUT,
    .GPIO_Speed = GPIO_Speed_2MHz
  };
  GPIO_Init(GPIO_LED_PORT, &cfg);
}
///////////////////////////////////////////////////////

void init_TIM2(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef tcfg = {
    .TIM_ClockDivision = TIM_CKD_DIV1, //32 000 000
    .TIM_Prescaler = 31999, //here we have 1000 ticks per second?
    .TIM_CounterMode = TIM_CounterMode_Up,
    .TIM_Period = 999 //1 sec
  };

  TIM_TimeBaseInit(TIM2, &tcfg);
  TIM_Cmd(TIM2, ENABLE);

  NVIC_InitTypeDef nvicCfg = {
    .NVIC_IRQChannel = TIM2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 3,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };

  NVIC_Init(&nvicCfg);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}
///////////////////////////////////////////////////////
