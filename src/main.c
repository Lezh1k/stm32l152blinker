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

#define MODEM_USART USART1
#define MODEM_GPIO_AF_USART GPIO_AF_USART1

static void init_TIM2(void);

static void led_init(void);
static void led_green_turn(bool on);
static void led_blue_turn(bool on);

static void modem_init_USART(void);
static void modem_USART_set_baud_rate(uint32_t br);
static void modem_turn(bool on);
static void modem_write_cmd(const char *cmd);

static inline uint8_t modem_read_byte(uint32_t timeout);
static uint32_t modem_read_str(char *buff, uint32_t buff_size);

#define MODEM_PB_DONE_STR "\r\nPB DONE\r\n"
#define MODEM_OK_STR "\r\nOK\r\n"
#define MODEM_ERROR_STR "\r\nERROR\r\n"

int main(void) {
  char buff[128] = {0};
  int rr;
  led_init();
//  init_TIM2();

  modem_init_USART();
  modem_turn(true);

  led_green_turn(false);
  led_blue_turn(false);

  do {
    rr = modem_read_str(buff, sizeof (buff));
    if (!rr)
      continue;
  } while (strcmp(MODEM_PB_DONE_STR, buff));

  // init modem. set baud rate and CTS/RTS
  // 0. Set echo off
  modem_write_cmd("ATE0\r");
  rr = modem_read_str(buff, sizeof (buff));
  if (strcmp("ATE0\r\r\nOK\r\n", buff)) {
    led_green_turn(false);
  }

  led_green_turn(true);

  // 1. Set CTS/RTS (cts at least).
  modem_write_cmd("AT+IFC=2,2\r");
  rr = modem_read_str(buff, sizeof (buff));
  if (strcmp(MODEM_OK_STR, buff)) {
    led_green_turn(false);
  }

  // 2. Set modem baud/rate temporary
  modem_write_cmd("AT+IPR=4000000\r");
  rr = modem_read_str(buff, sizeof (buff));
  if (strcmp(MODEM_OK_STR, buff)) {
    led_green_turn(false);
  }

  modem_USART_set_baud_rate(4000000);
  USART_Cmd(MODEM_USART, ENABLE); //turn on modem usart

  // 3. Check that we can communicate
  modem_write_cmd("AT\r");
  rr = modem_read_str(buff, sizeof (buff));
  if (strcmp(MODEM_OK_STR, buff)) {
    led_blue_turn(false);
  }
  led_blue_turn(true);

  while(1) {
  }
  return 0;
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

void modem_USART_set_baud_rate(uint32_t br) {
  USART_InitTypeDef ucfg;
  USART_Cmd(MODEM_USART, DISABLE); //turn off modem
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
  modem_USART_set_baud_rate(115200); // init baud rate is 115200
  USART_Cmd(MODEM_USART, ENABLE);
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
  static bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    led_blue_turn(on = !on);
  }
}
///////////////////////////////////////////////////////

void led_green_turn(bool on) {
  if (on) {
    GPIO_SetBits(GPIO_LED_PORT, GPIO_GREEN_LED_PIN);
    return;
  }
  GPIO_ResetBits(GPIO_LED_PORT, GPIO_GREEN_LED_PIN);
}
///////////////////////////////////////////////////////

void led_blue_turn(bool on) {
  if (on) {
    GPIO_SetBits(GPIO_LED_PORT, GPIO_BLUE_LED_PIN);
    return;
  }
  GPIO_ResetBits(GPIO_LED_PORT, GPIO_BLUE_LED_PIN);
}
///////////////////////////////////////////////////////

void led_init(void) {
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
