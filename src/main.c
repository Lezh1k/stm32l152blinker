#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_tim.h"
#include "misc.h"

#define GPIO_BLUE_LED_PIN GPIO_Pin_6
#define GPIO_GREEN_LED_PIN GPIO_Pin_7
#define GPIO_LED_PORT GPIOB

#define MODEM_USART USART2

static void initTIM2(void);
static void initLEDS(void);

static void modem_initUSART(void);
static void modem_writeCommand(const char *cmd);

static inline uint8_t modem_readByte(uint32_t timeout);
static int modem_readStr(uint8_t *buff, uint32_t buffSize);

int main(void) {
  char buff[128] = {0};
  int rr;
  initLEDS();
  initTIM2();
  modem_initUSART();

  modem_writeCommand("ATE0\r");
  rr = modem_readStr((uint8_t*)buff, sizeof (buff));
  while(1) {
    modem_writeCommand("AT\r");
    rr = modem_readStr((uint8_t*)buff, sizeof (buff));

    modem_writeCommand("AT+IPR?\r");
    rr = modem_readStr((uint8_t*)buff, sizeof (buff));

    modem_writeCommand("AT+IPR=?\r");
    rr = modem_readStr((uint8_t*)buff, sizeof (buff));
  }
  return 0;
}
///////////////////////////////////////////////////////

uint8_t modem_readByte(uint32_t timeout) {
  (void)timeout; //todo use this!!
  while (!(MODEM_USART->SR & USART_SR_RXNE))
    ; //nop()
  return (uint8_t) (MODEM_USART->DR & 0x00ff);
}
///////////////////////////////////////////////////////

//modem response has this format : <cr><lf>response<cr><lf>
int modem_readStr(uint8_t *buff,
                  uint32_t buffSize) {
  //primitive check that we got something that we can use
  uint8_t c = modem_readByte(200);
  if (c != '\r')
    return -1;
  c = modem_readByte(200);
  if (c != '\n')
    return -2;

  uint8_t *b = buff;
  for (; buffSize-- > 0; ++b) {
    *b = modem_readByte(200);
    if (*b == '\n')
      break;
  }
  *b++ = 0;
  //todo check buff size and timeouts.
  return 0;
}
///////////////////////////////////////////////////////

void modem_writeCommand(const char *cmd) {
  for (; *cmd; ++cmd) {
    while(!(MODEM_USART->SR & USART_SR_TXE))
      ;
    MODEM_USART->DR = *cmd;
  }
}
///////////////////////////////////////////////////////

void USART2_IRQHandler(void) {
  GPIO_LED_PORT->BSRRL |= GPIO_GREEN_LED_PIN;
}
///////////////////////////////////////////////////////

void modem_initUSART(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  //config GPIO
  //USART2:
  //cts --> PA0
  //rts --> PA1
  //tx --> PA2
  //rx --> PA3
  GPIO_InitTypeDef ioCfg;
  ioCfg.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  ioCfg.GPIO_Mode = GPIO_Mode_AF;
  ioCfg.GPIO_Speed = GPIO_Speed_40MHz;
  ioCfg.GPIO_OType = GPIO_OType_PP;
  ioCfg.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &ioCfg);

  //set AF to GPIOA
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  //  config usart
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  USART_InitTypeDef ucfg;
  ucfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  ucfg.USART_Parity = USART_Parity_No;
  ucfg.USART_BaudRate = 115200;
  ucfg.USART_StopBits = 1;
  ucfg.USART_WordLength = USART_WordLength_8b;
  ucfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
  USART_OverSampling8Cmd(MODEM_USART, ENABLE);
  USART_Init(MODEM_USART, &ucfg);

  // enable interrupt
  NVIC_InitTypeDef ncfg = {
    .NVIC_IRQChannel = USART2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 1,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&ncfg);

  USART_ITConfig(MODEM_USART, USART_IT_ORE_RX, ENABLE);

  //enable usart
  USART_Cmd(MODEM_USART, ENABLE);
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
  static bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    //if (on) led_on() else led_off()
    volatile uint16_t *reg = (on = !on) ? &GPIO_LED_PORT->BSRRL : &GPIO_LED_PORT->BSRRH;
    *reg |= GPIO_BLUE_LED_PIN;
  }
}
///////////////////////////////////////////////////////

void initLEDS(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitTypeDef cfg = {
    .GPIO_Pin = GPIO_BLUE_LED_PIN | GPIO_GREEN_LED_PIN,
    .GPIO_Mode = GPIO_Mode_OUT,
    .GPIO_Speed = GPIO_Speed_2MHz
  };
  GPIO_Init(GPIO_LED_PORT, &cfg);
}
///////////////////////////////////////////////////////

void initTIM2(void) {
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
