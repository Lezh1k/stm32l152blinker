#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include "stm32l1xx.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_tim.h"
#include "misc.h"

#include "commons.h"

#include "modem.h"
#include "modem_hw.h"
#include "modem_socket.h"

#include "i2c1.h"

#define GPIO_LED_RED_PIN GPIO_Pin_6
#define GPIO_LED_GREEN_PIN GPIO_Pin_7
#define GPIO_LED_PORT GPIOC

static void TIM2_init(void);
static void LED_init(void);
static void led_red_turn(bool on);
static void led_green_turn(bool on);

#define camera_i2c_write_addr 0x60
#define camera_i2c_read_addr  0x61

modem_t *debug_output;
static void i2c_scan_cb(uint8_t h, uint8_t l) {
  modem_hw_write_byte(debug_output,
                      1000,
                      h);
  modem_hw_write_byte(debug_output,
                      1000,
                      l);
}
///////////////////////////////////////////////////////

int main(void) {
  LED_init();
  TIM2_init();

  debug_output = modem_create_default();

  i2c1_init();
  while (1) {
    uint8_t buff[1];
    i2c1_finish_code fc;
    fc = i2c1_read_buff_sync(0x61, 0x0A, buff, 1);
    fc = i2c1_read_buff_sync(0x61, 0x0B, buff, 1);
  }

  return 0;
}
///////////////////////////////////////////////////////

void
led_red_turn(bool on) {
  if (on) {
    GPIO_SetBits(GPIO_LED_PORT, GPIO_LED_RED_PIN);
  } else {
    GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED_RED_PIN);
  }
}
///////////////////////////////////////////////////////

void led_green_turn(bool on) {
  if (on) {
    GPIO_SetBits(GPIO_LED_PORT, GPIO_LED_GREEN_PIN);
  } else {
    GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED_GREEN_PIN);
  }
}
///////////////////////////////////////////////////////

void
LED_init(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitTypeDef cfg = {
    .GPIO_Pin = GPIO_LED_RED_PIN | GPIO_LED_GREEN_PIN,
    .GPIO_Mode = GPIO_Mode_OUT,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd = GPIO_PuPd_UP,
  };
  GPIO_Init(GPIO_LED_PORT, &cfg);  
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
  static volatile bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    led_red_turn(on = !on);
    led_green_turn(!on);
  }
}
///////////////////////////////////////////////////////

void
TIM2_init(void) {
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
    .NVIC_IRQChannelSubPriority =0,
    .NVIC_IRQChannelCmd = ENABLE
  };

  NVIC_Init(&nvicCfg);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}
///////////////////////////////////////////////////////
