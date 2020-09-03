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

#define GPIO_BLUE_LED_PIN GPIO_Pin_6
#define GPIO_GREEN_LED_PIN GPIO_Pin_7
#define GPIO_LED_PORT GPIOB

static void TIM2_init(void);
static void LED_init(void);
static void led_blue_turn(bool on);
static void led_green_turn(bool on);

int main(void) {
  modem_t *m_modem;
  modem_err_t m_err;

  modem_socket_t m_sock;
  ms_error_t ms_err;

  SysTick_Config(SystemCoreClock / 1000); //1 ms tick. see commons.c
  LED_init();
  m_modem = modem_create_default();

  led_green_turn(true);
  m_err = modem_prepare_to_work(m_modem);
  if (m_err != ME_SUCCESS) {
    led_green_turn(false);
  }

  m_sock = ms_socket(m_modem);
  do {
    ms_err = ms_set_timeouts(&m_sock, 8000, 3000, 3000);
    if (ms_err != MSE_SUCCESS) {
      led_green_turn(false);
      break;
    }

    ms_err = ms_net_open(&m_sock);
    if (ms_err != MSE_SUCCESS) {
      led_green_turn(false);
      break;
    }

    // speed test
    for (int i = 0; i < 10; ++i) {
      ms_err = ms_tcp_connect(&m_sock, "212.42.115.163", 45223);
      if (ms_err != MSE_SUCCESS)
        break;

      for (int j = 0; j < 1024; ++j) {
        int sent = ms_send(&m_sock, (uint8_t*)"123456789012345678901234567890", 30);
        if (sent != 30) {
          ms_err = MSE_NETWORK_ERR_BASE;
          break;
        }
      }
      if (ms_err != MSE_SUCCESS)
        break;

      ms_err = ms_tcp_disconnect(&m_sock);
      delay_ms(2000);
    }

    if (ms_err != MSE_SUCCESS)
      break;

    ms_err = ms_net_close(&m_sock);
  } while (0);

  if (ms_err == MSE_SUCCESS)
    TIM2_init();

  while (1) {
  }

  return 0;
}
///////////////////////////////////////////////////////


void
led_blue_turn(bool on) {
  if (on) {
    GPIO_SetBits(GPIO_LED_PORT, GPIO_BLUE_LED_PIN);
  } else {
    GPIO_ResetBits(GPIO_LED_PORT, GPIO_BLUE_LED_PIN);
  }
}
///////////////////////////////////////////////////////

void led_green_turn(bool on) {
  if (on) {
    GPIO_SetBits(GPIO_LED_PORT, GPIO_GREEN_LED_PIN);
  } else {
    GPIO_ResetBits(GPIO_LED_PORT, GPIO_GREEN_LED_PIN);
  }
}
///////////////////////////////////////////////////////

void
LED_init(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitTypeDef cfg = {
    .GPIO_Pin = GPIO_BLUE_LED_PIN | GPIO_GREEN_LED_PIN,
    .GPIO_Mode = GPIO_Mode_OUT,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd = GPIO_PuPd_UP,
  };

  GPIO_Init(GPIO_LED_PORT, &cfg);

  GPIO_InitTypeDef cfg2 = {
    .GPIO_Pin = GPIO_Pin_0,
    .GPIO_Mode = GPIO_Mode_IN,
    .GPIO_Speed = GPIO_Speed_2MHz,
  };
  GPIO_Init(GPIOA, &cfg2);
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
  static volatile bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    led_blue_turn(on = !on);
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
