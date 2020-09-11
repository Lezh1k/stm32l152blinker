#include <stdint.h>
#include <stdbool.h>

#include <stm32l1xx.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_tim.h>
#include <misc.h>

#include "commons.h"
#include "modem.h"
#include "modem_hw.h"
#include "modem_socket.h"
#include "camera.h"

#include "i2c1.h"

#define GPIO_LED_RED_PIN GPIO_Pin_7
#define GPIO_LED_GREEN_PIN GPIO_Pin_6
#define GPIO_LED_PORT GPIOC

static void TIM2_init(void);
static void LED_init(void);
static void led_red_turn(bool on);
static void led_green_turn(bool on);

static void normal_mode(void) __attribute__((noreturn, unused));
static void camera_test_mode(void) __attribute__((noreturn, unused));

int main(void) {
//  normal_mode();
  camera_test_mode();
}
///////////////////////////////////////////////////////

void
camera_test_mode(void) {
#define test_buff_len 20
  uint8_t cam_data[test_buff_len] = {0};
  camera_err_t cam_err;
  uint32_t img_len;
  uint8_t tmp_byte;
  delay_init();
  LED_init();
  TIM2_init();

  led_red_turn(false);
  led_green_turn(false);

  do {
    cam_err = camera_init();
    if (cam_err != CE_SUCCESS) {
      led_red_turn(true);
      break;
    }

    cam_err = camera_snapshot_take(1000);
    if (cam_err != CE_SUCCESS) {
      led_red_turn(true);
      break;
    }

    cam_err = camera_snapshot_len(&img_len);
    if (cam_err != CE_SUCCESS) {
      led_red_turn(true);
      break;
    }

    cam_err = camera_burst_read_start();
    if (cam_err != CE_SUCCESS) {
      led_red_turn(true);
      goto lbl_cam_err;
    }

    //ignore first byte in burst read. so!
    cam_err = camera_burst_read_byte(&tmp_byte);
    if (cam_err != CE_SUCCESS) {
      led_red_turn(true);
      goto lbl_cam_err;
    }

//    while (img_len--) {
//      cam_err = camera_burst_read_byte(&tmp_byte);
//      if (cam_err != CE_SUCCESS)
//        break;
//      //print this byte
//    }

    for (int i = 0; i < test_buff_len; ++i) {
      cam_err = camera_burst_read_byte(&cam_data[i]);
      if (cam_err != CE_SUCCESS)
        break;
    }

    led_green_turn(true);
    // without goto worse, sorry.
lbl_cam_err:
    camera_burst_read_finish();
  } while (0);

  while (1)
    ;
}
///////////////////////////////////////////////////////

void
normal_mode(void) {
  modem_err_t m_err;
  ms_error_t ms_err;
  camera_err_t cam_err;

  modem_t *m_modem;
  modem_socket_t m_sock;

  bool is_initialized = false;
  uint32_t img_len;
  uint8_t tmp_byte;

  delay_init();
  LED_init();
  TIM2_init();

  led_red_turn(false);
  led_green_turn(false);
  m_modem = modem_create_default();
  do {
    // init camera
    cam_err = camera_init();
    if (cam_err != CE_SUCCESS) {
      led_red_turn(true);
      break;
    }

    // init modem
    m_err = modem_prepare_to_work(m_modem);
    if (m_err != ME_SUCCESS) {
      led_red_turn(true);
      break;
    }

    m_sock = ms_socket(m_modem);
    ms_err = ms_set_timeouts(&m_sock, 8000, 3000, 3000);
    if (ms_err != MSE_SUCCESS) {
      led_red_turn(true);
      break;
    }

    ms_err = ms_net_open(&m_sock);
    if (ms_err != MSE_SUCCESS) {
      led_red_turn(true);
      break;
    }

    is_initialized = true;
  } while (0);

  if (is_initialized) {
    led_green_turn(true);

    for (int i = 0; i < 1; ++i) {
      ms_err = ms_tcp_connect(&m_sock, "212.42.115.163", 45223);
      if (ms_err != MSE_SUCCESS) {
        led_red_turn(true);
        break;
      }

      cam_err = camera_snapshot_take(1000);
      if (cam_err != CE_SUCCESS) {
        led_red_turn(true);
        goto lbl_cam_err;
      }

      cam_err = camera_snapshot_len(&img_len);
      if (cam_err != CE_SUCCESS) {
        led_red_turn(true);
        goto lbl_cam_err;
      }

      cam_err = camera_burst_read_start();
      if (cam_err != CE_SUCCESS) {
        led_red_turn(true);
        goto lbl_cam_err;
      }

      //ignore first byte in burst read. so!
      cam_err = camera_burst_read_byte(&tmp_byte);
      if (cam_err != CE_SUCCESS) {
        led_red_turn(true);
        goto lbl_cam_err;
      }

      while (img_len--) {
        cam_err = camera_burst_read_byte(&tmp_byte);
        if (cam_err != CE_SUCCESS)
          break;
        if (ms_send(&m_sock, &tmp_byte, 1) != 1)
          break;
      }
      // without goto worse, sorry.
lbl_cam_err:
      camera_burst_read_finish();
      ms_err = ms_tcp_disconnect(&m_sock);
      if (ms_err != MSE_SUCCESS) {
        led_red_turn(true);
        break;
      }
      delay_ms(2000);
    } //for (i=0; i < N; ++i) take and send snapshot
  } // if (is_initialized)

  while (1) {
  }
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
  //  static volatile bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    //    led_red_turn(on = !on);
    //    led_green_turn(!on);
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
