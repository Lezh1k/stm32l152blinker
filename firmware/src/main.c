#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include "stm32l1xx.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_tim.h"
#include "misc.h"
#include "modem.h"
#include "commons.h"

#define GPIO_BLUE_LED_PIN GPIO_Pin_6
#define GPIO_GREEN_LED_PIN GPIO_Pin_7
#define GPIO_LED_PORT GPIOB

static void TIM2_init(void);
static void LED_init(void);
static void led_blue_turn(bool on);

static volatile bool m_need_to_send_data = false;
static volatile int16_t m_test_time;

#define MODEM_DATA_BUFF_LEN 1500u
static char m_modem_data_buff[MODEM_DATA_BUFF_LEN];

static uint16_t prepare_modem_buff();
static void send_test_data(const char* cmd_buff,
                           uint32_t chunks_count,
                           uint16_t count);

uint16_t
prepare_modem_buff(){
  static char alphabet[] = "abcdefghijklmnopqrstuvwxyz";
  uint16_t count = 0;
  for (size_t i = 0; i < sizeof(m_modem_data_buff) - 1; ++i) {
#define ETX 0x03
#define ESC 0x1b
#define CTRL_Z 0x1a
    char ch = alphabet[i % (sizeof(alphabet)-1)];
    switch (ch) {
      case ETX:
      case ESC:
      case CTRL_Z:
        m_modem_data_buff[count++] = ETX;
      default:
        m_modem_data_buff[count++] = ch;
    }
#undef ETX
#undef ESC
#undef CTRL_Z
  }
  m_modem_data_buff[sizeof(m_modem_data_buff) - 1] = 0;
  return count;
}
///////////////////////////////////////////////////////

void
send_test_data(const char* cmd_buff,
               uint32_t chunks_count,
               uint16_t count) {
  char buff[32] = {0};
  int rr;
  modem_parse_cmd_res_t mpr; //modem parse res
  // 5. send data in loop.
  while (chunks_count--) {
    modem_write_cmd(cmd_buff);
    //we are waiting here for "\r\n>"
    //after AT+CIPSEND=0,count we do not receive "\r\n>\r\n"
    //but just "\r\n>". SO THIS IS KIND OF HACK HERE
    rr = modem_read_str(buff, 3);
    mpr = modem_parse_cmd_answer(buff, "");
    if (mpr.code != ME_SUCCESS) {
      led_blue_turn(false);
    }
    if (strcmp("\r\n>", mpr.str_answer)) {
      led_blue_turn(false);
    }

    //
    modem_write_data(m_modem_data_buff, count);
    rr = modem_read_str(buff, sizeof(buff));
    mpr = modem_parse_cmd_answer(buff, "");
    if (mpr.code != ME_SUCCESS ||
        strcmp(MODEM_OK_STR, mpr.str_answer)) {
      led_blue_turn(false);
    }

    rr = modem_read_str(buff, sizeof(buff));
    //todo should be +CIPSEND: 0,count,count
    //in other case we should terminate this connection
  }
}
///////////////////////////////////////////////////////

int main(void) {
  modem_t *modem;
  modem_err_t modem_err;
  SysTick_Config(SystemCoreClock / 1000); //1 ms tick. see commons.c
  LED_init();
  TIM2_init();

  modem = modem_create_default();
  modem_err = modem_prepare_to_work(modem);
  if (modem_err != ME_SUCCESS) {
    // todo implement error handling
    // with leds.
  }


  led_blue_turn(true);

  ///////////////////////////////////////////////////////
  // let's start test! :)

  // 0. set timeouts
  modem_write_cmd("AT+CIPTIMEOUT=10000,10000,10000\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CIPTIMEOUT=10000,10000,10000\r");
  if (mpr.code != ME_SUCCESS ||
      strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 1. netopen - open socket
  modem_write_cmd("AT+NETOPEN\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+NETOPEN\r");
  if (mpr.code != ME_SUCCESS ||
      strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != ME_SUCCESS ) {
    led_blue_turn(false);
  }
  // todo get interface number! 0 here is interface number!!!!
  if (strcmp("\r\n+NETOPEN: 0\r\n", mpr.str_answer)) {
    led_blue_turn(false);
  }

  // 2. establish connection
  modem_write_cmd("AT+CIPOPEN=0,"); //open
  modem_write_cmd("\"TCP\","); //type of connection : TCP/UDP
  modem_write_cmd("\"212.42.115.163\","); //server.ip
  modem_write_cmd("9094"); //port
  modem_write_cmd("\r"); // end of cmd
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CIPOPEN=0,\"TCP\",\"212.42.115.163\",9094\r");
  if (mpr.code != ME_SUCCESS ||
      strcmp(MODEM_OK_STR, mpr.str_answer)) {
    led_blue_turn(false);
  }

  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != ME_SUCCESS ) {
    led_blue_turn(false);
  }
  if (strcmp("\r\n+CIPOPEN: 0,0\r\n", mpr.str_answer)) {
    led_blue_turn(false);
  }

  // 3. prepare send command
  uint16_t count = prepare_modem_buff();
  char cmd_buff[32] = {0};
  strcat(cmd_buff, "AT+CIPSEND=0,"); //interface number
  strcat(cmd_buff, u16_to_str(count)); //count of bytes to send
  strcat(cmd_buff, "\r");

  // 4. disable echo
  modem_write_cmd("ATE0\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "ATE0\r");
  if (mpr.code != ME_SUCCESS ||
      strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  m_test_time = 60; // 60 sec
  TIM2_init();
  while (m_test_time > 0) {
    if (!m_need_to_send_data)
      continue;
    modem_sleep_mode(false);
    delay_ms(20);

    modem_write_cmd("AT+CIPOPEN=0,"); //open
    modem_write_cmd("\"TCP\","); //type of connection : TCP/UDP
    modem_write_cmd("\"212.42.115.163\","); //server.ip
    modem_write_cmd("9094"); //port
    modem_write_cmd("\r"); // end of cmd
    rr = modem_read_str(buff, sizeof(buff));
    mpr = modem_parse_cmd_answer(buff, "");
    if (mpr.code != ME_SUCCESS ||
        strcmp(MODEM_OK_STR, mpr.str_answer)) {
      led_blue_turn(false);
    }

    rr = modem_read_str(buff, sizeof(buff));
    mpr = modem_parse_cmd_answer(buff, "");
    if (mpr.code != ME_SUCCESS) {
      led_blue_turn(false);
    }
    if (strcmp("\r\n+CIPOPEN: 0,0\r\n", mpr.str_answer)) {
      led_blue_turn(false);
    }

    send_test_data(cmd_buff,
                   30*1024 / sizeof(m_modem_data_buff),
                   count);
    m_need_to_send_data = false;

    modem_write_cmd("AT+CIPCLOSE=0\r");
    rr = modem_read_str(buff, sizeof(buff));
    mpr = modem_parse_cmd_answer(buff, "");
    if (mpr.code != ME_SUCCESS ||
        strcmp(MODEM_OK_STR, mpr.str_answer)) {
      led_blue_turn(false);
    }

    modem_sleep_mode(true);
  }

  // turn on echo
  modem_write_cmd("ATE1\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != ME_SUCCESS ||
      strcmp(MODEM_OK_STR, mpr.str_answer)) {
    led_blue_turn(false);
  }

  // close socket

  modem_write_cmd("AT+CIPCLOSE=0\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CIPCLOSE=0\r");
  if (mpr.code != ME_SUCCESS) {
    led_blue_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  while(1) {
    delay_ms(1000);
    led_blue_turn(true);
    delay_ms(1000);
    led_blue_turn(false);
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

void
LED_init(void) {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_InitTypeDef cfg = {
    .GPIO_Pin = GPIO_BLUE_LED_PIN | GPIO_GREEN_LED_PIN,
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
    led_blue_turn(on = !on);
    m_test_time -= 10;
    m_need_to_send_data = true;
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
    .TIM_Period = 9999 //10 sec
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
