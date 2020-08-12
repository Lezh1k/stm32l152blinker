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

static void init_TIM2(void);
static void led_init(void);
static void led_green_turn(bool on);
static void led_blue_turn(bool on);

static char modem_data_buff[0xff];
static uint16_t prepare_modem_buff() {
  static char alphabet[] = "abcdefghijklmnopqrstuvwxyz";
  uint16_t count = 0;
  for (size_t i = 0; i < sizeof(modem_data_buff) - 1; ++i) {
#define ETX 0x03
#define ESC 0x1b
#define CTRL_Z 0x1a
    char ch = alphabet[i % (sizeof(alphabet)-1)];
    switch (ch) {
      case ETX:
      case ESC:
      case CTRL_Z:
        modem_data_buff[count++] = ETX;
      default:
        modem_data_buff[count++] = ch;
    }
#undef ETX
#undef ESC
#undef CTRL_Z
  }
  modem_data_buff[sizeof(modem_data_buff) - 1] = 0;
  return count;
}
///////////////////////////////////////////////////////

int main(void) {
  char buff[128] = {0};
  int rr;
  modem_parse_cmd_res_t mpr; //modem parse res
  led_init();
  //  init_TIM2();

  modem_init_USART();
  modem_turn(true);

  led_green_turn(false);
  led_blue_turn(false);

  do {
    rr = modem_read_str(buff, sizeof(buff));
    if (!rr)
      continue;
  } while (strcmp(MODEM_PB_DONE_STR, buff));

  led_green_turn(true);
  led_blue_turn(true);

  // init modem. set baud rate and CTS/RTS
  // 1. Set CTS/RTS (cts at least).
  modem_write_cmd("AT+IFC=2,2\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+IFC=2,2\r");
  if (mpr.code != MODEM_SUCCESS) {
    led_green_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_green_turn(false);
  }

  // 2. Set modem baud/rate temporary
  modem_write_cmd("AT+IPR=4000000\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+IPR=4000000\r");
  if (mpr.code != MODEM_SUCCESS) {
    led_green_turn(false);
  }
  if (strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_green_turn(false);
  }

  modem_USART_change_baud_rate(MODEM_BR_4000000);
  led_blue_turn(false);

  // 3. Check that we can communicate
  for (;;) { //todo some counter.
    modem_write_cmd("AT\r");
    rr = modem_read_str_timeout(buff, sizeof(buff), 1000);
    mpr = modem_parse_cmd_answer(buff, "AT\r");
    if (mpr.code == MODEM_SUCCESS &&
        strcmp(mpr.str_answer, MODEM_OK_STR) == 0) {
      break;
    }
  }

  led_green_turn(false);
  led_blue_turn(true);

  init_TIM2();
  ///////////////////////////////////////////////////////
  // let's start test! :)

  // 0. set timeouts
  modem_write_cmd("AT+CIPTIMEOUT=10000,10000,10000\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CIPTIMEOUT=10000,10000,10000\r");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 1. netopen - open socket
  modem_write_cmd("AT+NETOPEN\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+NETOPEN\r");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }  

  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != MODEM_SUCCESS ) {
    led_blue_turn(false);
  }
  // todo get interface number! 0 here is interface number!!!!
  if (strcmp("\r\n+NETOPEN: 0\r\n", mpr.str_answer)) {
    led_blue_turn(false);
  }

  // 2. establish connection
  modem_write_cmd("AT+CIPOPEN=0,"); //open
  modem_write_cmd("\"TCP\","); //type of connection : TCP/UDP
  modem_write_cmd("\"77.95.61.25\","); //server.ip
  modem_write_cmd("9090"); //port
  modem_write_cmd("\r"); // end of cmd
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CIPOPEN=0,\"TCP\",\"77.95.61.25\",9090\r");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp(MODEM_OK_STR, mpr.str_answer)) {
    led_blue_turn(false);
  }

  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != MODEM_SUCCESS ) {
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
  if (mpr.code != MODEM_SUCCESS ||
      strcmp(mpr.str_answer, MODEM_OK_STR)) {
    led_blue_turn(false);
  }

  // 5. send data in loop.

  for (int i = 0; i < 1024; ++i) {
    modem_write_cmd(cmd_buff);
    //we are waiting here for "\r\n>"
    //after AT+CIPSEND=0,count we do not receive "\r\n>\r\n"
    //but just "\r\n>". SO THIS IS KIND OF HACK HERE
    rr = modem_read_str(buff, 3);
    mpr = modem_parse_cmd_answer(buff, "");
    if (mpr.code != MODEM_SUCCESS ) {
      led_blue_turn(false);
    }

    if (strcmp("\r\n>", mpr.str_answer)) {
      led_blue_turn(false);
    }

    modem_write_data(modem_data_buff, count);

    rr = modem_read_str(buff, sizeof(buff));
    mpr = modem_parse_cmd_answer(buff, cmd_buff);
    if (mpr.code != MODEM_SUCCESS ||
        strcmp(MODEM_OK_STR, mpr.str_answer)) {
      led_blue_turn(false);
    }

    rr = modem_read_str(buff, sizeof(buff));
    //todo should be +CIPSEND: 0,count,count
    //in other case we should terminate this connection
  }

  // turn on echo
  modem_write_cmd("ATE1\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp(MODEM_OK_STR, mpr.str_answer)) {
    led_blue_turn(false);
  }

  // close socket
  modem_write_cmd("AT+CIPCLOSE=0\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+CIPCLOSE=0\r");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp(MODEM_OK_STR, mpr.str_answer)) {
    led_blue_turn(false);
  }
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp("+CIPCLOSE: 0,0", mpr.str_answer)) {
    led_blue_turn(false);
  }

  // shutdown networking
  modem_write_cmd("AT+NETCLOSE\r");
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+NETCLOSE\r");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp("+CIPCLOSE: 0,0", mpr.str_answer)) {
    led_blue_turn(false);
  }
  rr = modem_read_str(buff, sizeof(buff));
  mpr = modem_parse_cmd_answer(buff, "AT+NETCLOSE\r");
  if (mpr.code != MODEM_SUCCESS ||
      strcmp("+CIPCLOSE: 0,0", mpr.str_answer)) {
    led_blue_turn(false);
  }

  //AT+CFUN=0 - min. functionality mode (almost sleep, 1.3 mA)
  //AT+CFUN=1 - full functionality mode

  while(1) {
    for (int i = 0; i < 3200000; ++i);
    led_blue_turn(true);
    for (int i = 0; i < 3200000; ++i);
    led_blue_turn(false);
  }
  return 0;
}
///////////////////////////////////////////////////////

void TIM2_IRQHandler(void) {
  static bool on = true;
  if (TIM2->SR & TIM_IT_Update) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);    
    led_green_turn(on = !on);
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
