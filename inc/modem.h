#ifndef MODEM_H
#define MODEM_H

#include <stdint.h>
#include <stdbool.h>

#define MODEM_PB_DONE_STR "\r\nPB DONE\r\n"
#define MODEM_OK_STR "\r\nOK\r\n"
#define MODEM_ERROR_STR "\r\nERROR\r\n"

#define MODEM_BR_4000000 4000000
#define MODEM_BR_115200 115200

typedef enum modem_err {
  MODEM_SUCCESS = 0,
  MODEM_CMD_ECHO_ERR
} modem_err_t;

typedef struct modem_parse_cmd_res {
  modem_err_t code;
  char *str_answer;
} modem_parse_cmd_res_t;

void modem_init_USART(void);
void modem_USART_change_baud_rate(uint32_t br); //todo make private
void modem_turn(bool on);
void modem_write_cmd(const char *cmd);
void modem_write_data(const char *buff, uint32_t size);
modem_parse_cmd_res_t modem_parse_cmd_answer(const char *buff,
                                             const char *cmd); //todo make private

uint8_t modem_read_byte(uint32_t timeout); //todo make private
uint32_t modem_read_str(char *buff,
                        uint32_t buff_size);
uint32_t modem_read_str_timeout(char *buff,
                                uint32_t buff_size,
                                uint32_t timeout_between_symbols_ticks);

#endif // MODEM_H
