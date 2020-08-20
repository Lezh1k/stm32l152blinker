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
  ME_SUCCESS = 0,
  ME_CMD_ECHO_ERR,
  ME_UNEXPECTED_ANSWER,
  ME_NOT_IMPLEMENTED
} modem_err_t;

typedef enum modem_state {
  MS_NOT_INITIALIZED = 0,
} modem_state_t;

typedef struct modem_parse_cmd_res {
  modem_err_t code;
  char *str_answer;
} modem_parse_cmd_res_t;

typedef uint8_t (*pf_read_byte)(uint16_t);
typedef void (*pf_write_byte)(char);

typedef struct modem {
  pf_read_byte fn_read_byte;
  pf_write_byte fn_write_byte;

  uint16_t data_buff_len;
  char *data_buff; // max 1500 for real device

  char at_cmd_buff[64];
  //  char _padding[2]; maybe compiller is smart enough to add padding
} modem_t;

// data_buff_len should be less or equal to 1500 (because of MTU)
modem_t* modem_create_default(char *data_buff,
                              uint16_t data_buff_len);

modem_t* modem_create(pf_read_byte fn_read_byte,
                      pf_write_byte fn_write_byte,
                      char *data_buff,
                      uint16_t data_buff_len);

// well, here we just wait for "PB DONE" message from modem
modem_err_t modem_wait_for_pb_ready(modem_t *modem);

modem_err_t modem_cmd(modem_t *modem,
                      const char *cmd,
                      const char **expected_answers);

void modem_write_cmd(const char *cmd);
void modem_write_data(const char *buff,
                      uint32_t size);

modem_parse_cmd_res_t modem_parse_cmd_answer(const char *buff,
                                             const char *cmd); //todo make private




#endif // MODEM_H
