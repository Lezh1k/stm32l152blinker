#include <string.h>
#include <stdlib.h>

#include "modem_socket.h"
#include "commons.h"
#include "modem_hw.h"

modem_socket_t ms_socket(modem_t *m) {
  modem_socket_t res;
  memset(&res, 0, sizeof(modem_socket_t));
  res.modem = m;
  return res;
}
///////////////////////////////////////////////////////

ms_error_t
ms_set_timeouts(modem_socket_t *ms,
                uint16_t netopen_timeout_ms,
                uint16_t cipopen_timeout_ms,
                uint16_t cipsend_timeout_ms) {
  char cmd[33] = {0}; //"AT+CIPTIMEOUT=10000,10000,10000\r"
  ms->netopen_timeout_ms = netopen_timeout_ms;
  ms->cipopen_timeout_ms = cipopen_timeout_ms;
  ms->cipsend_timeout_ms = cipsend_timeout_ms;

  strcat(cmd, "AT+CIPTIMEOUT=");
  strcat(cmd, u16_to_str(netopen_timeout_ms));
  strcat(cmd, ",");
  strcat(cmd, u16_to_str(cipopen_timeout_ms));
  strcat(cmd, ",");
  strcat(cmd, u16_to_str(cipsend_timeout_ms));
  strcat(cmd, "\r");

  modem_err_t merr = modem_exec_at_cmd(ms->modem, cmd, 5000,
                                       1,
                                       modem_expected_answer(cmd, modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF));
  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t
ms_net_open(modem_socket_t *ms) {
  static const char *pcmd = "AT+CIPMODE=1\r";
  static const char *cmd = "AT+NETOPEN\r";
  static const char *res_prefix = "\r\n+NETOPEN: ";
  modem_err_t merr;

  merr = modem_exec_at_cmd(ms->modem, pcmd, MODEM_TIMEOUT_MS_INFINITY,
                           1,
                           modem_expected_answer(pcmd, modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF));

  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  merr = modem_exec_at_cmd(ms->modem, cmd, ms->netopen_timeout_ms,
                           2,
                           modem_expected_answer(cmd, modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF),
                           modem_expected_answer(res_prefix, "--\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF));

  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  const char *rp, *mb;
  rp = res_prefix;
  mb = ms->modem->at_cmd_buff;

  while (*rp == *mb) {
    ++rp; ++mb;
  }

  if (*mb < '0' && *mb > '9') //it's not a digit
    return MSE_HARDWARE_ERR_BASE + ME_UNEXPECTED_ANSWER;

  ms->sock_num = (uint16_t)(*mb - '0'); //set port number. should be 0 here because of transparent mode
  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t
ms_tcp_connect(modem_socket_t *ms,
               const char *ip,
               uint16_t port) {
  char cmd[64] = {0};
  modem_err_t merr;

  //cmd
  //"AT+CIPOPEN=0,\"TCP\",\"212.42.115.163\",56645\r"
  strcat(cmd, "AT+CIPOPEN=");
  strcat(cmd, u16_to_str(ms->sock_num));
  strcat(cmd, ",\"TCP\",\"");
  strcat(cmd, ip);
  strcat(cmd, "\",");
  strcat(cmd, u16_to_str(port));
  strcat(cmd, "\r");

  merr = modem_exec_at_cmd(ms->modem, cmd, ms->cipopen_timeout_ms + 30,
                           1,
                           modem_expected_answer(cmd, "\r\nCONNECT 4000000\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF));

  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t
ms_tcp_disconnect(modem_socket_t *ms) {
  char cmd[32] = {0};

  ms_error_t err = ms_set_cmd_mode(ms);
  if (err != MSE_SUCCESS)
    return err;

  strcat(cmd, "AT+CIPCLOSE=");
  strcat(cmd, u16_to_str(ms->sock_num));
  strcat(cmd, "\r");

  modem_err_t merr = modem_exec_at_cmd(ms->modem, cmd, 1000,
                                       3,
                                       modem_expected_answer(cmd, modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF),
                                       modem_expected_answer("", "\r\nCLOSED\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF),
                                       modem_expected_answer("", "\r\n+CIPCLOSE: --,--\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF));
  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;
  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t ms_net_close(modem_socket_t *ms) {
  modem_err_t merr =
      modem_exec_at_cmd(ms->modem, "AT+NETCLOSE\r", 1000,
                        2,
                        modem_expected_answer("AT+NETCLOSE\r", modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF),
                        modem_expected_answer("", "\r\n+NETCLOSE: -\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF));
  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;
  //todo check errcode from netclose
  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

int32_t
ms_send(modem_socket_t *ms,
        uint8_t *buff,
        uint16_t buff_len) {
  int32_t count = 0;
  while (buff_len--) {
    modem_err_t err =
        ms->modem->fn_write_byte(ms->modem, MODEM_TIMEOUT_MS_INFINITY, (char)(*buff++));
    if (err != ME_SUCCESS)
      break;
    ++count;
  }
  return count;
}
///////////////////////////////////////////////////////

int32_t
ms_recv(modem_socket_t *ms,
        uint16_t timeout_ms,
        uint8_t *buff,
        uint16_t buff_len) {
  return -1;
}
///////////////////////////////////////////////////////

ms_error_t
ms_set_data_mode(modem_socket_t *ms) {
  //todo change to hardware flow: use DTR pin
  modem_err_t merr =
      modem_exec_at_cmd(ms->modem, "ATO\r", 2000,
                        1,
                        modem_expected_answer("ATO\r", "\r\nCONNECT 4000000\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF));
  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;
  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t
ms_set_cmd_mode(modem_socket_t *ms) {
  //todo change to hardware flow: use DTR pin
  modem_err_t merr;
  ms->modem->fn_delay_ms(1000);
  merr = modem_exec_at_cmd(ms->modem, "+++", MODEM_TIMEOUT_MS_INFINITY,
                           1,
                           modem_expected_answer("", modem_ok_str(), MODEM_USE_MAX_AVAILABLE_AT_BUFF));
  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;
  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

