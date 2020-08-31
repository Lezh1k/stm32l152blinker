#include <string.h>
#include <stdlib.h>

#include "modem_socket.h"
#include "commons.h"

modem_socket_t ms_socket(modem_t *m) {
  modem_socket_t res;
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

  modem_err_t merr = modem_exec_at_cmd(ms->modem, cmd, MODEM_TIMEOUT_MS_INFINITY,
                                       1, modem_expected_answer(cmd, MODEM_OK_STR, MODEM_USE_MAX_AVAILABLE_AT_BUFF));
  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t
ms_open(modem_socket_t *ms) {
  static const char *cmd = "AT+NETOPEN\r";
  static const char *res_prefix = "\r\n+NETOPEN: ";
  modem_err_t merr;

  merr = modem_exec_at_cmd(ms->modem, cmd, MODEM_TIMEOUT_MS_INFINITY,
                           2,
                           modem_expected_answer(cmd, MODEM_OK_STR, MODEM_USE_MAX_AVAILABLE_AT_BUFF),
                           modem_expected_answer(res_prefix, "-\r\n", MODEM_USE_MAX_AVAILABLE_AT_BUFF));

  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  const char *rp, *mb;
  rp = res_prefix;
  mb = modem_at_buff(ms->modem);

  while (*rp == *mb) {
    ++rp; ++mb;
  }

  if (*mb < '0' && *mb > '9') //it's not a digit
    return MSE_HARDWARE_ERR_BASE + ME_UNEXPECTED_ANSWER;

  ms->sock_num = (uint16_t)(*mb - '0'); //set port number
  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

ms_error_t
ms_connect_tcp(modem_socket_t *ms,
               const char *ip,
               uint16_t port) {
  char cmd[64] = {0};
  char exp[32] = {0};
  ms_error_t err;
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

  //exp
  //\r\n+CIPOPEN: 0,0\r\n
  strcat(exp, "\r\n+CIPOPEN: ");
  strcat(exp, u16_to_str(ms->sock_num));
  strcat(exp, ",0\r\n");

  // here we can receive some error
  // todo get this error from modem buff.
  merr = modem_exec_at_cmd(ms->modem, cmd, ms->cipopen_timeout_ms + 30,
                           2,
                           modem_expected_answer(cmd, MODEM_OK_STR, MODEM_USE_MAX_AVAILABLE_AT_BUFF),
                           modem_expected_answer("", exp, MODEM_USE_MAX_AVAILABLE_AT_BUFF));

  if (merr != ME_SUCCESS)
    return MSE_HARDWARE_ERR_BASE + merr;

  return MSE_SUCCESS;
}
///////////////////////////////////////////////////////

int32_t
ms_send_byte(modem_socket_t *ms,
             uint8_t b) {

}
///////////////////////////////////////////////////////

int32_t
ms_recv(modem_socket_t *ms,
        uint8_t *buff,
        uint16_t buff_len) {
  return -1;
}
///////////////////////////////////////////////////////

ms_error_t ms_close(modem_socket_t *ms) {
  return MSE_NOT_IMPLEMENTED;
}
///////////////////////////////////////////////////////

