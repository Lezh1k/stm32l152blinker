#ifndef MODEM_SOCKET_H
#define MODEM_SOCKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modem.h"

typedef enum ms_error {
  MSE_SUCCESS,

  MSE_HARDWARE_ERR_BASE = 100,

  MSE_NETWORK_ERR_BASE = 200,

  MSE_NOT_IMPLEMENTED = 300
} ms_error_t ;

typedef struct modem_socket {
  modem_t *modem;
  uint32_t netopen_timeout_ms;
  uint32_t cipopen_timeout_ms;
  uint32_t cipsend_timeout_ms;
  uint16_t sock_num; // netopen result
  uint8_t data_sending;
} modem_socket_t ;

modem_socket_t ms_socket(modem_t *m);
ms_error_t ms_set_timeouts(modem_socket_t *ms,
                           uint16_t netopen_timeout_ms,
                           uint16_t cipopen_timeout_ms,
                           uint16_t cipsend_timeout_ms);

ms_error_t ms_open(modem_socket_t *ms);

ms_error_t ms_connect_tcp(modem_socket_t *ms,
                          const char *ip,
                          uint16_t port);

int32_t ms_send(modem_socket_t *ms,
                uint8_t *buff,
                uint16_t buff_len);

int32_t ms_recv(modem_socket_t *ms,
                uint16_t timeout_ms,
                uint8_t *buff,
                uint16_t buff_len);

ms_error_t ms_set_data_mode(modem_socket_t *ms);
ms_error_t ms_set_cmd_mode(modem_socket_t *ms);

ms_error_t ms_close(modem_socket_t *ms);

#ifdef __cplusplus
}
#endif // extern "C"
#endif // MODEM_SOCKET_H
