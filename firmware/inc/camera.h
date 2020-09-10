#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

typedef enum camera_err {
  CE_SUCCESS,
  CE_I2C_HW_ERROR = 50,
  CE_SPI_HW_ERROR = 100,
  CE_SPI_TIMEOUT,
  CE_NOT_IMPLEMENTED = 150,
} camera_err_t;

camera_err_t camera_init(void);
camera_err_t camera_snapshot_take(uint32_t timeout_ms);
camera_err_t camera_snapshot_len(uint32_t *len);

camera_err_t camera_burst_read_start(void);
camera_err_t camera_burst_read_byte(uint8_t *rb);
camera_err_t camera_burst_read_finish(void);

#endif // CAMERA_H
