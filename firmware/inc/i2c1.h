#ifndef I2C1_H
#define I2C1_H

#include <stdint.h>
#include <stm32l1xx.h>
#include <stm32l1xx_i2c.h>

typedef enum {
  FC_FINISHED = 0,
  FC_IN_PROGRESS  = 1,
  FC_I2C_ERR = 2,
  FC_DFA_ERR = 3
} i2c1_finish_code;

void i2c1_init(void);
void i2c1_interrupts(FunctionalState on);

typedef void (*i2c1_pf_callback)(i2c1_finish_code);

void i2c1_read_buff_async(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len, volatile int8_t *finishCode, i2c1_pf_callback cb);
i2c1_finish_code i2c1_read_buff_sync(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len);

void i2c1_write_buff_async(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len, volatile int8_t *finishCode, i2c1_pf_callback cb);
i2c1_finish_code i2c1_write_buff_sync(uint8_t slaveAddr, uint8_t startReg, uint8_t *buff, uint8_t len);

void i2c1_scan(void (*cb)(uint8_t, uint8_t));

#endif // I2C1_H
