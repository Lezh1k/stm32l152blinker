#include <stdbool.h>

#include <stm32l1xx.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_spi.h>
#include <stm32l1xx_gpio.h>

#include "i2c1.h"
#include "camera.h"
#include "camera_regs.h"
#include "commons.h"

#define CAMERA_I2C_W_ADDR 0x60
#define CAMERA_I2C_R_ADDR  0x61

#define CAMERA_SPI SPI1
#define CAMERA_SPI_PORT GPIOB
#define CAMERA_SPI_RCC_GPIO RCC_AHBPeriph_GPIOB

#define CAMERA_SPI_MOSI GPIO_Pin_5
#define CAMERA_SPI_MISO GPIO_Pin_4
#define CAMERA_SPI_SCK  GPIO_Pin_3
#define CAMERA_SPI_CS   GPIO_Pin_8

#define CAMERA_SPI_MOSI_SOURCE GPIO_PinSource5
#define CAMERA_SPI_MISO_SOURCE GPIO_PinSource4
#define CAMERA_SPI_SCK_SOURCE  GPIO_PinSource3

typedef enum cam_i2c_reg {
  ccreg_common7 = 0x12, //page1
  ccreg_common10 = 0x15, //page1
  ccreg_H_size = 0x51, 	//page0
  ccreg_V_size = 0x52,	//page0
  ccreg_page = 0xFF,
} cam_i2c_reg_t;

typedef enum cam_spi_reg {
  ardu_chip_test = 0x00,
  ardu_chip_cap_ctr_reg = 0x01,
  ardu_chip_fifo_addr = 0x04,
  ardu_chip_trigger = 0x41,
  fifo_size_1 = 0x42,
  fifo_size_2 = 0x43,
  fifo_size_3 = 0x44
} cam_spi_reg_t;

//i2c commands
static camera_err_t cam_set_settings_page(uint8_t page);
static camera_err_t cam_write_i2c_regs(const camera_reg_t *regs);
static camera_err_t cam_write_i2c_reg(camera_reg_t reg);

//spi commands
static void spi_init(void);
static void cam_spi_cs_high(void);
static void cam_spi_cs_low(void);

static camera_err_t cam_spi_write_reg(uint8_t reg,
                                      uint8_t val);

static camera_err_t cam_spi_read_reg(uint8_t reg,
                                     uint8_t *ptr_val);

static camera_err_t cam_spi_read_write_data(uint8_t *tx,
                                            uint8_t *rx,
                                            uint16_t len);

static camera_err_t spi_read_write_byte(uint8_t w,
                                        uint8_t *ptr_val,
                                        uint32_t timeout_ms);

static camera_err_t cam_fifo_flush(void);
static camera_err_t cam_fifo_clear_write_done_flag(void);
static camera_err_t cam_fifo_start_capture(void);
static camera_err_t cam_fifo_reset_read_ptr(void);
static camera_err_t cam_wait_for_capture_done(uint32_t timeout_ms);

camera_err_t
camera_init(void) {
  camera_err_t err;
  i2c1_init();
  spi_init();
  cam_spi_cs_high();

  err = cam_set_settings_page(0);
  if (err != CE_SUCCESS)
    return err;

  err = cam_write_i2c_regs(OV2640_JPEG_INIT);
  if (err != CE_SUCCESS)
    return err;

  err = cam_write_i2c_regs(OV2640_YUV422);
  if (err != CE_SUCCESS)
    return err;

  err = cam_write_i2c_regs(OV2640_JPEG);
  if (err != CE_SUCCESS)
    return err;

  err = cam_set_settings_page(1);
  if (err != CE_SUCCESS)
    return err;

  err = cam_write_i2c_reg(camera_reg(ccreg_common10, 0x00));
  if (err != CE_SUCCESS)
    return err;

  return cam_write_i2c_regs(OV2640_800x600_JPEG);
}
///////////////////////////////////////////////////////

camera_err_t
cam_set_settings_page(uint8_t page) {
  i2c1_finish_code fc = i2c1_write_buff_sync(CAMERA_I2C_W_ADDR,
                                             ccreg_page,
                                             &page, 1);
  return fc == FC_FINISHED ? CE_SUCCESS : CE_I2C_HW_ERROR + fc;
}
///////////////////////////////////////////////////////

camera_err_t
cam_write_i2c_reg(camera_reg_t reg) {
  i2c1_finish_code fc = i2c1_write_buff_sync(CAMERA_I2C_W_ADDR,
                                             reg.reg,
                                             &reg.val,
                                             1);
  return fc == FC_FINISHED ? CE_SUCCESS : CE_I2C_HW_ERROR + fc;
}
///////////////////////////////////////////////////////

camera_err_t
cam_write_i2c_regs(const camera_reg_t *regs) {
  camera_err_t err;
  for (; regs->reg != 0xff && regs->val != 0xff; ++regs) {
    err = cam_write_i2c_reg(*regs);
    if (err != CE_SUCCESS)
      return err;
  }
  return CE_SUCCESS;
}

void
spi_init(void) {
  GPIO_InitTypeDef gpio_cfg;
  SPI_InitTypeDef spi_cfg;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHBPeriphClockCmd(CAMERA_SPI_RCC_GPIO, ENABLE);

  gpio_cfg.GPIO_Pin = CAMERA_SPI_SCK | CAMERA_SPI_MISO | CAMERA_SPI_MOSI;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_Speed = GPIO_Speed_400KHz; //HIGHEST speed

  GPIO_Init(CAMERA_SPI_PORT, &gpio_cfg);

  GPIO_PinAFConfig(CAMERA_SPI_PORT, CAMERA_SPI_SCK_SOURCE, GPIO_AF_SPI1);
  GPIO_PinAFConfig(CAMERA_SPI_PORT, CAMERA_SPI_MISO_SOURCE, GPIO_AF_SPI1);
  GPIO_PinAFConfig(CAMERA_SPI_PORT, CAMERA_SPI_MOSI_SOURCE, GPIO_AF_SPI1);

  gpio_cfg.GPIO_Pin = CAMERA_SPI_CS;
  gpio_cfg.GPIO_Mode = GPIO_Mode_OUT;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_cfg.GPIO_OType = GPIO_OType_PP;
  gpio_cfg.GPIO_Speed = GPIO_Speed_400KHz;

  GPIO_Init(CAMERA_SPI_PORT, &gpio_cfg);

  spi_cfg.SPI_NSS = SPI_NSS_Soft;
  spi_cfg.SPI_CPHA = SPI_CPHA_1Edge;
  spi_cfg.SPI_CPOL = SPI_CPOL_Low;
  spi_cfg.SPI_Mode = SPI_Mode_Master;
  spi_cfg.SPI_DataSize = SPI_DataSize_8b;
  spi_cfg.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_cfg.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_cfg.SPI_CRCPolynomial = 7;
  spi_cfg.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 1Mhz. We can increase this!!!!!!!

  SPI_Init(CAMERA_SPI, &spi_cfg);
  SPI_Cmd(CAMERA_SPI, ENABLE);
}
///////////////////////////////////////////////////////

void
cam_spi_cs_high(void) {
  GPIO_SetBits(CAMERA_SPI_PORT, CAMERA_SPI_CS);
}
///////////////////////////////////////////////////////

void
cam_spi_cs_low(void) {
  GPIO_ResetBits(CAMERA_SPI_PORT, CAMERA_SPI_CS);
}
///////////////////////////////////////////////////////

camera_err_t
cam_fifo_flush(void) {
  return cam_spi_write_reg(ardu_chip_fifo_addr, 0x01); // have no idea why
}

camera_err_t
cam_fifo_clear_write_done_flag(void) {
  return cam_spi_write_reg(ardu_chip_fifo_addr, 0x01);
}
///////////////////////////////////////////////////////

camera_err_t
cam_fifo_start_capture(void) {
  return cam_spi_write_reg(ardu_chip_fifo_addr, 0x02);
}
///////////////////////////////////////////////////////

camera_err_t
cam_fifo_reset_read_ptr(void) {
  return cam_spi_write_reg(ardu_chip_fifo_addr, 0x04);
}
///////////////////////////////////////////////////////

camera_err_t
cam_wait_for_capture_done(uint32_t timeout_ms) {
#define cap_is_done_msk 0x08
  camera_err_t err = CE_SUCCESS;
  uint32_t start_ms, curr_ms;
  uint8_t trig_reg;
  volatile bool is_timeout = false;

  start_ms = curr_ms = get_tick();
  do {
    curr_ms = get_tick();
    is_timeout = curr_ms - start_ms >= timeout_ms;
    err = cam_spi_read_reg(ardu_chip_trigger, &trig_reg);
    if (err != CE_SUCCESS)
      return err;
    if (trig_reg & cap_is_done_msk)
      break;
  } while (!is_timeout);
  return is_timeout ? CE_SPI_TIMEOUT : CE_SUCCESS;
#undef cap_is_done_msk
}
///////////////////////////////////////////////////////

camera_err_t
cam_spi_write_reg(uint8_t reg,
                  uint8_t val) {
  uint8_t data[2];
  data[0] = reg | 0x80;
  data[1] = val;
  return cam_spi_read_write_data(data, data, sizeof(data));
}
///////////////////////////////////////////////////////

camera_err_t
cam_spi_read_reg(uint8_t reg,
                 uint8_t *ptr_val) {
  camera_err_t err;
  uint8_t tx[2] = {0};
  uint8_t rx[2] = {0};
  tx[0] = reg & 0x7F;
  tx[1] = 0;

  err = cam_spi_read_write_data(tx, rx, sizeof (tx));
  if (err != CE_SUCCESS)
    return err;
  *ptr_val = rx[1];
  return CE_SUCCESS;
}
///////////////////////////////////////////////////////

camera_err_t
cam_spi_read_write_data(uint8_t *tx,
                        uint8_t *rx,
                        uint16_t len) {
  camera_err_t err = CE_SUCCESS;
  cam_spi_cs_low();
  while (len--) {
    err = spi_read_write_byte(*tx++, rx++, 200);
    if (err != CE_SUCCESS)
      break;
  }
  cam_spi_cs_high();
  return err;
}
///////////////////////////////////////////////////////

camera_err_t
spi_read_write_byte(uint8_t w,
                    uint8_t *ptr_val,
                    uint32_t timeout_ms) {
  camera_err_t err;
  uint32_t start_ms = get_tick();
  volatile uint32_t curr_ms = start_ms;
  volatile bool is_timeout = false;
  do {
    while(!(CAMERA_SPI->SR & SPI_I2S_FLAG_TXE) && !is_timeout) {
      curr_ms = get_tick();
      is_timeout = curr_ms - start_ms > timeout_ms;
    }

    if (is_timeout) {
      err = CE_SPI_TIMEOUT;
      break;
    }

    SPI_I2S_SendData(CAMERA_SPI, w);
    start_ms = get_tick();
    is_timeout = false;

    while(!(CAMERA_SPI->SR & SPI_I2S_FLAG_RXNE) && !is_timeout) {
      curr_ms = get_tick();
      is_timeout = curr_ms - start_ms > timeout_ms;
    }

    if (is_timeout) {
      err = CE_SPI_TIMEOUT;
      break;
    }

    *ptr_val = SPI_I2S_ReceiveData(CAMERA_SPI);
    err = CE_SUCCESS;
  } while (0);

  return err;
}
///////////////////////////////////////////////////////

camera_err_t
camera_snapshot_take(uint32_t timeout_ms) {
  camera_err_t err;
  err = cam_fifo_flush();
  if (err != CE_SUCCESS)
    return err;
  err = cam_fifo_clear_write_done_flag();
  if (err != CE_SUCCESS)
    return err;
  err = cam_spi_write_reg(ardu_chip_cap_ctr_reg, 0x00);
  if (err != CE_SUCCESS)
    return err;
  err = cam_fifo_start_capture();
  if (err != CE_SUCCESS)
    return err;
  return cam_wait_for_capture_done(timeout_ms);
}
///////////////////////////////////////////////////////

camera_err_t
camera_snapshot_len(uint32_t *len) {
  camera_err_t err;
  uint8_t size1 = 0;
  uint8_t size2 = 0;
  uint8_t size3 = 0;
  err = cam_spi_read_reg(fifo_size_1, &size1);
  if (err != CE_SUCCESS)
    return err;
  err = cam_spi_read_reg(fifo_size_2, &size2);
  if (err != CE_SUCCESS)
    return err;
  err = cam_spi_read_reg(fifo_size_3, &size3);
  if (err != CE_SUCCESS)
    return err;
  size3 &= 0x7f;
  *len = ((size3 << 16) | (size2 << 8) | size1) & 0x07fffff;
  return CE_SUCCESS;
}
///////////////////////////////////////////////////////

camera_err_t
camera_burst_read_start(void) {
  uint8_t dummy;
  cam_spi_cs_low();
  return cam_spi_read_reg(0x3C, &dummy);
}
///////////////////////////////////////////////////////

camera_err_t
camera_burst_read_byte(uint8_t *rb) {
  return spi_read_write_byte(0xff, rb, 200);
}
///////////////////////////////////////////////////////

camera_err_t
camera_burst_read_finish() {
  cam_spi_cs_high();
  return CE_SUCCESS;
}
///////////////////////////////////////////////////////
