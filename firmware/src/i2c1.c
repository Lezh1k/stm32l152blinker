#include <stddef.h>
#include <misc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>

#include "i2c1.h"
#include "commons.h"

#define I2C_RCC_Periph      RCC_APB1Periph_I2C1
#define I2C_RCC_AHBPort     RCC_AHBPeriph_GPIOB
#define I2C_Port            GPIOB

#define I2C_SCL_Pin         GPIO_Pin_6
#define I2C_SDA_Pin         GPIO_Pin_7

#define I2C1_SCL_PIN_SOURCE GPIO_PinSource6
#define I2C1_SDA_PIN_SOURCE GPIO_PinSource7

#define I2C_Speed           50000

typedef enum {
  I2CEV_StartBitSent = 0, //SB
  I2CEV_AddresSent, //ADDR
  I2CEV_DataByteTransferFinished, //BTF
  I2CEV_ReceiveBufferNotEmpty, //RxNE
  I2CEV_TransmitBufferEmpty //TxE
} I2CEvent;

typedef enum {
  I2CER_ErrClass = 5,
  I2CER_BusError, //BERR
  I2CER_ArbitrationLoss, //ARLO
  I2CER_AckFail, //NACK
  I2CER_Overrun, //OVR
  I2CER_PEC, //PECERR
  I2CER_Timeout, //TIMEOUT
  I2CER_SMBBusAlert //SMBBUSALERT
} I2CError;
///////////////////////////////////////////////////////

//read multiple bytes context
typedef struct rmb_context {
  I2C_TypeDef *i2c;
  volatile uint8_t *dst_buff;
  volatile int8_t *ptr_finish_code;
  i2c1_pf_callback fn_callback;
  volatile uint8_t read_len;
  uint8_t slave_addr;
  uint8_t dst_reg;
//  uint8_t padding[1]; I hope compiller is smart enough for padding
} rmb_context_t;

//write multiple bytes context
typedef struct wmb_context {
  I2C_TypeDef *i2c;
  volatile uint8_t *src_buff;
  volatile int8_t *ptr_finish_code;
  i2c1_pf_callback fn_callback;
  volatile uint8_t write_len;
  uint8_t slave_addr;
  uint8_t dst_reg;
//  uint8_t padding[1]; I hope compiller is smart enough for padding
} wmb_context_t;
///////////////////////////////////////////////////////

#define I2C_ERR_MASK (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT | I2C_SR1_AF)
#define DFA_INIT_STATE 0
#define POSSIBLE_DFA_SIGNALS_N 6
#define I2C1_LONGEST_DFA 6

struct i2c_async_dfa;
typedef struct i2c_async_dfa_state {
  int32_t ix;
  void *arg; //action arg
  void (*actions[POSSIBLE_DFA_SIGNALS_N]) (struct i2c_async_dfa_state*, struct i2c_async_dfa *dfa);
} i2c_async_dfa_state_t;
///////////////////////////////////////////////////////

typedef struct i2c_async_dfa {
  volatile int32_t cs; //current state
  i2c_async_dfa_state_t states[I2C1_LONGEST_DFA];
} i2c_async_dfa_t;
///////////////////////////////////////////////////////

static i2c_async_dfa_t m_dfa_i2c1;
static i2c_async_dfa_t pre_init_dfa(void);

static uint8_t dfa_signal(I2C_TypeDef *i2c);

static void dfa_next(I2C_TypeDef *i2c,
                     i2c_async_dfa_t *dfa);

static void dfa_nop(i2c_async_dfa_state_t *st,
                    i2c_async_dfa_t *dfa);

static void i2c1_reset(void);

i2c_async_dfa_t
pre_init_dfa(void) {
  i2c_async_dfa_t res;
  static i2c_async_dfa_state_t states[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {dfa_nop, dfa_nop, dfa_nop, dfa_nop, dfa_nop, dfa_nop}}, //init
  };
  res.cs = DFA_INIT_STATE;

  for (size_t i = 0; i < sizeof(states)/sizeof(i2c_async_dfa_state_t); ++i) {
    states[i].ix = (int32_t)i;
    res.states[i] = states[i];
  }
  return res;
}
///////////////////////////////////////////////////////

uint8_t
dfa_signal(I2C_TypeDef *i2c) {
  volatile uint16_t sr1, sr2 = 0;
  sr1 = i2c->SR1;

  if (sr1 & I2C_SR1_SB)
    return I2CEV_StartBitSent;

  if (sr1 & I2C_SR1_ADDR) {
    sr2 = i2c->SR2; //clear addr register if set
    return I2CEV_AddresSent;
  }

  if (sr1 & I2C_SR1_BTF)
    return I2CEV_DataByteTransferFinished;

  if (sr1 & I2C_SR1_RXNE)
    return I2CEV_ReceiveBufferNotEmpty;

  if (sr1 & I2C_SR1_TXE)
    return I2CEV_TransmitBufferEmpty;

  if (sr1 & I2C_ERR_MASK)
    return I2CER_ErrClass;
  return 0xff;
}
///////////////////////////////////////////////////////

void
dfa_next(I2C_TypeDef *i2c,
         i2c_async_dfa_t *dfa) {
  uint8_t sig;
  int32_t cs;
  sig = dfa_signal(i2c);
  if (sig != 0xff) {
    cs = dfa->cs;
    dfa->states[cs].actions[sig](&dfa->states[cs], dfa);
  }
}
///////////////////////////////////////////////////////

void
I2C1_EV_IRQHandler() {
  dfa_next(I2C1, &m_dfa_i2c1);
}
///////////////////////////////////////////////////////

void
I2C1_ER_IRQHandler() {
  dfa_next(I2C1, &m_dfa_i2c1);
}
///////////////////////////////////////////////////////

void
i2c1_reset(void) {
  I2C1->CR1 |= I2C_CR1_SWRST;
  //todo reset devices
}
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

static void rmb_i2c_err(i2c_async_dfa_state_t* st, i2c_async_dfa_t *dfa);
static void rmb_dfa_err(i2c_async_dfa_state_t* st, i2c_async_dfa_t *dfa);
static void rmb_gen_start(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void rmb_send_addr_transmitter(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void rmb_send_start_reg(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void rmb_send_addr_receiver(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void rmb_read_byte_btf(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void rmb_finish(i2c_async_dfa_state_t* st, i2c_async_dfa_t *dfa);
static void rmb_addr_received_len1(i2c_async_dfa_state_t* st, i2c_async_dfa_t *dfa);

static void rmb_i2c_err_Berr(rmb_context_t *ctx);
static void rmb_i2c_err_AF(rmb_context_t *ctx);
static void rmb_i2c_err_Arlo(rmb_context_t *ctx);
static void rmb_i2c_err_Ovr(rmb_context_t *ctx);
static void rmb_i2c_err_Timeout(rmb_context_t *ctx);

static i2c_async_dfa_t rmb_dfa_create(uint8_t slaveAddr,
                                      uint8_t startReg,
                                      volatile uint8_t *dstBuff,
                                      uint8_t readLen,
                                      volatile int8_t *pErr,
                                      i2c1_pf_callback cb);

void
rmb_i2c_err_Berr(rmb_context_t *ctx) {
  if (ctx->i2c->SR1 & I2C_IT_SB)
    i2c1_reset(); //if berr set and sb set we need to restart i2c at all.
}
///////////////////////////////////////////////////////

void
rmb_i2c_err_AF(rmb_context_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE); //clear busy flag
}
///////////////////////////////////////////////////////

void
rmb_i2c_err_Arlo(rmb_context_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void
rmb_i2c_err_Ovr(rmb_context_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void
rmb_i2c_err_Timeout(rmb_context_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void
rmb_i2c_err(i2c_async_dfa_state_t* st,
            i2c_async_dfa_t *dfa) {
  volatile uint16_t sr1;
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  sr1 = pctx->i2c->SR1;
  pctx->i2c->SR1 = (sr1 & ~I2C_ERR_MASK);

  if (sr1 & I2C_SR1_BERR)
    rmb_i2c_err_Berr(pctx);
  if (sr1 & I2C_SR1_AF)
    rmb_i2c_err_AF(pctx);
  if (sr1 & I2C_SR1_ARLO)
    rmb_i2c_err_Arlo(pctx);
  if (sr1 & I2C_SR1_OVR)
    rmb_i2c_err_Ovr(pctx);
  if (sr1 & I2C_SR1_TIMEOUT)
    rmb_i2c_err_Timeout(pctx);

  I2C_AcknowledgeConfig(pctx->i2c, ENABLE);
  if (pctx->fn_callback != NULL)
    pctx->fn_callback(FC_I2C_ERR);
  dfa->cs = DFA_INIT_STATE;
  *pctx->ptr_finish_code = FC_I2C_ERR;
}
///////////////////////////////////////////////////////

//todo: we can't be here. SO! just restart device and i2c peripheral
void
rmb_dfa_err(i2c_async_dfa_state_t* st,
            i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  I2C_AcknowledgeConfig(pctx->i2c, ENABLE);
  I2C_GenerateSTOP(pctx->i2c, ENABLE);

  i2c1_reset();
  if (pctx->fn_callback != NULL)
    pctx->fn_callback(FC_DFA_ERR);

  dfa->cs = DFA_INIT_STATE;
  *pctx->ptr_finish_code = FC_DFA_ERR;
}
///////////////////////////////////////////////////////

void
rmb_gen_start(i2c_async_dfa_state_t *st,
              i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  while (I2C_GetFlagStatus(pctx->i2c, I2C_FLAG_BUSY))
    __NOP();
  dfa->cs = 1;
  I2C_GenerateSTART(pctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void
rmb_send_addr_transmitter(i2c_async_dfa_state_t *st,
                          i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  dfa->cs = 2;
  I2C_Send7bitAddress(pctx->i2c, pctx->slave_addr, I2C_Direction_Transmitter);
}
///////////////////////////////////////////////////////

void
rmb_send_start_reg(i2c_async_dfa_state_t *st,
                   i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  dfa->cs = 3;
  I2C_SendData(pctx->i2c, pctx->dst_reg);
  I2C_GenerateSTART(pctx->i2c, ENABLE); //re-start after this byte transmitted.
}
///////////////////////////////////////////////////////

void
dfa_nop(i2c_async_dfa_state_t *st,
        i2c_async_dfa_t *dfa) {
  UNUSED(st);
  UNUSED(dfa);
}
///////////////////////////////////////////////////////

void
rmb_send_addr_receiver(i2c_async_dfa_state_t *st,
                       i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  I2C_Send7bitAddress(pctx->i2c, pctx->slave_addr, I2C_Direction_Receiver);
  dfa->cs = 4;
}
///////////////////////////////////////////////////////

void
rmb_read_byte_btf(i2c_async_dfa_state_t *st,
                  i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  if (pctx->read_len > 3) {
    *pctx->dst_buff++ = (uint8_t)pctx->i2c->DR;
    --pctx->read_len;
    return;
  }

  I2C_AcknowledgeConfig(pctx->i2c, DISABLE); //Send NACK after last byte received
  *pctx->dst_buff++ = (uint8_t)pctx->i2c->DR; //read N-2
  I2C_GenerateSTOP(pctx->i2c, ENABLE); //generate stop after last byte received
  *pctx->dst_buff++ = (uint8_t)pctx->i2c->DR; //read N-1
  pctx->read_len -= 2;
  dfa->cs = 5;
}
///////////////////////////////////////////////////////

void
rmb_finish(i2c_async_dfa_state_t* st,
           i2c_async_dfa_t *dfa) {
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);

  if (pctx->read_len == 1)
    *pctx->dst_buff++ = (uint8_t)pctx->i2c->DR; //read N-1 and clear rxne flag

  if (pctx->fn_callback != NULL)
    pctx->fn_callback(FC_FINISHED);

  I2C_AcknowledgeConfig(pctx->i2c, ENABLE);
  dfa->cs = DFA_INIT_STATE; //goto init state
  *pctx->ptr_finish_code = FC_FINISHED;
}
///////////////////////////////////////////////////////

void
rmb_addr_received_len1(struct i2c_async_dfa_state* st, struct i2c_async_dfa *dfa) {
  UNUSED(dfa);
  rmb_context_t *pctx = (rmb_context_t*)(st->arg);
  I2C_AcknowledgeConfig(pctx->i2c, DISABLE); //Send NACK after last byte received
  I2C_GenerateSTOP(pctx->i2c, ENABLE); //generate stop after last byte received
}
///////////////////////////////////////////////////////

i2c_async_dfa_t
rmb_dfa_create(uint8_t slaveAddr,
               uint8_t startReg,
               volatile uint8_t *dstBuff,
               uint8_t readLen,
               volatile int8_t *pErr,
               i2c1_pf_callback cb) {
  i2c_async_dfa_t res;
  static rmb_context_t rmbCtx;

  // SB, ADDR, BTF, RxNE, TxE, ERR
  static i2c_async_dfa_state_t statesGE2[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_i2c_err}}, //init. if some interrupt happens here - it's NOT OK!
    //1
    {.actions = {rmb_send_addr_transmitter, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_i2c_err}}, //start sent. send i2c slave addr (transmitter mode)
    //2
    {.actions = {rmb_dfa_err, rmb_send_start_reg, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_i2c_err}}, //addr sent. send read register addr. TxE not set in addr stage
    //3
    {.actions = {rmb_send_addr_receiver, rmb_dfa_err, dfa_nop, rmb_dfa_err, dfa_nop, rmb_i2c_err}}, //start sent. send i2c slave addr (receiver mode)
    //4
    {.actions = {rmb_dfa_err, dfa_nop, rmb_read_byte_btf, dfa_nop, dfa_nop, rmb_i2c_err}}, //addr sent. wait for byte received
    //5
    {.actions = {rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_finish, rmb_dfa_err, rmb_i2c_err}}, //clear flags after NACK and STOP conditions
  };

  // SB, ADDR, BTF, RxNE, TxE, ERR
  static i2c_async_dfa_state_t statesEQ1[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_i2c_err}}, //init. if some interrupt happens here - it's NOT OK!
    //1
    {.actions = {rmb_send_addr_transmitter, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_i2c_err}}, //start sent. send i2c slave addr (transmitter mode)
    //2
    {.actions = {rmb_dfa_err, rmb_send_start_reg, rmb_dfa_err, rmb_dfa_err, rmb_dfa_err, rmb_i2c_err}}, //addr sent. send read register addr. TxE not set in addr stage
    //3
    {.actions = {rmb_send_addr_receiver, rmb_dfa_err, dfa_nop, rmb_dfa_err, dfa_nop, rmb_i2c_err}}, //start sent. send i2c slave addr (receiver mode)
    //4
    {.actions = {rmb_dfa_err, rmb_addr_received_len1, rmb_dfa_err, rmb_finish, dfa_nop, rmb_i2c_err}}, //addr sent. wait for byte received
  };

  rmbCtx.i2c = I2C1;
  rmbCtx.slave_addr = slaveAddr;
  rmbCtx.dst_reg = startReg;
  rmbCtx.dst_buff = dstBuff;
  rmbCtx.read_len = readLen;
  rmbCtx.ptr_finish_code = pErr;
  rmbCtx.fn_callback = cb;
  res.cs = DFA_INIT_STATE;

  i2c_async_dfa_state_t *states = readLen == 1 ? statesEQ1 : statesGE2;
  size_t len = readLen == 1 ? sizeof(statesEQ1) : sizeof(statesGE2);

  for (size_t i = 0; i < len/sizeof(i2c_async_dfa_state_t); ++i) {
    states[i].arg = &rmbCtx;
    states[i].ix = (int32_t)i;
    res.states[i] = states[i];
  }

  return res;
}
///////////////////////////////////////////////////////

void
i2c1_init(void) {
  I2C_InitTypeDef i2c_cfg;
  GPIO_InitTypeDef gpio_cfg;
  m_dfa_i2c1 = pre_init_dfa();

  I2C_Cmd(I2C1, DISABLE);
  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(I2C_RCC_Periph, ENABLE);
  RCC_AHBPeriphClockCmd(I2C_RCC_AHBPort, ENABLE);

  /* Configure I2C pins: SCL and SDA */
  gpio_cfg.GPIO_Pin = I2C_SCL_Pin | I2C_SDA_Pin;
  gpio_cfg.GPIO_Mode = GPIO_Mode_AF;
  gpio_cfg.GPIO_OType = GPIO_OType_OD;
  gpio_cfg.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_cfg.GPIO_Speed = GPIO_Speed_40MHz;

  GPIO_Init(GPIOB, &gpio_cfg);

  GPIO_PinAFConfig(GPIOB, I2C1_SCL_PIN_SOURCE, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, I2C1_SDA_PIN_SOURCE, GPIO_AF_I2C1);

  /* I2C configuration */
  i2c_cfg.I2C_Mode = I2C_Mode_I2C;
  i2c_cfg.I2C_DutyCycle = I2C_DutyCycle_2;
  //  I2C_InitStructure.I2C_OwnAddress1 = MPU6050_DEFAULT_ADDRESS; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
  i2c_cfg.I2C_Ack = I2C_Ack_Enable;
  i2c_cfg.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  i2c_cfg.I2C_ClockSpeed = I2C_Speed;

  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &i2c_cfg);
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  i2c1_interrupts(ENABLE);
}
///////////////////////////////////////////////////////

void
i2c1_interrupts(FunctionalState on) {
  NVIC_InitTypeDef nvicInitCfg;
  /* Enable the I2C1 event Interrupt */
  nvicInitCfg.NVIC_IRQChannel = I2C1_EV_IRQn;
  nvicInitCfg.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInitCfg.NVIC_IRQChannelSubPriority = 0;
  nvicInitCfg.NVIC_IRQChannelCmd = on;
  NVIC_Init(&nvicInitCfg);

  /* Enable the I2C1 error interrupt with higher then event priority */
  nvicInitCfg.NVIC_IRQChannel = I2C1_ER_IRQn;
  nvicInitCfg.NVIC_IRQChannelPreemptionPriority = 0;
  nvicInitCfg.NVIC_IRQChannelSubPriority = 0;
  nvicInitCfg.NVIC_IRQChannelCmd = on;
  NVIC_Init(&nvicInitCfg);
  I2C_ITConfig(I2C1, I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR, on);
}
///////////////////////////////////////////////////////

void
i2c1_read_buff_async(uint8_t slaveAddr,
                     uint8_t startReg,
                     uint8_t *buff,
                     uint8_t len,
                     volatile int8_t *finishCode,
                     i2c1_pf_callback cb) {
  *finishCode = FC_IN_PROGRESS;
  m_dfa_i2c1 = rmb_dfa_create(slaveAddr, startReg, buff, len, finishCode, cb);
  rmb_gen_start(&m_dfa_i2c1.states[0], &m_dfa_i2c1);
}
///////////////////////////////////////////////////////

i2c1_finish_code
i2c1_read_buff_sync(uint8_t slaveAddr,
                    uint8_t startReg,
                    uint8_t *buff,
                    uint8_t len) {
  volatile int8_t finishCode = FC_IN_PROGRESS;
  i2c1_read_buff_async(slaveAddr, startReg, buff, len, &finishCode, NULL);
  while (finishCode == FC_IN_PROGRESS);
  return (i2c1_finish_code)finishCode;
}
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

// write multiple bytes
static i2c_async_dfa_t wmb_dfa_create(uint8_t slaveAddr,
                              uint8_t startReg,
                              volatile uint8_t *srcBuff,
                              uint8_t writeLen,
                              volatile int8_t *pErr,
                              i2c1_pf_callback cb);
static void wmb_gen_start(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void wmb_send_addr(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void wmb_send_start_reg(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void wmb_send_data(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void wmb_finish(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);

static void wmb_dfa_err(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);
static void wmb_i2c_err(i2c_async_dfa_state_t *st, i2c_async_dfa_t *dfa);

static void wmb_i2c_err_Berr(wmb_context_t *ctx);
static void wmb_i2c_err_AF(wmb_context_t *ctx);
static void wmb_i2c_err_Arlo(wmb_context_t *ctx);
static void wmb_i2c_err_ovr(wmb_context_t *ctx);
static void wmb_i2c_err_timeout(wmb_context_t *ctx);

void
wmb_gen_start(i2c_async_dfa_state_t *st,
              i2c_async_dfa_t *dfa) {
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  while (I2C_GetFlagStatus(pctx->i2c, I2C_FLAG_BUSY))
    __NOP();
  dfa->cs = 1;
  I2C_GenerateSTART(pctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void
wmb_send_addr(i2c_async_dfa_state_t *st,
              i2c_async_dfa_t *dfa) {
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  I2C_Send7bitAddress(pctx->i2c, pctx->slave_addr, I2C_Direction_Transmitter);
  dfa->cs = 2;
}
///////////////////////////////////////////////////////

void
wmb_send_start_reg(i2c_async_dfa_state_t *st,
                   i2c_async_dfa_t *dfa) {
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  I2C_SendData(pctx->i2c, pctx->dst_reg);
  dfa->cs = 3;
}
///////////////////////////////////////////////////////

void
wmb_send_data(i2c_async_dfa_state_t *st,
              i2c_async_dfa_t *dfa) {
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  I2C_SendData(pctx->i2c, *pctx->src_buff++);
  if (pctx->write_len-- != 1) {
    dfa->cs = 3;
    return;
  }
  I2C_GenerateSTOP(pctx->i2c, ENABLE);
  dfa->cs = 4;
}
///////////////////////////////////////////////////////

void
wmb_finish(i2c_async_dfa_state_t* st,
           i2c_async_dfa_t *dfa) {
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  if (pctx->fn_callback != NULL)
    pctx->fn_callback(FC_FINISHED);
  dfa->cs = DFA_INIT_STATE; //goto init state
  *pctx->ptr_finish_code = FC_FINISHED;
}
///////////////////////////////////////////////////////

void
wmb_i2c_err_Berr(wmb_context_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void
wmb_i2c_err_AF(wmb_context_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void
wmb_i2c_err_Arlo(wmb_context_t *ctx) {
  UNUSED(ctx);
  //todo reset all slaves
}
///////////////////////////////////////////////////////

void
wmb_i2c_err_ovr(wmb_context_t *ctx) {
  I2C_GenerateSTOP(ctx->i2c, ENABLE);
}
///////////////////////////////////////////////////////

void
wmb_i2c_err_timeout(wmb_context_t *ctx) {
  UNUSED(ctx); //do nothing
}
///////////////////////////////////////////////////////

void
wmb_i2c_err(i2c_async_dfa_state_t* st,
            i2c_async_dfa_t *dfa) {
  volatile uint16_t sr1;
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  sr1 = pctx->i2c->SR1;
  pctx->i2c->SR1 = (sr1 & ~I2C_ERR_MASK);

  if (sr1 & I2C_SR1_BERR)
    wmb_i2c_err_Berr(pctx);
  if (sr1 & I2C_SR1_AF)
    wmb_i2c_err_AF(pctx);
  if (sr1 & I2C_SR1_ARLO)
    wmb_i2c_err_Arlo(pctx);
  if (sr1 & I2C_SR1_OVR)
    wmb_i2c_err_ovr(pctx);
  if (sr1 & I2C_SR1_TIMEOUT)
    wmb_i2c_err_timeout(pctx);
  if (pctx->fn_callback != NULL)
    pctx->fn_callback(FC_I2C_ERR);
  *pctx->ptr_finish_code = FC_I2C_ERR;
  dfa->cs = DFA_INIT_STATE;
}
///////////////////////////////////////////////////////

//todo: we can't be here. SO! just restart device and i2c peripheral
void
wmb_dfa_err(i2c_async_dfa_state_t* st,
            i2c_async_dfa_t *dfa) {
  wmb_context_t *pctx = (wmb_context_t*)(st->arg);
  I2C_Cmd(pctx->i2c, DISABLE);
  I2C_Cmd(pctx->i2c, ENABLE);
  if (pctx->fn_callback != NULL)
    pctx->fn_callback(FC_I2C_ERR);
  *pctx->ptr_finish_code = FC_I2C_ERR;
  dfa->cs = DFA_INIT_STATE;
}
///////////////////////////////////////////////////////

i2c_async_dfa_t
wmb_dfa_create(uint8_t slaveAddr,
               uint8_t startReg,
               volatile uint8_t *srcBuff,
               uint8_t writeLen,
               volatile int8_t *pErr,
               i2c1_pf_callback cb) {
  i2c_async_dfa_t res;
  static wmb_context_t wmbCtx;

  // SB, ADDR, BTF, RxNE, TxE, ERR
  static i2c_async_dfa_state_t states[I2C1_LONGEST_DFA] = {
    //0
    {.actions = {wmb_dfa_err, wmb_dfa_err, wmb_dfa_err, wmb_dfa_err, wmb_dfa_err, wmb_i2c_err}}, //init
    //1
    {.actions = {wmb_send_addr, wmb_dfa_err, wmb_dfa_err, wmb_dfa_err, wmb_dfa_err, wmb_i2c_err}}, //start sent. send i2c slave addr (transmitter mode)
    //2
    {.actions = {wmb_dfa_err, wmb_send_start_reg, wmb_dfa_err, wmb_dfa_err, wmb_dfa_err, wmb_i2c_err}}, //addr sent. send write register addr. TxE not set in addr stage
    //3
    {.actions = {wmb_dfa_err, dfa_nop, wmb_send_data, wmb_dfa_err, dfa_nop, wmb_i2c_err}}, //addr sent. wait for byte transmitted
    //4
    {.actions = {wmb_dfa_err, wmb_dfa_err, wmb_finish, wmb_dfa_err, dfa_nop, wmb_i2c_err}}, //clear flags after NACK and STOP conditions
  };

  wmbCtx.i2c = I2C1;
  wmbCtx.slave_addr = slaveAddr;
  wmbCtx.dst_reg = startReg;
  wmbCtx.src_buff = srcBuff;
  wmbCtx.write_len = writeLen;
  wmbCtx.ptr_finish_code = pErr;
  wmbCtx.fn_callback = cb;
  res.cs = DFA_INIT_STATE;

  for (size_t i = 0; i < sizeof(states)/sizeof(i2c_async_dfa_state_t); ++i) {
    states[i].arg = &wmbCtx;
    states[i].ix = (int32_t)i;
    res.states[i] = states[i];
  }
  return res;
}
///////////////////////////////////////////////////////

void
i2c1_write_buff_async(uint8_t slaveAddr,
                      uint8_t startReg,
                      uint8_t *buff,
                      uint8_t len,
                      volatile int8_t *finishCode,
                      i2c1_pf_callback cb) {
  *finishCode = FC_IN_PROGRESS;
  m_dfa_i2c1 = wmb_dfa_create(slaveAddr, startReg, buff, len, finishCode, cb);
  wmb_gen_start(&m_dfa_i2c1.states[0], &m_dfa_i2c1);
}
///////////////////////////////////////////////////////

i2c1_finish_code
i2c1_write_buff_sync(uint8_t slaveAddr,
                     uint8_t startReg,
                     uint8_t *buff,
                     uint8_t len) {
  volatile int8_t finishCode = FC_IN_PROGRESS;
  i2c1_write_buff_async(slaveAddr, startReg, buff, len, &finishCode, NULL);
  while (finishCode == FC_IN_PROGRESS);
  return (i2c1_finish_code) finishCode;
}
///////////////////////////////////////////////////////

void
i2c1_scan(void (*cb)(uint8_t, uint8_t)) {
  uint8_t count = 0;
  for (uint8_t i = 0x60; i <= 0x61; ++i) {
    uint8_t tmp = 0;
    i2c1_finish_code err = i2c1_read_buff_sync(i << 1, 0, &tmp, 1);
    if (err == FC_FINISHED) {
      cb(i, count++);
    }
  }
}
///////////////////////////////////////////////////////
