\TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += \
    STM32L1 \
    USE_STDPERIPH_DRIVER \    
    HSE_VALUE=24000000 \
    STM32L1XX_MDP

INCLUDEPATH += \
    inc \
    CMSIS/inc \
    CMSIS/device \
    StdPeriph_Driver/inc

DISTFILES += \
    Makefile \
    startup/stm32l152rctx.s \
    stm32l152rctx.ld

HEADERS += \
    CMSIS/device/stm32l1xx.h \
    CMSIS/device/system_stm32l1xx.h \
    CMSIS/inc/arm_common_tables.h \
    CMSIS/inc/arm_const_structs.h \
    CMSIS/inc/arm_math.h \
    CMSIS/inc/core_cm0.h \
    CMSIS/inc/core_cm0plus.h \
    CMSIS/inc/core_cm3.h \
    CMSIS/inc/core_cm4.h \
    CMSIS/inc/core_cm4_simd.h \
    CMSIS/inc/core_cmFunc.h \
    CMSIS/inc/core_cmInstr.h \
    CMSIS/inc/core_sc000.h \
    CMSIS/inc/core_sc300.h \
    StdPeriph_Driver/inc/misc.h \
    StdPeriph_Driver/inc/stm32l1xx_adc.h \
    StdPeriph_Driver/inc/stm32l1xx_aes.h \
    StdPeriph_Driver/inc/stm32l1xx_comp.h \
    StdPeriph_Driver/inc/stm32l1xx_crc.h \
    StdPeriph_Driver/inc/stm32l1xx_dac.h \
    StdPeriph_Driver/inc/stm32l1xx_dbgmcu.h \
    StdPeriph_Driver/inc/stm32l1xx_dma.h \
    StdPeriph_Driver/inc/stm32l1xx_exti.h \
    StdPeriph_Driver/inc/stm32l1xx_flash.h \
    StdPeriph_Driver/inc/stm32l1xx_fsmc.h \
    StdPeriph_Driver/inc/stm32l1xx_gpio.h \
    StdPeriph_Driver/inc/stm32l1xx_i2c.h \
    StdPeriph_Driver/inc/stm32l1xx_iwdg.h \
    StdPeriph_Driver/inc/stm32l1xx_lcd.h \
    StdPeriph_Driver/inc/stm32l1xx_opamp.h \
    StdPeriph_Driver/inc/stm32l1xx_pwr.h \
    StdPeriph_Driver/inc/stm32l1xx_rcc.h \
    StdPeriph_Driver/inc/stm32l1xx_rtc.h \
    StdPeriph_Driver/inc/stm32l1xx_sdio.h \
    StdPeriph_Driver/inc/stm32l1xx_spi.h \
    StdPeriph_Driver/inc/stm32l1xx_syscfg.h \
    StdPeriph_Driver/inc/stm32l1xx_tim.h \
    StdPeriph_Driver/inc/stm32l1xx_usart.h \
    StdPeriph_Driver/inc/stm32l1xx_wwdg.h \
    inc/camera.h \
    inc/camera_regs.h \
    inc/commons.h \
    inc/modem.h \
    inc/modem_hw.h \
    inc/modem_socket.h \
    inc/i2c1.h

SOURCES += \
    StdPeriph_Driver/src/misc.c \
    StdPeriph_Driver/src/stm32l1xx_adc.c \
    StdPeriph_Driver/src/stm32l1xx_aes.c \
    StdPeriph_Driver/src/stm32l1xx_aes_util.c \
    StdPeriph_Driver/src/stm32l1xx_comp.c \
    StdPeriph_Driver/src/stm32l1xx_crc.c \
    StdPeriph_Driver/src/stm32l1xx_dac.c \
    StdPeriph_Driver/src/stm32l1xx_dbgmcu.c \
    StdPeriph_Driver/src/stm32l1xx_dma.c \
    StdPeriph_Driver/src/stm32l1xx_exti.c \
    StdPeriph_Driver/src/stm32l1xx_flash.c \
    StdPeriph_Driver/src/stm32l1xx_flash_ramfunc.c \
    StdPeriph_Driver/src/stm32l1xx_fsmc.c \
    StdPeriph_Driver/src/stm32l1xx_gpio.c \
    StdPeriph_Driver/src/stm32l1xx_i2c.c \
    StdPeriph_Driver/src/stm32l1xx_iwdg.c \
    StdPeriph_Driver/src/stm32l1xx_lcd.c \
    StdPeriph_Driver/src/stm32l1xx_opamp.c \
    StdPeriph_Driver/src/stm32l1xx_pwr.c \
    StdPeriph_Driver/src/stm32l1xx_rcc.c \
    StdPeriph_Driver/src/stm32l1xx_rtc.c \
    StdPeriph_Driver/src/stm32l1xx_sdio.c \
    StdPeriph_Driver/src/stm32l1xx_spi.c \
    StdPeriph_Driver/src/stm32l1xx_syscfg.c \
    StdPeriph_Driver/src/stm32l1xx_tim.c \
    StdPeriph_Driver/src/stm32l1xx_usart.c \
    StdPeriph_Driver/src/stm32l1xx_wwdg.c \
    src/camera.c \
    src/commons.c \
    src/main.c \
    src/modem.c \
    src/modem_hw.c \
    src/modem_socket.c \
    src/i2c1.c \
    src/system_stm32l1xx.c

