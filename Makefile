ARM_TOOLCHAIN_PREFIX = arm-none-eabi-
CC=$(ARM_TOOLCHAIN_PREFIX)gcc
AS=$(ARM_TOOLCHAIN_PREFIX)as
LD=$(ARM_TOOLCHAIN_PREFIX)ld
OBJDUMP=$(ARM_TOOLCHAIN_PREFIX)objdump
OBJCOPY=$(ARM_TOOLCHAIN_PREFIX)objcopy
SIZE=$(ARM_TOOLCHAIN_PREFIX)size
LINK=$(CC)

BUILD_DIR=build
OBJ_DIR=$(BUILD_DIR)/obj
BIN_DIR=bin
CMSIS_DIR=CMSIS
STD_PERIPH_DRIVER_DIR=StdPeriph_Driver

LIBS := 
DEFS := -DSTM32L1 -DSTM32L152RCTx -DUSE_STDPERIPH_DRIVER -DSTM32L1XX_MDP -DHSE_VALUE=24000000 
WARN_LEVEL = -Wall -pedantic

PRG = blinker
MMCU = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft 
INCLUDES = -Iinc -I$(CMSIS_DIR)/inc -I$(CMSIS_DIR)/device -I$(STD_PERIPH_DRIVER_DIR)/inc
CFLAGS := $(INCLUDES) $(MMCU) $(DEFS) $(WARN_LEVEL) -fmessage-length=0 -ffunction-sections -MMD -MP

debug: CFLAGS += -O0 -g3
debug: all
release: CFLAGS += -O2
release: all

LDFLAGS = $(MMCU) -Wl,-T,stm32l152rctx.ld -Wl,-Map,$(BIN_DIR)/$(PRG).map -Wl,--gc-sections -Wl,-print-memory-usage 

SRC_C := $(wildcard *.c) $(wildcard src/*.c) $(wildcard $(STD_PERIPH_DRIVER_DIR)/*/*.c) 
SRC_A := $(wildcard startup/*.s)

OBJECTS := $(SRC_C:%.c=$(OBJ_DIR)/%.o)
OBJECTS += $(SRC_A:%.s=$(OBJ_DIR)/%.o)

all: directories $(PRG)

$(PRG): $(BIN_DIR)/$(PRG) $(BIN_DIR)/$(PRG).lst $(BIN_DIR)/$(PRG).bin
	
$(OBJ_DIR)/%.o: %.s
	@mkdir -p $(@D)
	$(AS) $(INCLUDES) $(MMCU) -g -o $@ $^

$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -o $@ -c $<

$(BIN_DIR)/$(PRG): $(OBJECTS)
	@mkdir -p $(@D)
	$(LINK) -o $(BIN_DIR)/$(PRG).elf $^ $(LDFLAGS) $(LIBS)

$(BIN_DIR)/lst: $(BIN_DIR)/$(PRG).lst
$(BIN_DIR)/%.lst: $(BIN_DIR)/%.elf
	$(OBJDUMP) -h -S $< > $@

$(BIN_DIR)/bin: $(BIN_DIR)/$(PRG).bin
$(BIN_DIR)/%.bin: $(BIN_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BIN_DIR)

clean:
	@rm -rf $(BUILD_DIR)/*
	@rm -rf $(BIN_DIR)/*

mrproper:
	@rm -rf $(BUILD_DIR)
	@rm -rf $(BIN_DIR)

program:
	@st-flash write $(BIN_DIR)/$(PRG).bin 0x8000000
