# BLE link stress firmware (Slice 1) — GD32F103, GigaDevice SPL, arm-none-eabi-gcc.
#
# Self-contained except for the SPL, which is referenced through GD_SPL (default = the sibling submodule,
# mirroring the repo's existing ../../../runtime-hal coupling). Build runs on the Mac:
#   make            -> build/stress.elf + build/stress.bin
#   make clean
# Override the toolchain prefix or SPL path if they live elsewhere:
#   make CROSS=arm-none-eabi- GD_SPL=/path/to/GD32F10x

CROSS   ?= arm-none-eabi-
CC       = $(CROSS)gcc
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size

GD_SPL  ?= ../../../runtime-hal/third_party/GD32Firmware/GD32F10x
SPL_SRC      = $(GD_SPL)/GD32F10x_standard_peripheral/Source
SPL_INC      = $(GD_SPL)/GD32F10x_standard_peripheral/Include
CMSIS_GD_INC = $(GD_SPL)/CMSIS/GD/GD32F10x/Include
CMSIS_GD_SRC = $(GD_SPL)/CMSIS/GD/GD32F10x/Source
CMSIS_INC    = $(GD_SPL)/CMSIS

BUILD = build

INCLUDES = -I. -I$(SPL_INC) -I$(CMSIS_GD_INC) -I$(CMSIS_INC)
# GD32F103C8 = medium density; USE_STDPERIPH_DRIVER pulls in the SPL via gd32f10x_libopt.h (vendored here).
DEFS = -DGD32F10X_MD -DUSE_STDPERIPH_DRIVER

CFLAGS  = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Os -ffreestanding \
          -ffunction-sections -fdata-sections -Wall -std=gnu11 $(DEFS) $(INCLUDES)
LDFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -T link.ld -nostartfiles \
          -Wl,--gc-sections -Wl,-Map=$(BUILD)/stress.map

# Firmware sources (local) + the needed SPL sources (rcu clock tree, gpio AF, usart, fmc wait states,
# CMSIS system file).
LOCAL_OBJS = $(BUILD)/startup.o $(BUILD)/main.o
SPL_OBJS   = $(BUILD)/gd32f10x_rcu.o $(BUILD)/gd32f10x_gpio.o $(BUILD)/gd32f10x_usart.o \
             $(BUILD)/gd32f10x_fmc.o $(BUILD)/system_gd32f10x.o
OBJS = $(LOCAL_OBJS) $(SPL_OBJS)

all: $(BUILD)/stress.bin

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: $(SPL_SRC)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/system_gd32f10x.o: $(CMSIS_GD_SRC)/system_gd32f10x.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/stress.elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	$(SIZE) $@

$(BUILD)/stress.bin: $(BUILD)/stress.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
