# BLE link stress firmware (Slice 1) - GigaDevice SPL, arm-none-eabi-gcc.
#
# ONE source tree, TWO board wirings, selected by VARIANT. See the variant block at the top of main.c.
#
#   make                    -> bench   (GD32F103 bench master, CC2541 on USART2 PB10/PB11)
#   make VARIANT=offroad    -> offroad (GD32F130 offroad master, CC2541 on USART0 PB6/PB7 AF0)
#   make clean              -> removes BOTH variants' build dirs
#
# Each variant builds into its own dir so the two never share a stale object with the wrong -D:
#   build/stress.elf + build/stress.bin                  (bench)
#   build-offroad/stress.elf + build-offroad/stress.bin  (offroad)
#
# Self-contained except for the SPL, which is referenced through GD_SPL_ROOT (default = the sibling
# submodule, mirroring the repo's existing ../../../runtime-hal coupling). Build runs on the Mac.
# Override the toolchain prefix or SPL root if they live elsewhere:
#   make CROSS=arm-none-eabi- GD_SPL_ROOT=/path/to/GD32Firmware
#
# BENCH SAFETY: the offroad variant drives PB6/PB7 as USART0. On a BENCH F130 those pins are the
# hardware-I2C0 IMU bus. Do not flash the offroad image to a bench board.

CROSS   ?= arm-none-eabi-
CC       = $(CROSS)gcc
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size

VARIANT ?= bench

GD_SPL_ROOT ?= ../../../runtime-hal/third_party/GD32Firmware

ifeq ($(VARIANT),offroad)
  # GD32F130: GD32F1x0 family, Cortex-M3, 64 KiB flash / 8 KiB SRAM.
  FAMILY   = GD32F1x0
  LC       = gd32f1x0
  BUILD    = build-offroad
  # GD32F130_150 selects the device category (gd32f1x0.h #errors without it).
  DEFS     = -DGD32F130_150 -DUSE_STDPERIPH_DRIVER -DBOARD=1
  # F1x0's system file calls nvic_vector_table_set, which lives in the misc unit. The F10x one does
  # not, so misc stays out of the bench link rather than being added to both "for symmetry".
  EXTRA_SPL_OBJS = $(BUILD)/$(LC)_misc.o
  # The vendored GD32F1x0/CMSIS ships only core_cm3.h, but that file #includes core_cmInstr.h and
  # core_cmFunc.h, which upstream left out of this family's tree. Those two are GENERIC ARM CMSIS
  # headers, not GigaDevice ones, and the proof they are interchangeable here is that GD32F1x0's and
  # GD32F10x's core_cm3.h are BYTE-IDENTICAL (CMSIS 3.20) - the same file asking for the same siblings.
  # So the F10x CMSIS dir is added as a fallback include, AFTER the family's own, to supply just those
  # two. This is a gap in the vendored SPL, not a cross-family dependency of this firmware.
  EXTRA_INCLUDES = -I$(GD_SPL_ROOT)/GD32F10x/CMSIS
else ifeq ($(VARIANT),bench)
  # GD32F103C8: GD32F10x medium density, Cortex-M3, 64 KiB flash / 20 KiB SRAM.
  FAMILY   = GD32F10x
  LC       = gd32f10x
  BUILD    = build
  DEFS     = -DGD32F10X_MD -DUSE_STDPERIPH_DRIVER -DBOARD=0
else
  $(error VARIANT must be `bench` or `offroad` (got `$(VARIANT)`))
endif

GD_SPL       = $(GD_SPL_ROOT)/$(FAMILY)
SPL_SRC      = $(GD_SPL)/$(FAMILY)_standard_peripheral/Source
SPL_INC      = $(GD_SPL)/$(FAMILY)_standard_peripheral/Include
CMSIS_GD_INC = $(GD_SPL)/CMSIS/GD/$(FAMILY)/Include
CMSIS_GD_SRC = $(GD_SPL)/CMSIS/GD/$(FAMILY)/Source
CMSIS_INC    = $(GD_SPL)/CMSIS

INCLUDES = -I. -I$(SPL_INC) -I$(CMSIS_GD_INC) -I$(CMSIS_INC) $(EXTRA_INCLUDES)

# Both parts are Cortex-M3 (GD32F1x0 is M3, not M0 - its CMSIS pulls core_cm3.h), so the machine flags
# are shared and DWT CYCCNT is available for the delay loops on both.
CFLAGS  = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Os -ffreestanding \
          -ffunction-sections -fdata-sections -Wall -std=gnu11 $(DEFS) $(INCLUDES)
LDFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -T $(BUILD)/link.ld -nostartfiles \
          -Wl,--gc-sections -Wl,-Map=$(BUILD)/stress.map

# Firmware sources (local) + the needed SPL sources (rcu clock tree, gpio AF, usart, fmc wait states,
# CMSIS system file, plus any per-family extras).
LOCAL_OBJS = $(BUILD)/startup.o $(BUILD)/main.o
SPL_OBJS   = $(BUILD)/$(LC)_rcu.o $(BUILD)/$(LC)_gpio.o $(BUILD)/$(LC)_usart.o \
             $(BUILD)/$(LC)_fmc.o $(BUILD)/system_$(LC).o $(EXTRA_SPL_OBJS)
OBJS = $(LOCAL_OBJS) $(SPL_OBJS)

all: $(BUILD)/stress.bin

$(BUILD):
	mkdir -p $(BUILD)

# The linker script is preprocessed with the SAME -DBOARD as the C, so the SRAM length and the
# firmware's wiring can never disagree about which board this image is for.
$(BUILD)/link.ld: link.ld.in | $(BUILD)
	$(CC) -E -P -x c $(DEFS) $< -o $@

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: $(SPL_SRC)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/system_$(LC).o: $(CMSIS_GD_SRC)/system_$(LC).c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/stress.elf: $(OBJS) $(BUILD)/link.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	$(SIZE) $@

$(BUILD)/stress.bin: $(BUILD)/stress.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf build build-offroad

.PHONY: all clean
