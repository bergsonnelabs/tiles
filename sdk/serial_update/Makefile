# Serial-update flasher blob (STM32L422).
#
# Builds the SRAM-resident flasher and regenerates the committed header the
# app embeds (sdk/hal/hal_serial_update_blob_l4.h). Standalone: no coregen, no
# config.json. Run after editing anything under sdk/serial_update/:
#
#   make -C sdk/serial_update          # build + regenerate the header
#   make -C sdk/serial_update check    # header matches the sources (CI)
#   make -C sdk/serial_update test     # protocol tests on the host (no hardware)
#
# -ffreestanding + -nostdlib: the flasher runs while flash is being erased, so
# it must not call anything that lives there. Any libc / libgcc reference is a
# link error rather than a crash on the bench.

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size
PYTHON ?= python3

SDK_ROOT  := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../..)
BUILD_DIR := build
TARGET    := $(BUILD_DIR)/flasher_l4
HEADER    := $(SDK_ROOT)/sdk/hal/hal_serial_update_blob_l4.h
GEN       := $(SDK_ROOT)/tools/gen_serial_update_blob.py

SOURCES = flasher_l4.c su_core.c

CFLAGS  = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
CFLAGS += -DSTM32L422xx -I. -I$(SDK_ROOT)/sdk/ll
CFLAGS += -Os -std=gnu11 -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns
CFLAGS += -ffunction-sections -fdata-sections -fno-common
CFLAGS += -Wall -Wextra -Werror

LDFLAGS  = -nostdlib -nostartfiles -Wl,--gc-sections -Wl,-T,flasher_l4.ld
LDFLAGS += -Wl,--no-warn-rwx-segments

Q = $(if $(V),,@)

.PHONY: all check test clean

all: $(HEADER)

$(TARGET).elf: $(SOURCES) su_core.h su_protocol.h flasher_l4.ld | $(BUILD_DIR)
	@echo "  LD    $(notdir $@)"
	$(Q)$(CC) $(CFLAGS) $(SOURCES) $(LDFLAGS) -o $@
	$(Q)$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(Q)$(OBJCOPY) -O binary $< $@

$(HEADER): $(TARGET).bin $(GEN)
	@echo "  GEN   $(notdir $@)"
	$(Q)$(PYTHON) $(GEN) --bin $< --out $@

check:
	$(Q)$(PYTHON) $(GEN) --check --out $(HEADER)

test:
	$(Q)$(MAKE) -C test

$(BUILD_DIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)
	$(Q)$(MAKE) -C test clean
