# Serial-update flasher blobs (STM32L422 and STM32H523).
#
# Builds the SRAM-resident flashers and regenerates the committed headers the
# app embeds (sdk/hal/hal_serial_update_blob_{l4,h5}.h). Standalone: no
# coregen, no config.json. Run after editing anything under sdk/serial_update/:
#
#   make -C sdk/serial_update          # build both + regenerate the headers
#   make -C sdk/serial_update check    # headers match the sources (CI)
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
GEN       := $(SDK_ROOT)/tools/gen_serial_update_blob.py

COMMON  = -mthumb -Os -std=gnu11 -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns
COMMON += -ffunction-sections -fdata-sections -fno-common
COMMON += -Wall -Wextra -Werror -I. -I$(SDK_ROOT)/sdk/ll

# STM32L422: the protocol defaults (2 KB pages, 8-byte program unit, v1).
CFLAGS_l4 = -mcpu=cortex-m4 -mfloat-abi=soft -DSTM32L422xx $(COMMON)
# STM32H523: 8 KB sectors as pages, 128-bit flash words, READY v2.
CFLAGS_h5 = -mcpu=cortex-m33 -mfloat-abi=soft -DSTM32H523xx $(COMMON) \
            -DSU_MAX_PAYLOAD=8192u -DSU_PROG_UNIT=16u -DSU_PROTOCOL_VERSION=2u

LDFLAGS  = -nostdlib -nostartfiles -Wl,--gc-sections -Wl,--no-warn-rwx-segments

Q = $(if $(V),,@)

.PHONY: all check test clean

all: $(SDK_ROOT)/sdk/hal/hal_serial_update_blob_l4.h $(SDK_ROOT)/sdk/hal/hal_serial_update_blob_h5.h

$(BUILD_DIR)/flasher_%.elf: flasher_%.c su_core.c su_core.h su_protocol.h flasher_%.ld $(wildcard $(SDK_ROOT)/sdk/ll/*.h) | $(BUILD_DIR)
	@echo "  LD    $(notdir $@)"
	$(Q)$(CC) $(CFLAGS_$*) flasher_$*.c su_core.c $(LDFLAGS) -Wl,-T,flasher_$*.ld -o $@
	$(Q)$(SIZE) $@

$(BUILD_DIR)/flasher_%.bin: $(BUILD_DIR)/flasher_%.elf
	$(Q)$(OBJCOPY) -O binary $< $@

$(SDK_ROOT)/sdk/hal/hal_serial_update_blob_%.h: $(BUILD_DIR)/flasher_%.bin $(GEN)
	@echo "  GEN   $(notdir $@)"
	$(Q)$(PYTHON) $(GEN) --chip $* --bin $< --out $@

check:
	$(Q)$(PYTHON) $(GEN) --chip l4 --check --out $(SDK_ROOT)/sdk/hal/hal_serial_update_blob_l4.h
	$(Q)$(PYTHON) $(GEN) --chip h5 --check --out $(SDK_ROOT)/sdk/hal/hal_serial_update_blob_h5.h

test:
	$(Q)$(MAKE) -C test

$(BUILD_DIR):
	@mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)
	$(Q)$(MAKE) -C test clean
