# Bench test for USB CDC robustness and USB-reachable sleep on Core.ST.L4.
# See README.md; the host side is host_test.py (python3 + pyserial).
#
#   make && make flash-serial && python3 host_test.py
#
# BOOTLOADER is deliberately NOT set: "bootloader": "rom" in config.json gives
# the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4
TILES_ENABLED := 0
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0
# Test-only hooks in hal_usb_cdc.c (Stop / USB-wakeup counters).
EXTRA_CFLAGS  += -DHAL_USB_CDC_TEST_HOOKS

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" $@
