# Bench test for core_reset() / core_reset_reason() on a Core.ST.L4 (ROM-DFU
# layout, "bootloader": "rom" in config.json). See main.c.
#   make flash-dfu      # first time
#   make flash-serial   # after that
#   make reset-serial   # the host-side remote reset (tools/serial_update.py --reset)
TILE         := Core.ST.L4
TILES_ENABLED := 0
BLE_ENABLED  := 0
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash flash-dfu flash-serial reset-serial size generate
all clean distclean flash flash-dfu flash-serial reset-serial size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
