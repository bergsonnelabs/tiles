# Host side of the l0-adc-mux bring-up: a Core.ST.L4 reads a Core.ST.L0
# running projects/l0-adc-mux over I2C1 (pads 4/5) and reports over USB-CDC.
#
#   make flash-dfu     # ROM-DFU layout, set by "bootloader": "rom" in config.json
#
# BOOTLOADER is deliberately NOT set here. The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4
TILES_ENABLED := 0
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

.PHONY: all clean distclean flash flash-dfu flash-rom size generate
all clean distclean flash flash-dfu flash-rom size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
