# Hardware test for the Sense.ADC.6 driver, on a Core.ST.L4 host.
#
#   make flash-dfu     # ROM-DFU layout, set by "bootloader": "rom"
#
# BOOTLOADER is deliberately NOT set: the custom-bootloader layout is parked,
# and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4
TILES_ENABLED := 1
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

.PHONY: all clean distclean flash flash-dfu flash-rom size generate
all clean distclean flash flash-dfu flash-rom size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
