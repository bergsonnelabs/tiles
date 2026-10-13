TILE         := Core.ST.L4
TILES_ENABLED := 1
BLE_ENABLED  := 0
# Flash layout comes from config.json ("bootloader": "rom", the fleet standard:
# app at 0x08000000). The custom-bootloader layout (BOOTLOADER := 1) is parked.
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial size generate
all clean distclean flash flash-dfu flash-rom flash-serial size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
