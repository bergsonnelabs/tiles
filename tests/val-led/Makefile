# Compile check for the onboard-LED helpers, including the VDD-scaled on-time.
# coregen follows config.json's "core", so to build another Core change it there
# (or build a copy with SDK_ROOT pointed back here).
TILE         ?= Core.ST.L4
TILES_ENABLED := 0
BLE_ENABLED  := 0
BOOTLOADER   := 0
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash flash-dfu size generate
all clean distclean flash flash-dfu size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) BOOTLOADER=$(BOOTLOADER) V=$(V) $@
