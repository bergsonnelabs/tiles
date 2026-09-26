# Default Core is the W5 — the one this test exists for (see main.c). Build the
# others by overriding TILE:
#   make                        # Core.ST.W5  (no USB, no DFU path)
#   make TILE=Core.ST.L4
#   make TILE=Core.ST.H5
#   make TILE=Core.ST.L0
TILE         ?= Core.ST.W5
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
