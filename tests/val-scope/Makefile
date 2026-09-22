# The Scope API on every Core. Default is the L4 (USB link); override TILE:
#   make                        # Core.ST.L4.1  USB link
#   make TILE=Core.ST.H5.1      # USB link
#   make TILE=Core.ST.W5        # no link without BLE: calls are inert
#   make TILE=Core.ST.L0.1      # no link: calls are inert
#   make SCOPE_ENABLED=0        # compiled out
TILE         ?= Core.ST.L4.1
TILES_ENABLED := 0
BLE_ENABLED  := 0
BOOTLOADER   := 0
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash flash-dfu size generate
all clean distclean flash flash-dfu size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) $(if $(SCOPE_ENABLED),SCOPE_ENABLED=$(SCOPE_ENABLED)) BOOTLOADER=$(BOOTLOADER) V=$(V) $@
