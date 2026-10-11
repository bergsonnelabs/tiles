# Validation: the USB line reader (core_usb_line_feed / core_usb_poll_line)
# and coregen's Core.USB.line dispatch. USB Cores only:
#   make                        # Core.ST.L4
#   make TILE=Core.ST.H5
# (Core.ST.W5 and Core.ST.L0 have no USB.)
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
