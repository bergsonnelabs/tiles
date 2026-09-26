# Quiet SWD bench image: no self-reset, no watchdog, heartbeat + boot counter
# readable over SWD, USB CDC and serial update kept. See main.c.
#
#   make && make flash-serial        # or over SWD, see main.c
#   python3 swd_attach.py            # hot-attach series through the CoreProbe
#
# BOOTLOADER is deliberately NOT set: "bootloader": "rom" in config.json gives
# the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.H5
TILES_ENABLED := 0
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
