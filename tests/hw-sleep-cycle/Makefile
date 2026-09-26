# Bench test for sleep / RTC / backup registers / watchdog-during-sleep on
# Core.ST.L0, Core.ST.L4 / L4.2 and Core.ST.W5. See README.md.
#
#   make                            # Core.ST.L4 (PASS/FAIL over USB CDC)
#   make TILE=Core.ST.L0          # LED + SWD-readable result codes
#   make TILE=Core.ST.W5
#
# Switching Cores: `make distclean` first — coregen/ is kept by `make clean`
# and would otherwise carry the previous Core's generated init.
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

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
