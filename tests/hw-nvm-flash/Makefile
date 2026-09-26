# Bench test for core_nvm's flash emulation on Core.ST.L4 / L4.2, Core.ST.W5
# and Core.ST.H5. See README.md.
#
#   make                            # Core.ST.L4 (report over USB CDC)
#   make TILE=Core.ST.L4.2
#   make TILE=Core.ST.H5            # config-h5.json (report over USB CDC)
#   make TILE=Core.ST.W5            # result in g_nvm_result over SWD
#   make TILE=Core.ST.W5 BLE=1      # adds T5 (writes while advertising)
#
# Switching Cores or BLE: `make distclean` first — coregen/ is kept by
# `make clean` and would otherwise carry the previous build's generated init.
#
# BOOTLOADER is deliberately NOT set: "bootloader": "rom" in config.json gives
# the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4
BLE           ?= 0
TILES_ENABLED := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

ifeq ($(BLE),1)
  CONFIG := CONFIG_JSON=$(PROJECT_DIR)/config-ble.json BLE_ENABLED=1
else ifeq ($(TILE),Core.ST.H5)
  CONFIG := CONFIG_JSON=$(PROJECT_DIR)/config-h5.json BLE_ENABLED=0
else
  CONFIG := BLE_ENABLED=0
endif

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) $(CONFIG) V=$(V) $@
