# Bench test for core_spi on Core.ST.L4.1 / L4.2 and Core.ST.W5, with one
# jumper from MOSI to MISO. See README.md.
#
#   make                            # Core.ST.L4.1 (report over USB CDC)
#   make TILE=Core.ST.L4.2          # config-l42.json: MISO on pad 18, CS on pad 19
#   make TILE=Core.ST.W5            # config-w5.json: SPI3; result in g_spi_lb over SWD
#
# Switching Cores: `make distclean` first — coregen/ is kept by `make clean`
# and would otherwise carry the previous Core's generated init.
#
# BOOTLOADER is deliberately NOT set: "bootloader": "rom" in the configs gives
# the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4.1
TILES_ENABLED := 1
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

ifeq ($(TILE),Core.ST.L4.2)
  CONFIG := CONFIG_JSON=$(PROJECT_DIR)/config-l42.json EXTRA_CFLAGS="-DLB_MISO=18 -DLB_CS=19"
else ifeq ($(TILE),Core.ST.W5)
  CONFIG := CONFIG_JSON=$(PROJECT_DIR)/config-w5.json EXTRA_CFLAGS="-DLB_MOSI=6 -DLB_MISO=7 -DLB_CS=3 -DLB_SCK=2"
else
  CONFIG :=
endif

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) $(CONFIG) V=$(V) $@
