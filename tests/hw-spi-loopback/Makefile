# Bench test for core_spi on Core.ST.L4 / L4.2 and Core.ST.W5, with one
# jumper from MOSI to MISO. See README.md.
#
#   make                            # Core.ST.L4 (report over USB CDC)
#   make TILE=Core.ST.L4.2          # config-l42.json: MISO on pad 18, CS on pad 19
#   make TILE=Core.ST.W5            # config-w5.json: SPI3; result in g_spi_lb over SWD
#   make TWO_TILES=1                # config-2tiles.json: two SPI1 tiles, CS pads 9 and 4 (T7)
#   make TILE=Core.ST.W5 TWO_TILES=1 # config-w5-2tiles.json: two SPI3 tiles, CS pads 3 and 4 (T7)
#   make TILE=Core.ST.W5 DUAL=1      # config-w5-dual.json: SPI3 tile on pad 3 + dual-bus
#                                    # (I2C1 + SPI3) tile on pad 4 (T7 and T8)
#
# Switching Cores or configs: `make distclean` first — coregen/ is kept by `make clean`
# and would otherwise carry the previous Core's generated init.
#
# BOOTLOADER is deliberately NOT set: "bootloader": "rom" in the configs gives
# the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4
TILES_ENABLED := 1
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

# TWO_TILES=1 adds T7: two tiles on the bus, each with its own chip select.
# DUAL=1 (W5 only) makes the second one a dual-bus tile and adds T8.
TWO  := $(filter 1,$(TWO_TILES))
DUAL_ := $(filter 1,$(DUAL))
ifeq ($(TILE),Core.ST.L4.2)
  CFG := config-l42.json
  LB  := -DLB_MISO=18 -DLB_CS=19
else ifeq ($(TILE),Core.ST.W5)
  CFG := config-w5$(if $(DUAL_),-dual,$(if $(TWO),-2tiles)).json
  LB  := -DLB_MOSI=6 -DLB_MISO=7 -DLB_CS=3 -DLB_SCK=2 $(if $(DUAL_)$(TWO),-DLB_CS2=4) $(if $(DUAL_),-DLB_DUAL=1)
else
  CFG := config$(if $(TWO),-2tiles).json
  LB  := $(if $(TWO),-DLB_CS2=4)
endif
ifneq ($(DUAL_),)
  ifneq ($(TILE),Core.ST.W5)
    $(error DUAL=1 is Core.ST.W5 only (its I2C1 pads 10/11 leave pad 4 free for the second CS))
  endif
endif
CONFIG := CONFIG_JSON=$(PROJECT_DIR)/$(CFG) EXTRA_CFLAGS="$(LB)"

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) $(CONFIG) V=$(V) $@
