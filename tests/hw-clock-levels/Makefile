# Bench test for the clock levels (low / medium / high / max) on Core.ST.L0,
# Core.ST.L4 / L4.2 and Core.ST.W5. One build per level. See README.md.
#
#   make CLOCK=high                      # Core.ST.L4 (report over USB CDC)
#   make CLOCK=max TILE=Core.ST.L0     # LED + SWD-readable result
#   make CLOCK=low TILE=Core.ST.W5
#
# Each level has its own config-<level>.json. Changing CLOCK or TILE throws
# away coregen/ and build/ first (below), so a build never carries the previous
# level's generated clock setup. Switching by hand: `make distclean`.
#
# BOOTLOADER is deliberately NOT set: "bootloader": "rom" in the configs gives
# the ROM-DFU layout (app @ 0x08000000). The custom-bootloader layout is
# parked, and flashing it onto a ROM-DFU board corrupts the resident app.
TILE          ?= Core.ST.L4
CLOCK         ?= medium
TILES_ENABLED := 0
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

CONFIG := $(PROJECT_DIR)/config-$(CLOCK).json
ifeq ($(wildcard $(CONFIG)),)
  $(error CLOCK=$(CLOCK): no config-$(CLOCK).json. Use CLOCK=low, medium, high or max)
endif

# The generated clock setup depends on the level and the Core: start clean
# when either changed since the last build.
STAMP := $(PROJECT_DIR)/coregen/.level
ifneq ($(shell cat "$(STAMP)" 2>/dev/null),$(CLOCK) $(TILE))
  $(shell rm -rf "$(PROJECT_DIR)/coregen" "$(PROJECT_DIR)/build" && mkdir -p "$(PROJECT_DIR)/coregen" && echo "$(CLOCK) $(TILE)" > "$(STAMP)")
endif

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate
all clean distclean flash flash-dfu flash-rom flash-serial flash-coreprobe size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) CONFIG_JSON=$(CONFIG) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
