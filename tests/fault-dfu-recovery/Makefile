# Test: HardFault → DFU reboot recovery
# Flash this, wait for the deliberate crash, then verify 1200-touch still works.

TILE         := Core.ST.L4
TILES_ENABLED := 1
ROM_DFU      := 1
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash flash-dfu size generate
all clean distclean flash flash-dfu size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) ROM_DFU=$(ROM_DFU) V=$(V) $@
