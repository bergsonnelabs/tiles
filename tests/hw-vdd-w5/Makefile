# Bench check for the WBA55 VREFINT / temperature fix (and the W5 status-LED
# brightness scaling that depends on it). The W5 has no USB, so results are
# left in RAM for the debugger: see main.c for how to read them.
TILE         := Core.ST.W5
TILES_ENABLED := 0
BLE_ENABLED  := 0
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash size generate
all clean distclean flash size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BLE_ENABLED=$(BLE_ENABLED) V=$(V) $@
