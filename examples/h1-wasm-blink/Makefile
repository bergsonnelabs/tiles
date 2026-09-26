# h1-wasm-blink — the A4c proof-of-life. Loads a precompiled `.wasm`
# module into WAMR and drives a host-import loop that flashes the
# onboard LED via core_led_heartbeat. Core.ST.H5 only today; Core.ST.W5 gets
# its own example once WAMR is verified here.
#
# Run from this folder: `make`. Flash: `make flash-dfu`.

TILE         := Core.ST.H5
TILES_ENABLED := 0
ROM_DFU      := 1
WAMR_ENABLED := 1
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash flash-dfu size generate
all clean distclean flash flash-dfu size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) ROM_DFU=$(ROM_DFU) WAMR_ENABLED=$(WAMR_ENABLED) V=$(V) $@
