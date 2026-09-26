# Bench test for core_nvm on the Core.ST.L0's data EEPROM (STM32L011).
# One build per clock level: `low` runs in voltage range 3, where the EEPROM
# can't be programmed, so it exercises core_nvm's step up to range 2; `max`
# is what firmware/sense-adc-6 runs at. Reading the result: main.c.
#
#   make CLOCK=low                       # default
#   make CLOCK=max
#
# Changing CLOCK or TILE throws away coregen/ and build/ first (below).
# The L0 has no USB: flash it over SWD (app at 0x08000000). Never build this
# with BOOTLOADER := 1.

TILE          ?= Core.ST.L0
CLOCK         ?= low
TILES_ENABLED := 0
BLE_ENABLED   := 0
PROJECT       := $(notdir $(CURDIR))
PROJECT_DIR   := $(abspath $(CURDIR))
SDK_ROOT      := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V             ?= 0

CONFIG := $(PROJECT_DIR)/config-$(CLOCK).json
ifeq ($(wildcard $(CONFIG)),)
  $(error CLOCK=$(CLOCK): no config-$(CLOCK).json. Use CLOCK=low or max)
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
