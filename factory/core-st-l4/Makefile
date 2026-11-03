# Core.ST.L4 factory image: the firmware every new Core.ST.L4 ships with.
# The TileJig writes it into blank Cores over the ROM bootloader (I2C); see
# README.md here for what it does and how a release is cut.
#
#   make              # build (writes build_info.h with the SDK commit)
#   make flash-serial # flash over USB serial
#   make flash-dfu    # flash over USB DFU

TILE := Core.ST.L4

# Set to 1 once config.json declares a "tiles" array.
TILES_ENABLED ?= 0

# 1 = show every command.
V ?= 0

PROJECT     := $(notdir $(CURDIR))
PROJECT_DIR := $(abspath $(CURDIR))

# Defaults to this template's in-repo location (two levels up). Override after
# copying the folder somewhere else.
SDK_ROOT ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)

# The SDK commit this image is built from, shown in its USB banner. A release
# is built from a clean checkout at its tag, so this reads the tag; "-dirty"
# means uncommitted changes went in. Rewritten only when it changes.
FACTORY_SDK := $(shell git -C $(SDK_ROOT) describe --tags --always --dirty --abbrev=12 2>/dev/null || echo unknown)
$(shell printf '#define FACTORY_SDK "%s"\n' '$(FACTORY_SDK)' > build_info.h.tmp; \
        cmp -s build_info.h.tmp build_info.h || mv build_info.h.tmp build_info.h; rm -f build_info.h.tmp)

ifeq ($(wildcard $(SDK_ROOT)/Makefile),)
  $(error SDK not found at "$(SDK_ROOT)" — pass SDK_ROOT=/path/to/tiles, or \
check that the path has no spaces in it (make cannot build from those))
endif

# Bootloader mode is deliberately NOT passed through: config.json's
# "bootloader" key is the single source of truth, and the SDK Makefile reads it.

.PHONY: all clean distclean flash flash-dfu flash-rom flash-serial reset-serial size generate doctor
all clean distclean flash flash-dfu flash-rom flash-serial reset-serial size generate doctor:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) V=$(V) $@
