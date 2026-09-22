# Starter project for Core.ST.L0.1
#
#   make              # build
#   make flash        # flash over SWD (OpenOCD + ST-Link)
#   make doctor       # check the toolchain, report what's missing
#   make clean
#
# Copy this folder anywhere and rename it — the folder name becomes the
# project name and the binary name. If you copy it outside the SDK checkout,
# point SDK_ROOT at the SDK:
#
#   make SDK_ROOT=/path/to/tiles
#
# Windows: run this from Git Bash or MSYS2, not cmd.exe / PowerShell.

TILE := Core.ST.L0.1

# Set to 1 once config.json declares a "tiles" array.
TILES_ENABLED ?= 0

# 1 = show every command.
V ?= 0

PROJECT     := $(notdir $(CURDIR))
PROJECT_DIR := $(abspath $(CURDIR))

# Defaults to this template's in-repo location (two levels up). Override after
# copying the folder somewhere else.
SDK_ROOT ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)

ifeq ($(wildcard $(SDK_ROOT)/Makefile),)
  $(error SDK not found at "$(SDK_ROOT)" — pass SDK_ROOT=/path/to/tiles, or \
check that the path has no spaces in it (make cannot build from those))
endif

# Bootloader mode is deliberately NOT passed through: config.json's
# "bootloader" key is the single source of truth, and the SDK Makefile reads it.

.PHONY: all clean distclean flash flash-dfu flash-rom size generate doctor
all clean distclean flash flash-dfu flash-rom size generate doctor:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) V=$(V) $@
