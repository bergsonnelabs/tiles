# Thread (OpenThread) on Core.ST.W5: config.json enables it ("thread"), the SDK
# Makefile reads that key (THREAD_ENABLED) and links OpenThread in.
TILE         := Core.ST.W5
TILES_ENABLED := 0
BOOTLOADER   := 0
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash size generate
all clean distclean flash size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BOOTLOADER=$(BOOTLOADER) V=$(V) $@
