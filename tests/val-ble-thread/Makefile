# BLE + Thread in one image on Core.ST.W5: config.json enables both ("ble",
# "thread"), the SDK Makefile reads those keys (BLE_ENABLED, THREAD_ENABLED)
# and links ST's concurrent link layer, the BLE stack and OpenThread.
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
