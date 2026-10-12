# Thread joins on boot (config.json "thread"): main.c never mentions core_thread,
# yet OpenThread must be linked and running (core_init() starts it).
#
#   make check-linked   # build, then fail unless OpenThread is in the image
TILE         := Core.ST.W5
TILES_ENABLED := 0
BOOTLOADER   := 0
PROJECT      := $(notdir $(CURDIR))
PROJECT_DIR  := $(abspath $(CURDIR))
SDK_ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
V            ?= 0

.PHONY: all clean distclean flash size generate check-linked
all clean distclean flash size generate:
	$(MAKE) --no-print-directory -C $(SDK_ROOT) TILE=$(TILE) PROJECT=$(PROJECT) PROJECT_DIR=$(PROJECT_DIR) TILES_ENABLED=$(TILES_ENABLED) BOOTLOADER=$(BOOTLOADER) V=$(V) $@

check-linked: all
	@arm-none-eabi-nm build/$(PROJECT).elf | grep -qw otInstanceInitSingle || \
	  { echo "val-thread-auto: OpenThread is not linked: nothing starts Thread" >&2; exit 1; }
	@arm-none-eabi-nm build/$(PROJECT).elf | grep -qw core_thread_start || \
	  { echo "val-thread-auto: core_thread_start is not linked" >&2; exit 1; }
	@echo "val-thread-auto: OpenThread linked, started by core_init()"
