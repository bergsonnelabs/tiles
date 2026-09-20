# Core Firmware SDK — Top-level Makefile
#
# Usage (from SDK root, building an example):
#   make                          # Build blink for Core.ST.L4.2 (default)
#   make TILE=Core.ST.L4.2 PROJECT=blink
#   make flash                    # Flash via SWD (OpenOCD / CubeProgrammer)
#   make flash-dfu                # Flash over USB DFU
#   make generate                 # Run coregen only (no compile)
#   make doctor                   # Check the toolchain, report what's missing
#   make clean
#
# Usage (from your own project folder):
#   cd my-firmware && make        # Uses per-project Makefile which calls back here
#
# Usage (external project, explicit):
#   make TILE=Core.ST.W5 PROJECT=my-firmware PROJECT_DIR=/path/to/my-firmware
#
# Windows: run from Git Bash / MSYS2 (this Makefile uses a POSIX shell — `[`,
#   `mkdir -p`, `rm -rf`, `ls`, …). The blocks below force sh.exe and pick a
#   working Python launcher automatically. Override with `make PYTHON=…` if
#   auto-detection guesses wrong. `make doctor` checks the whole toolchain and
#   prints what is missing.

# ---- SDK root (absolute path to this file's directory) ----

SDK_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# ---- Shell (Windows) ----
# GNU make on Windows defaults SHELL to cmd.exe, which lacks the POSIX builtins
# this Makefile relies on (`[`, `mkdir -p`, `rm -rf`, `ls`, `&&`/`||`). Force a
# POSIX shell — Git for Windows / MSYS2 provide sh.exe.
# ($(OS) is set to Windows_NT even inside Git-Bash/MSYS2, so this detects there.)
ifeq ($(OS),Windows_NT)
  SHELL       := sh.exe
  .SHELLFLAGS := -c
  # If sh.exe isn't on PATH, make falls back to cmd.exe *per recipe line*,
  # producing cryptic "'[' is not recognized" / "-p already exists" cascades.
  # Probe for a working POSIX shell and fail loud with instructions.
  #
  # The probe must use syntax cmd.exe CANNOT run: `echo ok` succeeds in cmd
  # too, so an echo-only probe is a false green that never fires. `[ -d . ]`
  # is a POSIX builtin — cmd answers "'[' is not recognized" and $(shell …)
  # comes back empty.
  ifneq ($(shell [ -d . ] && echo posix_shell_ok),posix_shell_ok)
    $(error No POSIX shell found: sh.exe is not on PATH. Run this build from \
"Git Bash" or MSYS2 (Start menu -> Git Bash), or install one with `winget install Git.Git` \
and add its usr\bin to PATH. See the Windows note at the top of this Makefile.)
  endif
endif

# ---- Host OS ----
# Drives the serial-port glob, the stty flag, and the STM32CubeProgrammer
# default below. Windows is decided by $(OS) rather than uname, because uname
# under MSYS2 reports MINGW64_NT-10.0-… / MSYS_NT-… (and under Cygwin
# CYGWIN_NT-…) — a moving target that an exact match would keep missing.
_UNAME_S := $(shell uname -s 2>/dev/null || echo unknown)
ifeq ($(OS),Windows_NT)
  HOST_OS := windows
else ifeq ($(_UNAME_S),Darwin)
  HOST_OS := macos
else ifeq ($(_UNAME_S),Linux)
  HOST_OS := linux
else
  HOST_OS := unknown
endif

# ---- Paths with spaces are not buildable ----
# Make cannot represent a space inside a target or prerequisite name — it is
# the separator. A project under e.g. "C:/Users/First Last/proj" (or
# "~/My Documents/…") therefore breaks at the object-file rules no matter how
# carefully the recipes quote, and the failure surfaces as a baffling "No rule
# to make target 'Last/main.c'". Refuse up front with the actual remedy.
# ($(SDK_DIR) is checked here; PROJECT_DIR is checked once it is resolved.)
ifneq ($(words $(SDK_DIR)),1)
  $(error The SDK path contains a space: "$(SDK_DIR)". GNU make cannot build \
from a path with spaces. Move the SDK somewhere without spaces (e.g. C:/dev/tiles \
on Windows, ~/src/tiles on macOS/Linux))
endif

# ---- Python interpreter ----
# On Windows, bare `python3` usually resolves to the Microsoft Store alias stub
# (frequently blocked by org policy — "this app has been blocked by your admin");
# the `py` launcher is the reliable invocation there, while MSYS2-native Python
# only has `python3`, and some Windows installs only put `python` on PATH.
# Probe each candidate for a Python 3 that actually RUNS (the Store stub exits
# non-zero, so `-c ""` filters it out) and take the first that works. Override
# explicitly with `make PYTHON=…`.
# (`ifndef` so an explicit PYTHON from the command line or the environment skips
# the probe entirely; a plain `:=` would still be overridden, but would pay for
# a subprocess on every build.)
ifndef PYTHON
  PYTHON := $(shell for p in python3 "py -3" python; do \
              $$p -c "import sys; sys.exit(0 if sys.version_info[0]==3 else 1)" >/dev/null 2>&1 \
                && { echo "$$p"; break; }; \
            done)
endif
ifeq ($(strip $(PYTHON)),)
  # No working interpreter. Don't limp on with an empty PYTHON — coregen would
  # fail as a bare `"…/coregen.py" is not executable`, which reads like an SDK
  # bug rather than a missing prerequisite.
  $(error No Python 3 found (tried python3, py -3, python). Install Python 3 \
and make sure it is on PATH — Windows: `winget install Python.Python.3.12` \
(tick "Add python.exe to PATH"), macOS: `brew install python`, Debian/Ubuntu: \
`apt install python3`. On Windows, a bare `python3` that opens the Microsoft \
Store is the alias stub, not a real install. Override with `make PYTHON=…`)
endif

# Windows Python encodes stdout with the legacy ANSI code page (cp1252) whenever
# stdout is not a real console — which is exactly what it is under make. A single
# non-ASCII character in a tool's progress output (coregen prints "→", validate
# prints "✓", driver briefs are full of "µ" and "—") then raises
# UnicodeEncodeError and kills the build. Every tools/*.py entry point forces
# UTF-8 on its own streams; exporting this covers the inline `$(PYTHON) -c` calls
# and anything added later that forgets to.
export PYTHONIOENCODING := utf-8

# ---- Verbosity ----
# Default: quiet (shows GEN, LD, BIN/HEX, size). Use V=1 for full output.

V     ?= 0
ifeq ($(V),0)
  Q              := @
  LOG            := @:
  COREGEN_QUIET  := > /dev/null
else
  Q              :=
  LOG            := @echo
  COREGEN_QUIET  :=
endif

# ---- Configuration ----

TILE    ?= Core.ST.L4.2
PROJECT ?= blink

# ---- Public Core name aliases ----
# The vendor-segmented public names (Core.ST.<family>.<n>) resolve onto the
# canonical, DB-synced definition stems. Either form is accepted as TILE; the
# alias rewrites to the stem so the JSON lookup + MCU mapping below stay
# unchanged. Each public name maps to the most-complete definition for its MCU.
ifeq ($(TILE),Core.ST.L4.1)
  override TILE := Core-ST-L4-1-b
else ifeq ($(TILE),Core.ST.L4.2)
  override TILE := Core-ST-L4-2-a
else ifeq ($(TILE),Core.ST.H5.1)
  override TILE := Core-ST-H5-1-a
else ifeq ($(TILE),Core.ST.L0.1)
  override TILE := Core-ST-L0-1-a
else ifeq ($(TILE),Core.ST.W5)
  override TILE := Core-ST-W5-b
endif

# Project directory — override to point at any folder outside the SDK
PROJECT_DIR ?= $(SDK_DIR)examples/$(PROJECT)

# Same space rule as the SDK path above — this is the one that actually bites on
# Windows, where the natural home for a project is "C:/Users/First Last/…".
ifneq ($(words $(PROJECT_DIR)),1)
  $(error The project path contains a space: "$(PROJECT_DIR)". GNU make cannot \
build from a path with spaces. Move the project somewhere without spaces \
(e.g. C:/dev/my-firmware))
endif

# Toolchain
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size
GDB     = $(PREFIX)gdb

# Coregen
COREGEN      = $(PYTHON) "$(SDK_DIR)tools/coregen/coregen.py"
TILE_JSON    = $(SDK_DIR)definitions/$(TILE).json
CONFIG_JSON ?= $(PROJECT_DIR)/config.json
GEN_DIR      ?= $(PROJECT_DIR)/coregen

# Space/backslash-safe existence test for config.json. $(wildcard) treats its
# argument as a space-separated pattern list and mishandles backslashes, so a
# project under e.g. "C:/Users/First Last/proj" silently misses config.json and
# coregen runs config-less (no core.h / core_config.h / core_init.*). A shell
# `test -f` with the path quoted is robust to both. (Needs a POSIX shell, which
# the SHELL block above guarantees on Windows.)
CONFIG_FOUND := $(shell [ -f "$(CONFIG_JSON)" ] && echo 1)

# ---- Tile → MCU mapping ----
# coregen generates the headers; the Makefile still needs to know
# CPU architecture and linker script for compiler flags.

ifeq ($(TILE),$(filter $(TILE),Core-ST-L4-1-a Core-ST-L4-1-b Core-ST-L4-2-a))
  MCU_FAMILY  = stm32l4xx
  MCU_PART    = STM32L422xx
  PROBE_RS_CHIP = STM32L422KB
  CPU         = cortex-m4
  FPU         = fpv4-sp-d16
  FLOAT_ABI   = hard
  LDSCRIPT    = $(SDK_DIR)sdk/device/stm32l422tb.ld
  STARTUP     = $(SDK_DIR)sdk/device/stm32l4xx/startup_stm32l422xx.s
  OPENOCD_CFG = $(SDK_DIR)sdk/debug/stm32l4.cfg
else ifeq ($(TILE),Core-ST-L0-1-a)
  MCU_FAMILY  = stm32l0xx
  MCU_PART    = STM32L011xx
  PROBE_RS_CHIP = STM32L011K4
  CPU         = cortex-m0plus
  LDSCRIPT    = $(SDK_DIR)sdk/device/stm32l011e4.ld
  STARTUP     = $(SDK_DIR)sdk/device/stm32l0xx/startup_stm32l011xx.s
  OPENOCD_CFG = $(SDK_DIR)sdk/debug/stm32l0.cfg
else ifeq ($(TILE),Core-ST-W5-b)
  MCU_FAMILY  = stm32wbaxx
  MCU_PART    = STM32WBA55xx
  PROBE_RS_CHIP = STM32WBA55CG
  CPU         = cortex-m33
  FPU         = fpv5-sp-d16
  FLOAT_ABI   = hard
  LDSCRIPT    = $(SDK_DIR)sdk/device/stm32wba55hg.ld
  STARTUP     = $(SDK_DIR)sdk/device/stm32wbaxx/startup_stm32wba55xx.s
  OPENOCD_CFG = $(SDK_DIR)sdk/debug/stm32wba.cfg
  FLASH_TOOL  = cubeprog
else ifeq ($(TILE),Core-ST-H5-1-a)
  MCU_FAMILY  = stm32h5xx
  MCU_PART    = STM32H523xx
  PROBE_RS_CHIP = STM32H523CE
  CPU         = cortex-m33
  FPU         = fpv5-sp-d16
  FLOAT_ABI   = hard
  LDSCRIPT    = $(SDK_DIR)sdk/device/stm32h523he.ld
  STARTUP     = $(SDK_DIR)sdk/device/stm32h5xx/startup_stm32h523xx.s
  OPENOCD_CFG = $(SDK_DIR)sdk/debug/stm32h5.cfg
else
  $(error Unknown TILE: $(TILE). Supported: Core-ST-L0-1-a, Core-ST-L4-1-a, Core-ST-L4-1-b, Core-ST-L4-2-a, Core-ST-W5-b, Core-ST-H5-1-a — or public names Core.ST.L0.1 / Core.ST.L4.1 / Core.ST.L4.2 / Core.ST.W5 / Core.ST.H5.1)
endif

# ---- Bootloader support ----
# Bootloader mode is read from config.json: "bootloader": "custom"|"rom"|"none"
#
#   "custom" — Custom DFU 1.1 bootloader at 0x08000000, app at 0x08002000.
#   "rom"    — ROM DfuSe bootloader (0483:DF11), app at 0x08000000.
#   "none"   — No DFU support. Flash via SWD or BOOT0.
#
# The mode maps to BOOTLOADER/ROM_DFU flags below. Command-line overrides
# still work (e.g. make BOOTLOADER=1) because ?= defers to explicit values.

_BOOT_MODE := $(shell $(PYTHON) -c "import json; print(json.load(open('$(CONFIG_JSON)')).get('bootloader','none'))" 2>/dev/null || echo none)

ifeq ($(_BOOT_MODE),custom)
  BOOTLOADER ?= 1
  ROM_DFU    ?= 0
else ifeq ($(_BOOT_MODE),rom)
  BOOTLOADER ?= 0
  ROM_DFU    ?= 1
else
  BOOTLOADER ?= 0
  ROM_DFU    ?= 0
endif

ifeq ($(BOOTLOADER),1)
  ifeq ($(ROM_DFU),1)
    $(error BOOTLOADER=1 and ROM_DFU=1 are mutually exclusive)
  endif
  # The custom-bootloader layout (app @ 0x08002000) is PARKED — the fleet runs
  # ROM-DFU layout (app @ 0x08000000; Studio's default, and where the L4
  # strike→ROM-DFU brick recovery lives). Flashing a BOOTLOADER=1 image onto a
  # ROM-DFU board corrupts the resident app mid-flash (see 2026-07-15 incident).
  # The mode is kept for future use behind an explicit acknowledgment.
  ifneq ($(CUSTOM_BOOTLOADER_ACK),1)
    $(error BOOTLOADER=1 (custom-bootloader layout) is parked — the fleet uses \
ROM-DFU layout. Use "bootloader": "rom" in config.json (or ROM_DFU=1). If this \
board really has the custom bootloader at 0x08000000, re-run with \
CUSTOM_BOOTLOADER_ACK=1)
  endif
  APP_ADDR   = 0x08002000
  APP_OFFSET = $(APP_ADDR)UL
  ifeq ($(TILE),$(filter $(TILE),Core-ST-L4-1-a Core-ST-L4-1-b Core-ST-L4-2-a))
    LDSCRIPT = $(SDK_DIR)sdk/device/stm32l422tb_app.ld
  endif
  ifeq ($(TILE),Core-ST-H5-1-a)
    LDSCRIPT = $(SDK_DIR)sdk/device/stm32h523he_app.ld
  endif
endif

ifeq ($(ROM_DFU),1)
  ifeq ($(TILE),$(filter $(TILE),Core-ST-L4-1-a Core-ST-L4-1-b Core-ST-L4-2-a))
    LDSCRIPT = $(SDK_DIR)sdk/device/stm32l422tb_romdfu.ld
  endif
  # Core.ST.H5: standard linker script already has noinit reservation and
  # app at 0x08000000 — no separate ROM DFU linker script needed.
endif

# ---- Paths ----

BUILD_DIR ?= $(PROJECT_DIR)/build
TARGET    = $(BUILD_DIR)/$(PROJECT)

# STM32CubeProgrammer CLI — used for tiles where OpenOCD lacks support (Core.ST.W5 / WBA55).
# Installed in a different place on each host; override on the command line if
# yours lives elsewhere. (The Windows default is the installer's default path —
# quoted at every use site because "Program Files" contains a space.)
ifeq ($(HOST_OS),windows)
  STM32_PROG_CLI ?= /c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe
else ifeq ($(HOST_OS),linux)
  STM32_PROG_CLI ?= /opt/st/stm32cubeprog/bin/STM32_Programmer_CLI
else
  STM32_PROG_CLI ?= /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI
endif

# Default flash tool is openocd; Core.ST.W5 overrides to cubeprog above.
FLASH_TOOL ?= openocd

# ---- Serial port (1200-baud touch) ----
# The DFU trigger opens the Core's USB-CDC port at 1200 baud and drops DTR.
# Where that port lives and how stty addresses it is host-specific:
#   macOS  /dev/tty.usbmodem*   stty -f <dev>
#   Linux  /dev/ttyACM*         stty -F <dev>
#   Windows COM<n>              no /dev entry at all — see flash-dfu below
# SERIAL_PORT can be set explicitly (`make flash-dfu SERIAL_PORT=COM5`) to skip
# detection; on Windows it is the only way to get an automatic touch.
ifeq ($(HOST_OS),macos)
  SERIAL_GLOB := /dev/tty.usbmodem*
  STTY_DEV    := -f
else ifeq ($(HOST_OS),linux)
  SERIAL_GLOB := /dev/ttyACM*
  STTY_DEV    := -F
else
  SERIAL_GLOB :=
  STTY_DEV    := -F
endif

# ---- Extract SYSCLK for SWO baud rate calc ----
SYSCLK_MHZ = $(shell $(PYTHON) -c "\
import json; \
level = json.load(open('$(CONFIG_JSON)'))['clock']; \
configs = json.load(open('$(TILE_JSON)')).get('clock',{}).get('configurations',[]); \
print(next((c['sysclk_mhz'] for c in configs if c['name']==level), 80))" 2>/dev/null || echo 80)
SYSCLK_HZ  = $(shell echo $$(($(SYSCLK_MHZ) * 1000000)))

# ---- Sources ----

# Every .c in the project directory — lets a project split into modules
# (e.g. ring_ble.c, ring_hw.c), not just main.c. Generated sources live in
# coregen/ (a subdirectory, not matched here) and are added via GEN_SOURCES.
# Use a quoted shell glob, not $(wildcard): make's wildcard splits on spaces and
# mishandles backslashes, so a project under e.g. "C:/Users/First Last/proj"
# would silently match zero sources and main.c would never compile.
C_SOURCES   := $(shell ls "$(PROJECT_DIR)"/*.c 2>/dev/null)
ASM_SOURCES = $(STARTUP)

# ---- Compiler flags ----

CPU_FLAGS = -mcpu=$(CPU) -mthumb
ifdef FPU
  CPU_FLAGS += -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)
endif

CFLAGS  = $(CPU_FLAGS)
CFLAGS += -Wall -Wextra -Wshadow -Wdouble-promotion
CFLAGS += -fdata-sections -ffunction-sections -fno-common
CFLAGS += -std=c17
CFLAGS += -D$(MCU_PART)
CFLAGS += -I$(GEN_DIR)
CFLAGS += -I$(PROJECT_DIR)
CFLAGS += -I$(SDK_DIR)sdk/ll
CFLAGS += -I$(SDK_DIR)sdk/hal
CFLAGS += -I$(SDK_DIR)sdk/tal
CFLAGS += -I$(SDK_DIR)sdk/core
CFLAGS += -Og -g3
CFLAGS += $(EXTRA_CFLAGS)   # per-project variant flags (e.g. -DRING_VARIANT_V1)

ifdef APP_OFFSET
  CFLAGS += -DAPP_OFFSET=$(APP_OFFSET)
endif
ifeq ($(ROM_DFU),1)
  CFLAGS += -DROM_DFU
endif

# ---- Tile driver support (optional) ----
TILES_ENABLED ?= 0
ifeq ($(TILES_ENABLED),1)
  # Plain (error-surfacing) include for build goals; skipped for clean/distclean
  # so those never trigger coregen. With `-include`, a coregen failure is
  # silently swallowed and resurfaces as a cryptic "No rule to make target
  # core_drivers.mk" — plain `include` stops the build with coregen's real error
  # (which now goes to stderr, so V=0's `>/dev/null` can't hide it either).
  ifeq (,$(filter clean distclean,$(MAKECMDGOALS)))
    include $(GEN_DIR)/core_drivers.mk
  endif
  CFLAGS += -I"$(SDK_DIR)" -I"$(SDK_DIR)drivers" -I"$(SDK_DIR)hal"
  TILES_SOURCES =
  ifdef TILES_DRIVERS
    TILES_SOURCES += $(foreach drv,$(TILES_DRIVERS),$(SDK_DIR)drivers/$(drv).c)
  endif
endif

# ---- WAMR runtime support (optional; A4c) ----
# Opt-in via `WAMR_ENABLED=1`. When enabled, links the WAMR
# interpreter into the firmware so a user's DSL-compiled `.wasm`
# blob can execute on-device. Gated only to Cortex-M33 targets
# (Core.ST.H5, Core.ST.W5) — Core.ST.L4's M4 footprint is over budget per the
# A4b spike and routes through a separate interpreter path.
WAMR_ENABLED ?= 0
ifeq ($(WAMR_ENABLED),1)
  ifneq ($(CPU),cortex-m33)
    $(error WAMR_ENABLED=1 is only supported on Cortex-M33 Cores (Core.ST.H5.1, Core.ST.W5))
  endif
  include $(SDK_DIR)sdk/wamr/wamr.mk
endif

# ---- BLE support (Core.ST.W5 only) ----
#
# Driven from config.json, the same way "bootloader" is:
#
#   "ble": { "enabled": true }
#
# This is what lets Studio (and the cloud build service) turn BLE on for a
# project: they only ever write config.json, and previously BLE_ENABLED was a
# Makefile-only flag, so a BLE project simply could not be built through
# Studio — not even in escape-to-C. `?=` keeps `make BLE_ENABLED=1` working as
# an override for local one-offs.
_BLE_CFG := $(shell $(PYTHON) -c "import json; c=json.load(open('$(CONFIG_JSON)')); print(1 if (c.get('ble') or {}).get('enabled') else 0)" 2>/dev/null || echo 0)
BLE_ENABLED ?= $(_BLE_CFG)
# coregen emits ble_contract.{h,c} whenever config.json enables BLE — the file
# carries the radio settings (TX power, advertising interval, pairing) even when
# no GATT contract is declared, and core_init() calls into it. Keyed off the
# CONFIG value rather than BLE_ENABLED, so a command-line `make BLE_ENABLED=1`
# on a project whose config says nothing still matches what coregen generated.
# Detected here rather than with $(wildcard) because on a clean build the files
# do not exist until coregen has run, and wildcard is evaluated before that.
ifeq ($(BLE_ENABLED),1)
  ifeq ($(MCU_PART),STM32WBA55xx)
    LDSCRIPT = $(SDK_DIR)sdk/device/stm32wba55hg_ble.ld
    CFLAGS += -I$(SDK_DIR)sdk/ble/include
    CFLAGS += -I$(SDK_DIR)sdk/ble/include/auto
    CFLAGS += -I$(SDK_DIR)sdk/ble/link_layer/inc
    CFLAGS += -DBLE_ENABLED=1 -DBASIC_FEATURES=1 -DBLE=1
    CFLAGS += -D'__PACKED_STRUCT=struct __attribute__((packed))'
    CFLAGS += -D'__PACKED_UNION=union __attribute__((packed))'
    CFLAGS += -include $(SDK_DIR)sdk/ble/include/cmsis_compiler.h
    BLE_LIBS  = $(SDK_DIR)sdk/ble/lib/stm32wba_ble_stack_basic.a
    BLE_LIBS += $(SDK_DIR)sdk/ble/lib/LinkLayer_BLE_Basic_lib.a
    BLE_SOURCES = $(wildcard $(SDK_DIR)sdk/ble/*.c)
    BLE_OBJS = $(addprefix $(BUILD_DIR)/sdk/ble/, $(notdir $(BLE_SOURCES:.c=.o)))
    BLE_OBJS += $(BUILD_DIR)/sdk/core/core_ble.o
  else
    $(error BLE is only supported on Core.ST.W5 (STM32WBA55xx) — this build is \
$(MCU_PART). Remove "ble" from config.json, or build for Core.ST.W5.)
  endif
endif

ASFLAGS = $(CPU_FLAGS) -Wall

LDFLAGS  = $(CPU_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
# --gc-sections everywhere, BLE included. The BLE build used to opt out, which
# arrived with the very first bring-up commit ("everything needed to compile and
# link") and was never revisited — it reads as a get-it-linking expedient rather
# than a requirement. Keeping it cost 62 KB of dead code in every BLE binary:
# 200116 B -> 136696 B on the same project, a 31% cut, and 31% off the flash
# time that goes with it.
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -specs=nosys.specs -specs=nano.specs
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref

# ---- Object files ----

C_OBJS   = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
ASM_OBJS = $(patsubst $(SDK_DIR)%.s, $(BUILD_DIR)/%.o, $(ASM_SOURCES))
HAL_SOURCES = $(wildcard $(SDK_DIR)sdk/hal/*.c)
HAL_OBJS = $(addprefix $(BUILD_DIR)/sdk/hal/, $(notdir $(HAL_SOURCES:.c=.o)))
GEN_OBJS = $(GEN_SOURCES:.c=.o)
# Core-layer sources compiled into every build (the rest of core_* is
# header-only): core_led.c owns the heartbeat state the SysTick handler
# drives, core_pdm.c the PDM decimator, core_stepper.c the STEP/DIR pulse
# generator's ISR state. (core_ble.o stays BLE-gated, below.)
CORE_OBJS = $(BUILD_DIR)/sdk/core/core_led.o $(BUILD_DIR)/sdk/core/core_pdm.o \
            $(BUILD_DIR)/sdk/core/core_stepper.o
OBJECTS  = $(C_OBJS) $(ASM_OBJS) $(HAL_OBJS) $(CORE_OBJS) $(GEN_OBJS)

ifeq ($(TILES_ENABLED),1)
  TILES_OBJS = $(addprefix $(BUILD_DIR)/tiles/, $(notdir $(TILES_SOURCES:.c=.o)))
  OBJECTS += $(TILES_OBJS)
endif

ifeq ($(BLE_ENABLED),1)
  OBJECTS += $(BLE_OBJS)
endif

ifeq ($(WAMR_ENABLED),1)
  OBJECTS += $(WAMR_ALL_OBJS)
endif

# ---- Default goal ----

.DEFAULT_GOAL := all

# ---- Generated headers (coregen) ----

GEN_HEADERS = $(GEN_DIR)/core_pads.h $(GEN_DIR)/core_board.h $(GEN_DIR)/core_interfaces.h

ifeq ($(CONFIG_FOUND),1)
  GEN_HEADERS  += $(GEN_DIR)/core_config.h $(GEN_DIR)/core_init.h
  GEN_SOURCES   = $(GEN_DIR)/core_init.c
  ifeq ($(_BLE_CFG),1)
    GEN_HEADERS += $(GEN_DIR)/ble_contract.h
    GEN_SOURCES += $(GEN_DIR)/ble_contract.c
  endif
  COREGEN_FLAGS = --config "$(CONFIG_JSON)"
  # Per-project WAMR natives ride alongside core_init.c so the adapters
  # for pad/pwm/adc/dac (which reach into PAD_*_PORT macros, core_dac,
  # core_adc1, etc.) compile in the same translation-unit scope. Only
  # link when the project pulled WAMR in — otherwise the symbols are
  # generated but ignored at link time.
  ifeq ($(WAMR_ENABLED),1)
    GEN_HEADERS += $(GEN_DIR)/studio_natives_project.h
    GEN_SOURCES += $(GEN_DIR)/studio_natives_project.c
  endif
else
  GEN_SOURCES   =
  COREGEN_FLAGS =
  # Config-less codegen produces no core.h / core_config.h / core_init.* — which
  # every modern project needs (main.c does `#include "core.h"`). This is almost
  # always a misconfiguration, so say so loudly instead of failing cryptically
  # later at the C preprocessor.
  $(warning coregen: no config.json found at "$(CONFIG_JSON)" — building config-less)
  $(warning coregen: core.h / core_config.h / core_init.* will NOT be generated)
  $(warning coregen: if you have a config.json, check PROJECT_DIR and the path (spaces/backslashes break detection))
endif

# core_drivers.mk is an included makefile — it must have its own recipe so
# GNU Make detects it was remade and restarts (picking up TILES_DRIVERS).
$(GEN_DIR)/core_drivers.mk: $(TILE_JSON) $(if $(CONFIG_FOUND),$(CONFIG_JSON)) $(SDK_DIR)tools/coregen/coregen.py $(SDK_DIR)tools/coregen/templates/*.j2
	@mkdir -p "$(GEN_DIR)"
	@echo "  GEN   $(TILE)"
	$(Q)$(COREGEN) "$(TILE_JSON)" "$(GEN_DIR)" $(COREGEN_FLAGS) $(COREGEN_QUIET)

# Stamp prevents re-running coregen for each header file target.
GEN_STAMP = $(GEN_DIR)/.coregen.stamp
$(GEN_STAMP): $(GEN_DIR)/core_drivers.mk
	$(Q)touch $(GEN_STAMP)

$(GEN_HEADERS): $(GEN_STAMP)

.PHONY: generate
generate: $(GEN_STAMP)

# ---- Rules ----

.PHONY: all clean distclean flash flash-coreprobe probe-power-check size

all: $(TARGET).bin $(TARGET).hex size

$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	@echo "  LD    $(notdir $@)"
ifeq ($(BLE_ENABLED),1)
	$(Q)$(CC) $(OBJECTS) $(BLE_LIBS) $(LDFLAGS) -o $@
else
	$(Q)$(CC) $(OBJECTS) $(LDFLAGS) -o $@
endif

$(TARGET).bin: $(TARGET).elf
	@echo "  BIN   $(notdir $@)"
	$(Q)$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	@echo "  HEX   $(notdir $@)"
	$(Q)$(OBJCOPY) -O ihex $< $@

# C sources depend on generated headers
$(BUILD_DIR)/%.o: $(PROJECT_DIR)/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

# Generated C sources (core_init.c, studio_natives_project.c, etc.).
# Static pattern: each `<name>.o` in GEN_OBJS pairs with the matching
# `<name>.c` in GEN_SOURCES. Static-pattern syntax keeps the rule
# crisp even when GEN_DIR is an absolute path (regular `%` pattern
# rules silently fail to match in that case on some make builds).
$(GEN_OBJS): $(GEN_DIR)/%.o: $(GEN_DIR)/%.c $(GEN_HEADERS)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

# HAL sources
$(BUILD_DIR)/sdk/hal/%.o: $(SDK_DIR)sdk/hal/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

# Core-layer sources compiled into every build (core_led.c). core_ble.c is
# in the same dir but BLE-gated, with its own explicit rule below.
$(BUILD_DIR)/sdk/core/core_led.o: $(SDK_DIR)sdk/core/core_led.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/core/core_pdm.o: $(SDK_DIR)sdk/core/core_pdm.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/core/core_stepper.o: $(SDK_DIR)sdk/core/core_stepper.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/device/%.o: $(SDK_DIR)sdk/device/%.s
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  AS    $(notdir $<)"
	$(Q)$(AS) $(ASFLAGS) -c $< -o $@

# Tile driver sources
ifeq ($(TILES_ENABLED),1)
$(BUILD_DIR)/tiles/%.o: $(SDK_DIR)hal/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -c "$<" -o $@

$(BUILD_DIR)/tiles/%.o: $(SDK_DIR)drivers/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -c "$<" -o $@
endif

# BLE sources (relaxed warnings — vendor headers are noisy)
ifeq ($(BLE_ENABLED),1)
$(BUILD_DIR)/sdk/ble/%.o: $(SDK_DIR)sdk/ble/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -c $< -o $@

# core_ble.c — in sdk/core/ but only compiled for BLE builds
$(BUILD_DIR)/sdk/core/core_ble.o: $(SDK_DIR)sdk/core/core_ble.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@
endif

size: $(TARGET).elf
	@echo ""
	$(Q)$(SIZE) $<
	@echo ""

# Generated sources compile in place, so their objects live in GEN_DIR rather
# than BUILD_DIR. Sweep those too: leaving them behind means switching a project
# between a soft-float Core (L0) and a hard-float one (L4/W5/H5) links a stale
# object with the wrong FP ABI and fails with "uses VFP register arguments".
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(GEN_DIR)/*.o $(GEN_DIR)/*.d

distclean: clean
	rm -rf $(GEN_DIR)
	rm -f $(PROJECT_DIR)/core.h
	rm -f $(PROJECT_DIR)/tiles.h

# ---- Flash via CoreProbe (CMSIS-DAP + probe-rs) ----
#
# Separate from `flash` (OpenOCD / CubeProgrammer over an ST-Link) because the
# CoreProbe can also POWER the board, and that is a decision no tool may make on
# its own: 5 V into a 1V8 part destroys it. tools/coreprobe_power.py reads the
# project's declaration out of config.json and refuses rather than guessing —
# see its module docstring for the rules and the config keys.
#
# The power step runs first and gates the flash: if it refuses, nothing is
# programmed. A board that already has its own rail is never fed, whatever the
# project declares.
flash-coreprobe: $(TARGET).elf
	@command -v probe-rs >/dev/null 2>&1 || { \
		echo "  SWD   probe-rs not found. Install it once:"; \
		echo "  SWD     curl -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh"; \
		exit 1; }
	@$(PYTHON) "$(SDK_DIR)tools/coreprobe_power.py" apply --config "$(CONFIG_JSON)"
	@echo "  SWD   probe-rs download ($(PROBE_RS_CHIP))"
	probe-rs download --chip $(PROBE_RS_CHIP) --binary-format elf $<
	probe-rs reset --chip $(PROBE_RS_CHIP)

# What the power step would do, without driving anything. Useful when bringing
# up an unfamiliar board, and the only safe way to find out what a project
# declares before it is acted on.
probe-power-check:
	@$(PYTHON) "$(SDK_DIR)tools/coreprobe_power.py" apply --config "$(CONFIG_JSON)" --dry-run

# ---- Flash via OpenOCD (ST-Link) ----

flash: $(TARGET).elf
ifeq ($(FLASH_TOOL),cubeprog)
	@[ -x "$(STM32_PROG_CLI)" ] || { \
		echo "STM32_Programmer_CLI not found at:"; \
		echo "  $(STM32_PROG_CLI)"; \
		echo "Install STM32CubeProgrammer, or point at yours:"; \
		echo "  make flash STM32_PROG_CLI=/path/to/STM32_Programmer_CLI"; \
		exit 1; }
	"$(STM32_PROG_CLI)" -c port=SWD mode=UR -w $< -v -rst
else
	openocd -f $(OPENOCD_CFG) \
		-c "init" \
		-c "reset halt" \
		-c "program $< verify" \
		-c "reset run" \
		-c "exit"
endif

# ---- Flash via USB DFU ----
#
# flash-dfu:  Smart flash via USB DFU. Behavior depends on boot mode:
#
#   BOOTLOADER=1: 1200-baud touch triggers custom DFU bootloader.
#     Tries plain DFU 1.1 first, falls back to DfuSe at APP_ADDR.
#
#   ROM_DFU=1: 1200-baud touch triggers ROM bootloader (DfuSe).
#     Flashes at 0x08000000 with :leave to auto-reset into app.
#
#   Neither: Direct DfuSe flash to 0x08000000 (board must already
#     be in DFU mode, e.g. BOOT0 held high).
#
# flash-rom:  Always flashes at 0x08000000 via ROM DFU (for fresh boards
#             or when BOOT0 is held high). Works regardless of boot mode.

# Canned recipe: the 1200-baud touch that reboots the Core into DFU. Shared by
# both flash-dfu branches so the host-specific handling lives in one place.
ifeq ($(HOST_OS),windows)
# Windows has no /dev entry for a USB-CDC port, so there is nothing to glob and
# stty cannot address COM ports. Do the touch through pyserial when the user
# names the port, and otherwise say plainly that this step is manual — the old
# code globbed a macOS path, found nothing, said nothing, and left dfu-util to
# fail with "No DFU capable USB device available".
define DFU_TOUCH
@if [ -n "$(SERIAL_PORT)" ]; then \
	echo "  DFU   Triggering reboot via 1200-baud touch on $(SERIAL_PORT)..."; \
	$(PYTHON) -c "import serial,sys; p=serial.Serial(sys.argv[1],1200); p.setDTR(False); p.close()" "$(SERIAL_PORT)" \
		|| { echo "  DFU   Touch failed — it needs pyserial (pip install pyserial)"; \
		     echo "  DFU   or the port name is wrong (Device Manager -> Ports (COM & LPT))."; }; \
	echo "  DFU   Waiting for DFU device..."; \
	sleep 4; \
else \
	echo "  DFU   Windows: no automatic 1200-baud touch — COM ports have no /dev entry."; \
	echo "  DFU     name the port:  make flash-dfu SERIAL_PORT=COM5   (needs pyserial)"; \
	echo "  DFU     or enter DFU by hand: hold BOOT0 while plugging in USB"; \
	echo "  DFU   dfu-util on Windows also needs the WinUSB driver bound to the device"; \
	echo "  DFU   (install it once with Zadig). 'make doctor' checks the rest."; \
fi
endef
else
define DFU_TOUCH
@TTY="$(SERIAL_PORT)"; \
if [ -z "$$TTY" ]; then TTY=$$(ls $(SERIAL_GLOB) 2>/dev/null | head -1); fi; \
if [ -n "$$TTY" ]; then \
	echo "  DFU   Triggering reboot via 1200-baud touch on $$TTY..."; \
	stty $(STTY_DEV) "$$TTY" 1200 hupcl; \
	echo "  DFU   Waiting for DFU device..."; \
	sleep 4; \
else \
	echo "  DFU   No serial port matched $(SERIAL_GLOB) — assuming the Core is"; \
	echo "  DFU   already in DFU mode. If it isn't, plug it in (or hold BOOT0)."; \
fi
endef
endif

flash-dfu: $(TARGET).bin
ifeq ($(BOOTLOADER),1)
	$(DFU_TOUCH)
	@_dfu_log=$$(mktemp); \
	_dfu_ok=0; \
	dfu-util -a 0 -R -D $< > "$$_dfu_log" 2>&1; \
	if grep -q "Download done" "$$_dfu_log"; then _dfu_ok=1; fi; \
	if [ "$$_dfu_ok" = "0" ]; then \
		if [ "$(ALLOW_ROM_FALLBACK)" = "1" ]; then \
			dfu-util -a 0 -s $(APP_ADDR):leave -D $< > "$$_dfu_log" 2>&1; \
			if grep -q "Download done" "$$_dfu_log"; then _dfu_ok=1; fi; \
		else \
			echo "  DFU   Plain DFU failed — the device is likely the ST ROM bootloader (DfuSe)."; \
			echo "  DFU   REFUSING the DfuSe fallback at $(APP_ADDR): this build assumes a"; \
			echo "  DFU   custom-bootloader layout, but if the board was last flashed in"; \
			echo "  DFU   ROM-DFU layout (e.g. from Studio, whose default is bootloader=rom),"; \
			echo "  DFU   writing at $(APP_ADDR) would corrupt the resident app."; \
			echo "  DFU   - Custom bootloader IS present on this board:  re-run with ALLOW_ROM_FALLBACK=1"; \
			echo "  DFU   - Board is ROM-DFU layout (Studio-flashed):    build with ROM_DFU=1 BOOTLOADER=0"; \
			echo "  DFU     and use 'make flash-rom' (targets 0x08000000)"; \
			cat "$$_dfu_log"; \
			rm -f "$$_dfu_log"; \
			exit 1; \
		fi; \
	fi; \
	if [ "$$_dfu_ok" = "1" ]; then \
		_sz=$$(grep -o '[0-9]* bytes$$' "$$_dfu_log" | tail -1); \
		echo "  DFU   Downloaded $${_sz:-firmware}"; \
		echo "  DFU   OK"; \
		echo "  DFU   Resetting..."; \
	else \
		cat "$$_dfu_log"; \
		echo "  DFU   FAILED"; \
	fi; \
	rm -f "$$_dfu_log"
else ifeq ($(ROM_DFU),1)
	$(DFU_TOUCH)
	@_dfu_log=$$(mktemp); \
	dfu-util -a 0 -s 0x08000000:leave -D $< > "$$_dfu_log" 2>&1 || true; \
	if grep -q "File downloaded successfully" "$$_dfu_log"; then \
		_sz=$$(grep -o 'size = [0-9]*' "$$_dfu_log" | tail -1 | grep -o '[0-9]*'); \
		echo "  DFU   Downloaded $${_sz:-?} bytes"; \
		echo "  DFU   Resetting..."; \
		sleep 5; \
		if [ -n "$(SERIAL_GLOB)" ] && ls $(SERIAL_GLOB) >/dev/null 2>&1; then \
			echo "  DFU   OK"; \
		else \
			echo "  DFU   OK — power cycle if the app hasn't started"; \
		fi; \
	else \
		cat "$$_dfu_log"; \
		echo "  DFU   FAILED"; \
		rm -f "$$_dfu_log"; \
		exit 1; \
	fi; \
	rm -f "$$_dfu_log"
else
	dfu-util -a 0 -s 0x08000000:leave -D $<
endif

flash-rom: $(TARGET).bin
	dfu-util -a 0 -s 0x08000000:leave -D $<

# ---- Flash the DFU bootloader itself ----

flash-bootloader:
	$(MAKE) -C $(SDK_DIR)sdk/bootloader flash

# ---- GDB debug session ----
# Step 1: Run 'make openocd' in one terminal (starts GDB server)
# Step 2: Run 'make gdb' in another terminal (connects debugger)

openocd: $(TARGET).elf
	openocd -f $(OPENOCD_CFG) \
		-c "init" \
		-c "reset halt" \
		-c "program $< verify" \
		-c "reset halt"

gdb: $(TARGET).elf
	$(GDB) $< \
		-ex "target extended-remote :3333" \
		-ex "monitor reset halt" \
		-ex "load" \
		-ex "break main" \
		-ex "continue"

# ---- SWO trace output ----

swo: $(TARGET).elf
	openocd -f $(OPENOCD_CFG) \
		-c "init" \
		-c "reset halt" \
		-c "program $< verify" \
		-c "tpiu config internal /dev/stdout uart off $(SYSCLK_HZ) 2000000" \
		-c "itm port 0 on" \
		-c "reset run"

# ---- Toolchain check ----
# `make doctor` — one command that says what is missing and how to get it.
# Worth its own target because the failure modes it covers are otherwise
# indistinguishable from an SDK bug: a missing arm-none-eabi-gcc surfaces as
# "No such file or directory", the Store-stub python as a silent coregen no-op.

.PHONY: doctor
doctor:
	@echo "Host:      $(HOST_OS) ($(_UNAME_S))"
	@echo "Shell:     $(SHELL)"
	@echo "SDK:       $(SDK_DIR)"
	@echo "Project:   $(PROJECT_DIR)"
	@echo ""
	@_missing=0; \
	_check() { \
		if command -v "$$1" >/dev/null 2>&1; then \
			printf '  ok    %-20s %s\n' "$$1" "$$(command -v "$$1")"; \
		elif [ "$$2" = "optional" ]; then \
			printf '  --    %-20s not found (optional: %s)\n' "$$1" "$$3"; \
		else \
			printf '  MISS  %-20s not found — %s\n' "$$1" "$$3"; \
			_missing=1; \
		fi; \
	}; \
	printf '  ok    %-20s %s\n' "python" "$(PYTHON)"; \
	_check arm-none-eabi-gcc required "$(GCC_HINT)"; \
	_check dfu-util optional "$(DFU_HINT)"; \
	_check openocd optional "needed only for 'make flash' / 'make gdb' over SWD"; \
	echo ""; \
	if [ "$$_missing" = "1" ]; then \
		echo "Some required tools are missing — see the hints above."; \
		exit 1; \
	fi; \
	echo "Toolchain looks good."

ifeq ($(HOST_OS),windows)
  GCC_HINT := install the ARM toolchain (winget install Arm.GnuArmEmbeddedToolchain) and re-open Git Bash
  DFU_HINT := needed for 'make flash-dfu'; get dfu-util for Windows and bind WinUSB with Zadig
else ifeq ($(HOST_OS),linux)
  GCC_HINT := apt install gcc-arm-none-eabi (or the vendor tarball on PATH)
  DFU_HINT := needed for 'make flash-dfu'; apt install dfu-util
else
  GCC_HINT := brew install arm-none-eabi-gcc
  DFU_HINT := needed for 'make flash-dfu'; brew install dfu-util
endif

# ---- End-to-end validation ----
.PHONY: validate validate-generate
validate:
	@$(PYTHON) $(SDK_DIR)tools/validate.py

validate-generate:
	@$(PYTHON) $(SDK_DIR)tools/validate.py --generate-only
