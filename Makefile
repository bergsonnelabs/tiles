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

# Private overlays: checkouts of the private tiles repos (tiles-internal,
# tiles-alpha, tile-<family>-<name>), space-separated. Only public tiles'
# definitions and drivers are in this repo; a non-public Core, or a project
# using non-public tiles, builds from an overlay's definitions/<stem>.json and
# the drivers it declares in drivers/drivers.json. Public definitions/ is
# searched first. Exported so coregen looks up tile definitions and drivers
# the same way. The public Cores and tiles need none.
#   make TILE=<Core> TILES_OVERLAY="/abs/tiles-internal /abs/tiles-alpha"
TILES_OVERLAY ?=
export TILES_OVERLAY

# ---- Public Core name aliases ----
# The public names (Core.ST.<family>[.<n>], matching each board's silkscreen)
# resolve onto the canonical, DB-synced definition stems. Either form is
# accepted as TILE; the alias rewrites to the stem so the JSON lookup + MCU
# mapping below stay unchanged. Each public name maps to the most-complete
# definition for its board.
#
# Careful: Core.ST.L4 and Core.ST.L4.2 are two DIFFERENT boards (same STM32L422
# MCU; the L4.2 has more pads). Core.ST.L4 was called Core.ST.L4.1 until the
# 2026-09 rename.
#
# Retired names (before the 2026-09 rename) still resolve, with a NOTE, so
# older projects keep building: Core.ST.L4.1 -> Core.ST.L4, Core.ST.L0.1 ->
# Core.ST.L0, Core.ST.H5.1 -> Core.ST.H5, and the old stems Core-ST-L4-1-a/b,
# Core-ST-L0-1-a and Core-ST-H5-1-a. (MAKE_RESTARTS: say it once, not again
# when make re-reads itself after generating core_drivers.mk.)
_CORE_OLD_NAME := $(TILE)
ifeq ($(TILE),Core.ST.L4.1)
  override TILE := Core.ST.L4
else ifeq ($(TILE),Core.ST.L0.1)
  override TILE := Core.ST.L0
else ifeq ($(TILE),Core.ST.H5.1)
  override TILE := Core.ST.H5
else ifeq ($(TILE),Core-ST-L4-1-a)
  override TILE := Core-ST-L4-a
else ifeq ($(TILE),Core-ST-L4-1-b)
  override TILE := Core-ST-L4-b
else ifeq ($(TILE),Core-ST-L0-1-a)
  override TILE := Core-ST-L0-a
else ifeq ($(TILE),Core-ST-H5-1-a)
  override TILE := Core-ST-H5-a
endif
ifneq ($(TILE),$(_CORE_OLD_NAME))
  ifeq ($(MAKE_RESTARTS),)
    $(info NOTE: TILE=$(_CORE_OLD_NAME) is now $(TILE) (the old name still works; update your Makefile))
  endif
endif

ifeq ($(TILE),Core.ST.L4)
  override TILE := Core-ST-L4-b
else ifeq ($(TILE),Core.ST.L4.2)
  override TILE := Core-ST-L4-2-a
else ifeq ($(TILE),Core.ST.H5)
  override TILE := Core-ST-H5-a
else ifeq ($(TILE),Core.ST.L0)
  override TILE := Core-ST-L0-a
else ifeq ($(TILE),Core.ST.W5)
  override TILE := Core-ST-W5-b
endif

# ---- Core lookup: cores/<stem>/core.mk ----
# Each Core's MCU settings live in cores/<stem>/core.mk (contract in the file's
# header), searched like its definition: this SDK first, then each overlay's
# cores/<stem>/core.mk. An overlay Core may also be named by its public name
# (Core.<Family>.<Name>, dots), which has no alias above: Core.N.LM20B picks
# the overlay's highest-rev cores/Core-N-LM20B-<rev>/ (last in sort order),
# and TILE becomes that stem so the definition lookup and coregen get the stem.
# Included further down, where the per-Core settings have always been set.
_core_mk = $(firstword $(wildcard $(SDK_DIR)cores/$(1)/core.mk \
             $(foreach o,$(patsubst %/,%,$(TILES_OVERLAY)),$(o)/cores/$(1)/core.mk)))
CORE_MK := $(call _core_mk,$(TILE))
ifeq ($(CORE_MK),)
  ifneq ($(findstring .,$(TILE)),)
    _CORE_PREFIX := $(subst .,-,$(TILE))-
    _CORE_STEMS := $(sort $(foreach m, \
                     $(foreach o,$(patsubst %/,%,$(TILES_OVERLAY)),$(wildcard $(o)/cores/$(_CORE_PREFIX)*/core.mk)), \
                     $(notdir $(patsubst %/,%,$(dir $(m))))))
    # Only <prefix><rev> with a one-segment rev: Core.N.LM20 must not pick Core-N-LM20-B-a.
    _CORE_STEMS := $(foreach s,$(_CORE_STEMS),$(if $(findstring -,$(patsubst $(_CORE_PREFIX)%,%,$(s))),,$(s)))
    ifneq ($(_CORE_STEMS),)
      override TILE := $(lastword $(_CORE_STEMS))
      CORE_MK := $(call _core_mk,$(TILE))
    endif
  endif
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

# Toolchain: the Core's core.mk names it (TOOLCHAIN = arm-none-eabi for every
# public Core); recursive, because core.mk is included further down.
PREFIX  = $(TOOLCHAIN)-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size
GDB     = $(PREFIX)gdb

# Coregen
COREGEN      = $(PYTHON) "$(SDK_DIR)tools/coregen/coregen.py"
TILE_JSON    = $(firstword $(wildcard $(SDK_DIR)definitions/$(TILE).json \
                 $(foreach o,$(patsubst %/,%,$(TILES_OVERLAY)),$(o)/definitions/$(TILE).json)))
CONFIG_JSON ?= $(PROJECT_DIR)/config.json

# No definition anywhere: say so now, not as "No rule to make target".
ifeq ($(TILE_JSON),)
  ifeq (,$(filter clean distclean doctor,$(MAKECMDGOALS)))
    $(error No definition for TILE=$(TILE): definitions/$(TILE).json is not in \
this SDK$(if $(strip $(TILES_OVERLAY)), or in TILES_OVERLAY ($(strip $(TILES_OVERLAY))),). \
Only public tiles are here; a non-public Core builds from a checkout of its \
private repo: make TILE=$(TILE) TILES_OVERLAY=/path/to/tiles-internal)
  endif
endif
GEN_DIR      ?= $(PROJECT_DIR)/coregen

# Space/backslash-safe existence test for config.json. $(wildcard) treats its
# argument as a space-separated pattern list and mishandles backslashes, so a
# project under e.g. "C:/Users/First Last/proj" silently misses config.json and
# coregen runs config-less (no core.h / core_config.h / core_init.*). A shell
# `test -f` with the path quoted is robust to both. (Needs a POSIX shell, which
# the SHELL block above guarantees on Windows.)
CONFIG_FOUND := $(shell [ -f "$(CONFIG_JSON)" ] && echo 1)

# ---- Paths ----

BUILD_DIR ?= $(PROJECT_DIR)/build
TARGET    = $(BUILD_DIR)/$(PROJECT)

# ---- Tile → MCU mapping: include the Core's core.mk ----
# coregen generates the headers; the Makefile still needs to know CPU
# architecture and linker script for compiler flags. Those come from the
# core.mk found above (recursive `=`, so later blocks can still override
# LDSCRIPT). CORE_DIR (ends in /) is that file's directory, for an overlay
# core.mk to name its own files.

ifeq ($(CORE_MK),)
  $(error Unknown TILE: $(TILE). Supported: public names Core.ST.L0 / Core.ST.L4 / Core.ST.L4.2 / Core.ST.W5 / Core.ST.H5 (Core.ST.L4 and Core.ST.L4.2 are different boards), or stems Core-ST-L0-a, Core-ST-L4-a, Core-ST-L4-b, Core-ST-L4-2-a, Core-ST-W5-b, Core-ST-H5-a. A non-public Core needs cores/$(if $(findstring .,$(TILE)),$(subst .,-,$(TILE))-<rev>,$(TILE))/core.mk in a checkout of its private repo$(if $(strip $(TILES_OVERLAY)), (not found in TILES_OVERLAY: $(strip $(TILES_OVERLAY))),): make TILE=$(TILE) TILES_OVERLAY=/path/to/tiles-internal)
endif
CORE_DIR := $(dir $(CORE_MK))
include $(CORE_MK)

ifeq ($(strip $(TOOLCHAIN)),)
  $(error TILE=$(TILE): core.mk ($(CORE_MK)) sets no TOOLCHAIN (arm-none-eabi for \
every Cortex-M Core); the compiler is $$(TOOLCHAIN)-gcc)
endif

# The Core's status file (sdk/status/core-*.json, or an overlay's): coregen
# writes coregen/core_caps.h from it (CORE_HAS_*, the functions the Core
# lacks), so the status table, Studio and the compiler agree.
ifeq (,$(filter clean distclean doctor,$(MAKECMDGOALS)))
  ifeq ($(strip $(CORE_STATUS)),)
    $(error TILE=$(TILE): core.mk ($(CORE_MK)) sets no CORE_STATUS, the Core's \
status file (sdk/status/core-*.json) that coregen writes core_caps.h from)
  endif
  ifeq ($(wildcard $(CORE_STATUS)),)
    $(error TILE=$(TILE): CORE_STATUS $(CORE_STATUS) (core.mk $(CORE_MK)) does not exist)
  endif
endif

# ---- Port hook ----
# PORT = stm32 (every public Core) is what the rest of this Makefile builds. A
# core.mk for any other port sets PORT_MK to that port's make fragment (e.g.
# PORT_MK = $(CORE_DIR)../../sdk/port/nrf54l/port.mk in an overlay), included
# right here: after core.mk, before the flags and object lists are put
# together. The STM32-only parts below apply only to PORT = stm32: -D$(MCU_PART),
# the sdk/ll, sdk/hal, sdk/tal and sdk/port/stm32/core include dirs, the
# sdk/hal sources, the sdk/core objects (CORE_OBJS), the sdk/device startup
# rule, the bootloader, BLE and Thread blocks, and the flash / debug targets
# (OpenOCD, CubeProgrammer, DFU, serial update, gdb, swo). A port fragment
# supplies its own instead, by setting (use recursive `=` for anything that
# reads BLE_ENABLED, THREAD_ENABLED, WAMR_ENABLED or _BOOT_MODE, which are
# set after it):
#   PORT_CFLAGS     include dirs (-I), defines (-D) and other compile flags;
#                   they follow -I sdk/core, where STM32 puts its port dir
#   PORT_SOURCES    its .c files (absolute paths, unique basenames), compiled
#                   with CFLAGS into $(BUILD_DIR)/port/, linked first
#   STARTUP         its startup .s / .S (or core.mk sets it), assembled into
#                   $(BUILD_DIR)/port/, linked after the generated objects
#   PORT_ASFLAGS    extra assembler flags (-I / -D for a .S startup)
#   PORT_CPU_FLAGS  CPU flags for a TOOLCHAIN other than arm-none-eabi (for
#                   that one they come from core.mk's CPU / FPU / FLOAT_ABI)
#   PORT_LDFLAGS    extra link flags, after the generic ones
#   PORT_LIBS       archives, linked after the objects
#   LDSCRIPT        unless core.mk sets it
#   flash targets   `flash` and whatever else it offers, as plain rules. The
#                   generic targets stay: all, size, clean, distclean,
#                   generate, doctor, flash-coreprobe, probe-power-check,
#                   reset-serial.
# It can use SDK_DIR, CORE_DIR, TILE, PROJECT, PROJECT_DIR, GEN_DIR,
# CONFIG_JSON, BUILD_DIR, TARGET, PYTHON, CC / AS / OBJCOPY, Q and LOG, and
# what core.mk set. Same contract in AI.md (Build System, "Port fragment").
ifneq ($(strip $(PORT_MK)),)
  include $(PORT_MK)
else ifneq ($(PORT),stm32)
  $(error TILE=$(TILE): core.mk ($(CORE_MK)) sets PORT=$(PORT) but no PORT_MK; a non-stm32 port needs its own make fragment)
endif

# ---- Bootloader support (PORT = stm32) ----
# Bootloader mode is read from config.json: "bootloader": "custom"|"rom"|"none"
#
#   "custom" — Custom DFU 1.1 bootloader at 0x08000000, app at 0x08002000.
#   "rom"    — ROM DfuSe bootloader (0483:DF11), app at 0x08000000.
#   "none"   — No DFU support. Flash via SWD or BOOT0.
#
# The mode maps to BOOTLOADER/ROM_DFU flags below. Command-line overrides
# still work (e.g. make BOOTLOADER=1) because ?= defers to explicit values.
# The link script for each layout is the Core's (core.mk): LDSCRIPT_APP for
# the custom-bootloader layout, LDSCRIPT_ROMDFU for the ROM-DFU one. Another
# port reads _BOOT_MODE itself.

_BOOT_MODE := $(shell $(PYTHON) -c "import json; print(json.load(open('$(CONFIG_JSON)')).get('bootloader','none'))" 2>/dev/null || echo none)

ifeq ($(PORT),stm32)
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
  ifeq ($(strip $(LDSCRIPT_APP)),)
    $(error BOOTLOADER=1: TILE=$(TILE) has no custom-bootloader link script \
(app at 0x08002000): its core.mk ($(CORE_MK)) sets no LDSCRIPT_APP. Use \
"bootloader": "rom" in config.json)
  endif
  APP_ADDR   = 0x08002000
  APP_OFFSET = $(APP_ADDR)UL
  LDSCRIPT   = $(LDSCRIPT_APP)
endif

ifeq ($(ROM_DFU),1)
  ifeq ($(strip $(LDSCRIPT_ROMDFU)),)
    $(error ROM_DFU=1: TILE=$(TILE) has no ROM-DFU link script (app at \
0x08000000): its core.mk ($(CORE_MK)) sets no LDSCRIPT_ROMDFU. A Core whose \
standard LDSCRIPT already serves that layout names that file)
  endif
  LDSCRIPT = $(LDSCRIPT_ROMDFU)
endif
endif

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

# arm-none-eabi: -mcpu / -mthumb / -mfpu from core.mk. Another toolchain's
# port fragment gives its own (PORT_CPU_FLAGS).
ifeq ($(TOOLCHAIN),arm-none-eabi)
  CPU_FLAGS = -mcpu=$(CPU) -mthumb
  ifdef FPU
    CPU_FLAGS += -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)
  endif
else
  CPU_FLAGS = $(PORT_CPU_FLAGS)
endif

CFLAGS  = $(CPU_FLAGS)
CFLAGS += -Wall -Wextra -Wshadow -Wdouble-promotion
CFLAGS += -fdata-sections -ffunction-sections -fno-common
CFLAGS += -std=c17
ifeq ($(PORT),stm32)
  CFLAGS += -D$(MCU_PART)
endif
CFLAGS += -I$(GEN_DIR)
CFLAGS += -I$(PROJECT_DIR)
ifeq ($(PORT),stm32)
  CFLAGS += -I$(SDK_DIR)sdk/ll
  CFLAGS += -I$(SDK_DIR)sdk/hal
  CFLAGS += -I$(SDK_DIR)sdk/tal
endif
CFLAGS += -I$(SDK_DIR)sdk/core
# The port's own headers (port_caps.h, which coregen's core_caps.h includes
# for the facts that are not status features), after the neutral SDK dirs so
# they never shadow an sdk/ header. Another port puts its own port_caps.h on
# the include path, through PORT_CFLAGS.
ifeq ($(PORT),stm32)
  CFLAGS += -I$(SDK_DIR)sdk/port/stm32/core
else
  CFLAGS += $(PORT_CFLAGS)
endif
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
  # Overlay drivers' headers first: an overlay's driver wins over a public
  # one of the same name, as in coregen's driver map.
  CFLAGS += $(foreach d,$(wildcard $(foreach o,$(patsubst %/,%,$(TILES_OVERLAY)),$(o)/drivers)),-I"$(d)")
  CFLAGS += -I"$(SDK_DIR)" -I"$(SDK_DIR)drivers" -I"$(SDK_DIR)hal"
  TILES_SOURCES =
  ifdef TILES_DRIVERS
    TILES_SOURCES += $(foreach drv,$(TILES_DRIVERS),$(SDK_DIR)drivers/$(drv).c)
  endif
  # Drivers from TILES_OVERLAY checkouts, by path (coregen writes them).
  ifdef TILES_OVERLAY_DRIVERS
    TILES_SOURCES += $(addsuffix .c,$(TILES_OVERLAY_DRIVERS))
  endif
endif

# ---- WAMR runtime support (optional; A4c) ----
# Opt-in via `WAMR_ENABLED=1`. When enabled, links the WAMR
# interpreter into the firmware so a user's DSL-compiled `.wasm`
# blob can execute on-device. Gated only to Cortex-M33 targets
# (Core.ST.H5, Core.ST.W5) — the M4 footprint of the L4 Cores (Core.ST.L4,
# Core.ST.L4.2) is over budget per the A4b spike and routes through a separate
# interpreter path.
WAMR_ENABLED ?= 0
ifeq ($(WAMR_ENABLED),1)
  ifneq ($(CPU),cortex-m33)
    $(error WAMR_ENABLED=1 is only supported on Cortex-M33 Cores (Core.ST.H5, Core.ST.W5))
  endif
  include $(SDK_DIR)sdk/wamr/wamr.mk
endif

# ---- Scope (core_scope) ----
#
# On unless config.json says otherwise:
#
#   "scope": { "enabled": false }
#
# which compiles every core_scope_* call to nothing: the production-build
# switch (no flash, no RAM, no Studio Scope characteristic; Studio Link stays
# for the BLE firmware update). A project that never
# calls core_scope pays nothing either way: --gc-sections drops the module.
# `make SCOPE_ENABLED=0` overrides for a local one-off.
_SCOPE_CFG := $(shell $(PYTHON) -c "import json; c=json.load(open('$(CONFIG_JSON)')); print(0 if (c.get('scope') or {}).get('enabled') is False else 1)" 2>/dev/null || echo 1)
SCOPE_ENABLED ?= $(_SCOPE_CFG)
ifeq ($(SCOPE_ENABLED),0)
  CFLAGS += -DCORE_SCOPE_ENABLED=0
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
#
# Source trees:
#   sdk/wpan/   the 2.4 GHz radio plumbing any ST link-layer protocol needs
#               (sequencer, timer server, flash manager, link-layer platform
#               and ll_sys glue, radio IRQs, RNG/AES, memory managers,
#               link-layer headers, per-protocol ll_fw_config.h and libraries).
#               Shared by BLE and Thread.
#   sdk/ble/    the BLE host stack, its glue and the core_ble backend.
#   sdk/thread/ OpenThread (prebuilt FTD library), its platform glue and the
#               core_thread backend.
# Which link-layer library a build links (WPAN_LL_LIB) depends on the
# protocol it runs: BLE uses ST's BLE "basic" variant, Thread the Thread one,
# and BLE + Thread together ST's concurrent BLE_Thread one (see below).
_THREAD_CFG := $(shell $(PYTHON) -c "import json; c=json.load(open('$(CONFIG_JSON)')); print(1 if (c.get('thread') or {}).get('enabled') else 0)" 2>/dev/null || echo 0)
THREAD_ENABLED ?= $(_THREAD_CFG)
# Both on: one image runs both stacks (the BLE + Thread block below). The
# BLE-only and Thread-only blocks are untouched by it.
BLE_THREAD := $(if $(filter 11,$(BLE_ENABLED)$(THREAD_ENABLED)),1,0)

# sdk/wpan files that came from sdk/ble keep that file's place in the BLE link
# order (link order is flash order, see OBJECTS), so BLE images stay
# byte-identical to before the move. <wpan name>=<old sdk/ble name>.
WPAN_BLE_ORDER := wpan_flash=ble_flash wpan_irq=ble_irq wpan_seq_idle=ble_seq_idle wpan_sys=ble_glue
wpan_order_key = $(or $(patsubst $(1)=%,%,$(filter $(1)=%,$(WPAN_BLE_ORDER))),$(1))

# ---- BLE + Thread in one image (Core.ST.W5 only) ----
#
#   "ble": { "enabled": true }, "thread": { "enabled": true, ... }
#
# ST's concurrent link layer (LinkLayer_BLE_Thread_lib.a, STM32CubeWBA 1.6.1,
# the regular build for production silicon, not the _cut1_ one for cut 1.x
# samples) with its ll_fw_config.h, the BLE basic host stack, ST's mbedTLS and
# a trimmed OpenThread FTD library (stm32wba_ot_ftd_trim_lib.a: ST's OpenThread
# source rebuilt by tools/build_openthread_trim.py with fewer buffers and
# without the unused features, sdk/thread/config/
# stm32wba-openthread-ftd-trim-config.h, so both stacks leave RAM for an
# application). Compiled the way ST's BLE_HeartRate_Thread project compiles
# them: BLE and Thread defines, and no
# DISABLE_RFTS_EXT_EVNT_HNDLR (this link layer schedules the flash manager's
# radio-free windows, as the BLE one does). sdk/ble/include goes ahead of
# sdk/thread/include: its app_conf.h is the build's (it adds the OpenThread
# tasks when THREAD_ENABLED is defined). Linker script: the BLE one plus the
# OpenThread settings pages, kept out of the BLE update's staging area.
#
# STM32 only (another port brings its own radio stacks). Which Cores have the
# ST stacks, and their link scripts, is the Core's: core.mk's HAS_BLE_ST /
# HAS_THREAD_ST and LDSCRIPT_BLE / LDSCRIPT_THREAD / LDSCRIPT_BLE_THREAD.
ifeq ($(PORT),stm32)
ifeq ($(BLE_THREAD),1)
  ifeq ($(HAS_BLE_ST)$(HAS_THREAD_ST),11)
    LDSCRIPT = $(LDSCRIPT_BLE_THREAD)
    CFLAGS += -I$(SDK_DIR)sdk/ble/include
    CFLAGS += -I$(SDK_DIR)sdk/ble/include/auto
    CFLAGS += -I$(SDK_DIR)sdk/thread/include
    CFLAGS += -I$(SDK_DIR)sdk/thread/config
    CFLAGS += -I$(SDK_DIR)sdk/wpan/include
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/inc
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/inc/ot_inc
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/config/ble_thread
    CFLAGS += -DBLE_ENABLED=1 -DBASIC_FEATURES=1 -DBLE=1
    CFLAGS += -DTHREAD_ENABLED=1 -DMAC=1 -DOPENTHREAD_FTD=1
    CFLAGS += -D'OPENTHREAD_CONFIG_FILE=<stm32wba-openthread-ftd-trim-config.h>'
    CFLAGS += -D'OPENTHREAD_PROJECT_CORE_CONFIG_FILE=<stm32wba-openthread-ftd-trim-config.h>'
    CFLAGS += -D'__PACKED_STRUCT=struct __attribute__((packed))'
    CFLAGS += -D'__PACKED_UNION=union __attribute__((packed))'
    CFLAGS += -include $(SDK_DIR)sdk/wpan/include/cmsis_compiler.h
    WPAN_LL_LIB = $(SDK_DIR)sdk/wpan/lib/LinkLayer_BLE_Thread_lib.a
    BLE_THREAD_LIBS  = $(SDK_DIR)sdk/ble/lib/stm32wba_ble_stack_basic.a
    BLE_THREAD_LIBS += $(SDK_DIR)sdk/thread/lib/stm32wba_ot_ftd_trim_lib.a
    BLE_THREAD_LIBS += $(SDK_DIR)sdk/thread/lib/stm32wba_mbedtls_ftd_lib.a
    BLE_THREAD_LIBS += $(WPAN_LL_LIB)
    # All of sdk/wpan and sdk/ble in the BLE build's order, then sdk/thread.
    WPAN_SOURCES = $(wildcard $(SDK_DIR)sdk/wpan/*.c)
    BLE_SOURCES = $(wildcard $(SDK_DIR)sdk/ble/*.c)
    THREAD_SOURCES = $(wildcard $(SDK_DIR)sdk/thread/*.c)
    BLE_THREAD_OBJS = $(foreach e,$(sort $(foreach n,$(notdir $(basename $(WPAN_SOURCES) $(BLE_SOURCES))),$(call wpan_order_key,$(n))+$(n))),$(foreach n,$(word 2,$(subst +, ,$(e))),$(if $(filter %/sdk/wpan/$(n).c,$(WPAN_SOURCES)),$(BUILD_DIR)/sdk/wpan/$(n).o,$(BUILD_DIR)/sdk/ble/$(n).o)))
    BLE_THREAD_OBJS += $(addprefix $(BUILD_DIR)/sdk/thread/,$(notdir $(THREAD_SOURCES:.c=.o)))
    BLE_THREAD_OBJS += $(BUILD_DIR)/sdk/core/core_ble.o $(BUILD_DIR)/sdk/core/core_thread.o
  else
    $(error BLE and Thread are only supported on Core.ST.W5 (STM32WBA55xx) — this build is \
$(MCU_PART). Remove "ble" and "thread" from config.json, or build for Core.ST.W5.)
  endif
else ifeq ($(BLE_ENABLED),1)
  ifeq ($(HAS_BLE_ST),1)
    LDSCRIPT = $(LDSCRIPT_BLE)
    CFLAGS += -I$(SDK_DIR)sdk/ble/include
    CFLAGS += -I$(SDK_DIR)sdk/ble/include/auto
    CFLAGS += -I$(SDK_DIR)sdk/wpan/include
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/inc
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/config/ble
    CFLAGS += -DBLE_ENABLED=1 -DBASIC_FEATURES=1 -DBLE=1
    CFLAGS += -D'__PACKED_STRUCT=struct __attribute__((packed))'
    CFLAGS += -D'__PACKED_UNION=union __attribute__((packed))'
    CFLAGS += -include $(SDK_DIR)sdk/wpan/include/cmsis_compiler.h
    WPAN_LL_LIB = $(SDK_DIR)sdk/wpan/lib/LinkLayer_BLE_Basic_lib.a
    BLE_LIBS  = $(SDK_DIR)sdk/ble/lib/stm32wba_ble_stack_basic.a
    BLE_LIBS += $(WPAN_LL_LIB)
    WPAN_SOURCES = $(wildcard $(SDK_DIR)sdk/wpan/*.c)
    BLE_SOURCES = $(wildcard $(SDK_DIR)sdk/ble/*.c)
    # One list, ordered by file name across both trees (a moved file by its
    # old name, WPAN_BLE_ORDER): link order is flash order (see OBJECTS), and
    # this keeps it what it was when every file lived in sdk/ble.
    BLE_OBJS = $(foreach e,$(sort $(foreach n,$(notdir $(basename $(WPAN_SOURCES) $(BLE_SOURCES))),$(call wpan_order_key,$(n))+$(n))),$(foreach n,$(word 2,$(subst +, ,$(e))),$(if $(filter %/sdk/wpan/$(n).c,$(WPAN_SOURCES)),$(BUILD_DIR)/sdk/wpan/$(n).o,$(BUILD_DIR)/sdk/ble/$(n).o)))
    BLE_OBJS += $(BUILD_DIR)/sdk/core/core_ble.o
  else
    $(error BLE is only supported on Core.ST.W5 (STM32WBA55xx) — this build is \
$(MCU_PART). Remove "ble" from config.json, or build for Core.ST.W5.)
  endif
endif

# ---- Thread support (Core.ST.W5 only) ----
#
#   "thread": { "enabled": true, "channel": 15, "panid": "0x1234", ... }
#
# Driven from config.json like BLE (`make THREAD_ENABLED=1` overrides).
# coregen writes the network credentials into coregen/thread_dataset.h.
# OpenThread is ST's prebuilt full-device (FTD) library from STM32CubeWBA
# 1.6.1, compiled with the defines below (ST's Thread_Udp project); its
# headers resolve "radio.h" etc. to the link layer's ot_inc copies, as ST's
# include order does.
ifeq ($(BLE_THREAD),1)
  # (the BLE + Thread block above)
else ifeq ($(THREAD_ENABLED),1)
  ifeq ($(HAS_THREAD_ST),1)
    LDSCRIPT = $(LDSCRIPT_THREAD)
    CFLAGS += -I$(SDK_DIR)sdk/thread/include
    CFLAGS += -I$(SDK_DIR)sdk/thread/config
    CFLAGS += -I$(SDK_DIR)sdk/wpan/include
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/inc
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/inc/ot_inc
    CFLAGS += -I$(SDK_DIR)sdk/wpan/link_layer/config/thread
    CFLAGS += -DTHREAD_ENABLED=1 -DMAC=1 -DOPENTHREAD_FTD=1
    CFLAGS += -D'OPENTHREAD_CONFIG_FILE=<stm32wba-openthread-ftd-config.h>'
    CFLAGS += -D'OPENTHREAD_PROJECT_CORE_CONFIG_FILE=<stm32wba-openthread-ftd-config.h>'
    CFLAGS += -DDISABLE_RFTS_EXT_EVNT_HNDLR=1
    CFLAGS += -D'__PACKED_STRUCT=struct __attribute__((packed))'
    CFLAGS += -D'__PACKED_UNION=union __attribute__((packed))'
    CFLAGS += -include $(SDK_DIR)sdk/wpan/include/cmsis_compiler.h
    WPAN_LL_LIB = $(SDK_DIR)sdk/wpan/lib/LinkLayer_Thread_lib.a
    THREAD_LIBS  = $(SDK_DIR)sdk/thread/lib/stm32wba_ot_ftd_lib.a
    THREAD_LIBS += $(SDK_DIR)sdk/thread/lib/stm32wba_mbedtls_ftd_lib.a
    THREAD_LIBS += $(WPAN_LL_LIB)
    # sdk/wpan minus the BLE-only parts (BLE's memory managers, its AES).
    WPAN_BLE_ONLY = advanced_memory_manager stm32_mm hw_aes
    WPAN_SOURCES = $(filter-out $(foreach n,$(WPAN_BLE_ONLY),%/$(n).c),$(wildcard $(SDK_DIR)sdk/wpan/*.c))
    THREAD_SOURCES = $(wildcard $(SDK_DIR)sdk/thread/*.c)
    THREAD_OBJS  = $(addprefix $(BUILD_DIR)/sdk/wpan/,$(notdir $(WPAN_SOURCES:.c=.o)))
    THREAD_OBJS += $(addprefix $(BUILD_DIR)/sdk/thread/,$(notdir $(THREAD_SOURCES:.c=.o)))
    THREAD_OBJS += $(BUILD_DIR)/sdk/core/core_thread.o
  else
    $(error Thread is only supported on Core.ST.W5 (STM32WBA55xx) — this build is \
$(MCU_PART). Remove "thread" from config.json, or build for Core.ST.W5)
  endif
endif
endif

ASFLAGS = $(CPU_FLAGS) -Wall
ifneq ($(PORT),stm32)
  ASFLAGS += $(PORT_ASFLAGS)
endif

LDFLAGS  = $(CPU_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
# --gc-sections everywhere, BLE included. The BLE build used to opt out, which
# arrived with the very first bring-up commit ("everything needed to compile and
# link") and was never revisited — it reads as a get-it-linking expedient rather
# than a requirement. Keeping it cost 62 KB of dead code in every BLE binary:
# 200116 B -> 136696 B on the same project, a 31% cut, and 31% off the flash
# time that goes with it.
LDFLAGS += -Wl,--gc-sections
ifeq ($(TOOLCHAIN),arm-none-eabi)
  LDFLAGS += -specs=nosys.specs -specs=nano.specs
endif
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
ifneq ($(PORT),stm32)
  LDFLAGS += $(PORT_LDFLAGS)
endif

# ---- Object files ----

C_OBJS   = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
ifeq ($(PORT),stm32)
ASM_OBJS = $(patsubst $(SDK_DIR)%.s, $(BUILD_DIR)/%.o, $(ASM_SOURCES))
else
# A port's startup and sources build into $(BUILD_DIR)/port/ (rules below).
ASM_OBJS  = $(addprefix $(BUILD_DIR)/port/, $(addsuffix .o,$(notdir $(basename $(ASM_SOURCES)))))
PORT_OBJS = $(addprefix $(BUILD_DIR)/port/, $(notdir $(PORT_SOURCES:.c=.o)))
endif
HAL_SOURCES = $(wildcard $(SDK_DIR)sdk/hal/*.c)
HAL_OBJS = $(addprefix $(BUILD_DIR)/sdk/hal/, $(notdir $(HAL_SOURCES:.c=.o)))
GEN_OBJS = $(GEN_SOURCES:.c=.o)
# Core-layer sources compiled into every build (the rest of core_* is
# header-only): core_led.c owns the heartbeat state the SysTick handler
# drives, core_pdm.c the PDM decimator, core_stepper.c the STEP/DIR pulse
# generator's ISR state, core_watchdog.c the running watchdog's timeout
# (which core_power.h sleeps around), core_nvm.c the EEPROM / flash-emulation
# store, core_adc.c the default ADC handle for pad-less internal channels,
# core_reset.c core_reset() and the reset reason core_init() keeps.
# (core_ble.o stays BLE-gated, below.)
CORE_OBJS = $(BUILD_DIR)/sdk/core/core_led.o $(BUILD_DIR)/sdk/core/core_pdm.o \
            $(BUILD_DIR)/sdk/core/core_stepper.o $(BUILD_DIR)/sdk/core/core_scope.o \
            $(BUILD_DIR)/sdk/core/core_watchdog.o $(BUILD_DIR)/sdk/core/core_nvm.o \
            $(BUILD_DIR)/sdk/core/core_adc.o $(BUILD_DIR)/sdk/core/core_reset.o
# Link order = flash order. The linker scripts place prebuilt archives (the ST
# BLE stack, libc, libgcc) first and then every object in the order given here,
# so the project's own sources go LAST: a small edit to a program then changes
# only the last few flash pages, which SWD flash and a Bluetooth delta update
# skip the rest of (tools/page_diff.py measures it). Before them: the SDK, the
# generated objects (coregen), and the startup file, whose Reset_Handler /
# Reset_Continue hold the image's layout and so change with every edit anyway.
ifeq ($(PORT),stm32)
OBJECTS  = $(HAL_OBJS) $(CORE_OBJS)
else
OBJECTS  = $(PORT_OBJS)
endif

ifeq ($(TILES_ENABLED),1)
  TILES_OBJS = $(addprefix $(BUILD_DIR)/tiles/, $(notdir $(TILES_SOURCES:.c=.o)))
  OBJECTS += $(TILES_OBJS)
endif

ifeq ($(PORT),stm32)
ifeq ($(BLE_THREAD),1)
  OBJECTS += $(BLE_THREAD_OBJS)
else ifeq ($(BLE_ENABLED),1)
  OBJECTS += $(BLE_OBJS)
else ifeq ($(THREAD_ENABLED),1)
  OBJECTS += $(THREAD_OBJS)
endif
endif

ifeq ($(WAMR_ENABLED),1)
  OBJECTS += $(WAMR_ALL_OBJS)
endif

OBJECTS += $(GEN_OBJS) $(ASM_OBJS) $(C_OBJS)

# ---- Default goal ----

.DEFAULT_GOAL := all

# ---- Generated headers (coregen) ----

GEN_HEADERS = $(GEN_DIR)/core_caps.h $(GEN_DIR)/core_pads.h $(GEN_DIR)/core_board.h $(GEN_DIR)/core_interfaces.h

ifeq ($(CONFIG_FOUND),1)
  GEN_HEADERS  += $(GEN_DIR)/core_config.h $(GEN_DIR)/core_init.h
  GEN_SOURCES   = $(GEN_DIR)/core_init.c
  ifeq ($(_BLE_CFG),1)
    GEN_HEADERS += $(GEN_DIR)/ble_contract.h
    GEN_SOURCES += $(GEN_DIR)/ble_contract.c
  endif
  # The network credentials (core_thread.c falls back to the same defaults
  # when a `make THREAD_ENABLED=1` build's config has no "thread" block).
  ifeq ($(_THREAD_CFG),1)
    GEN_HEADERS += $(GEN_DIR)/thread_dataset.h
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
# Also remade when TILES_OVERLAY differs from the one it was generated with
# (TILES_OVERLAY_GEN, written by coregen), since it names overlay drivers by path.
# The Core's core.mk (and its port's PORT_MK) count too: a changed Core regenerates.
# So do its status file, features.json and tools/coregen/core_caps.py (core_caps.h).
# So do coregen's back ends: tools/coregen/ports/*.py here, and the Core's
# port's back end in an overlay (<overlay>/tools/coregen/ports/$(PORT).py).
COREGEN_OVERLAY_CHANGED := $(if $(filter 1,$(TILES_ENABLED)),$(if $(filter-out $(TILES_OVERLAY_GEN),$(TILES_OVERLAY))$(filter-out $(TILES_OVERLAY),$(TILES_OVERLAY_GEN)),FORCE_COREGEN))
.PHONY: FORCE_COREGEN
$(GEN_DIR)/core_drivers.mk: $(TILE_JSON) $(if $(CONFIG_FOUND),$(CONFIG_JSON)) $(SDK_DIR)tools/coregen/coregen.py $(SDK_DIR)tools/coregen/templates/*.j2 \
                           $(SDK_DIR)tools/coregen/core_caps.py $(CORE_STATUS) $(SDK_DIR)sdk/status/features.json \
                           $(wildcard $(SDK_DIR)tools/coregen/ports/*.py) \
                           $(wildcard $(foreach o,$(patsubst %/,%,$(TILES_OVERLAY)),$(o)/tools/coregen/ports/$(PORT).py)) \
                           $(CORE_MK) $(PORT_MK) \
                           $(wildcard $(foreach o,$(patsubst %/,%,$(TILES_OVERLAY)),$(o)/drivers/drivers.json)) \
                           $(COREGEN_OVERLAY_CHANGED)
	@mkdir -p "$(GEN_DIR)"
	@echo "  GEN   $(TILE)"
	$(Q)$(COREGEN) "$(TILE_JSON)" "$(GEN_DIR)" $(if $(PORT),--port $(PORT)) --status "$(CORE_STATUS)" $(COREGEN_FLAGS) $(COREGEN_QUIET)

# Stamp prevents re-running coregen for each header file target.
GEN_STAMP = $(GEN_DIR)/.coregen.stamp
$(GEN_STAMP): $(GEN_DIR)/core_drivers.mk
	$(Q)touch $(GEN_STAMP)

$(GEN_HEADERS): $(GEN_STAMP)

# The WAMR natives (sdk/wamr/studio_natives.c) include core_ headers, which
# include coregen's core_caps.h: generate first (wamr.mk's own rule is read
# before GEN_HEADERS is set).
ifeq ($(WAMR_ENABLED),1)
$(filter $(BUILD_DIR)/sdk/wamr/%,$(WAMR_OBJS)): $(GEN_HEADERS)
endif

.PHONY: generate
generate: $(GEN_STAMP)

# ---- Rules ----

.PHONY: all clean distclean flash flash-coreprobe probe-power-check size

all: $(TARGET).bin $(TARGET).hex size

$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) $(BLE_THREAD_LIBS) $(PORT_LIBS)
	@echo "  LD    $(notdir $@)"
ifneq ($(PORT),stm32)
	$(Q)$(CC) $(OBJECTS) $(PORT_LIBS) $(LDFLAGS) -o $@
else ifeq ($(BLE_THREAD),1)
	@# OpenThread, mbedTLS and the link layer call into each other: one group.
	$(Q)$(CC) $(OBJECTS) -Wl,--start-group $(BLE_THREAD_LIBS) -Wl,--end-group $(LDFLAGS) -o $@
	$(Q)$(PYTHON) "$(SDK_DIR)tools/check_ota_boot.py" --objdump $(OBJDUMP) $@ || { rm -f $@; exit 1; }
else ifeq ($(BLE_ENABLED),1)
	$(Q)$(CC) $(OBJECTS) $(BLE_LIBS) $(LDFLAGS) -o $@
	@# The BLE update's copier must not reach outside flash page 0 / its SRAM
	@# copy (sdk/ble/ble_update_boot.c): a failure here is a broken install
	@# path, caught before it ships (docs/ble-update-protocol.md §8.2).
	$(Q)$(PYTHON) "$(SDK_DIR)tools/check_ota_boot.py" --objdump $(OBJDUMP) $@ || { rm -f $@; exit 1; }
else ifeq ($(THREAD_ENABLED),1)
	@# OpenThread, mbedTLS and the link layer call into each other: one group.
	$(Q)$(CC) $(OBJECTS) -Wl,--start-group $(THREAD_LIBS) -Wl,--end-group $(LDFLAGS) -o $@
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

ifeq ($(PORT),stm32)
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

$(BUILD_DIR)/sdk/core/core_adc.o: $(SDK_DIR)sdk/core/core_adc.c $(GEN_HEADERS)
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

$(BUILD_DIR)/sdk/core/core_scope.o: $(SDK_DIR)sdk/core/core_scope.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/core/core_watchdog.o: $(SDK_DIR)sdk/core/core_watchdog.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/core/core_nvm.o: $(SDK_DIR)sdk/core/core_nvm.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/core/core_reset.o: $(SDK_DIR)sdk/core/core_reset.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/device/%.o: $(SDK_DIR)sdk/device/%.s
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  AS    $(notdir $<)"
	$(Q)$(AS) $(ASFLAGS) -c $< -o $@
else
# A port's sources and startup (PORT_SOURCES, STARTUP): one rule set per
# source directory, objects flat in $(BUILD_DIR)/port/.
define PORT_SOURCE_RULE
$$(BUILD_DIR)/port/%.o: $(1)%.c $$(GEN_HEADERS)
	$$(Q)mkdir -p $$(dir $$@)
	$$(LOG) "  CC    $$(notdir $$<)"
	$$(Q)$$(CC) $$(CFLAGS) -c "$$<" -o $$@
$$(BUILD_DIR)/port/%.o: $(1)%.s
	$$(Q)mkdir -p $$(dir $$@)
	$$(LOG) "  AS    $$(notdir $$<)"
	$$(Q)$$(AS) $$(ASFLAGS) -c "$$<" -o $$@
$$(BUILD_DIR)/port/%.o: $(1)%.S
	$$(Q)mkdir -p $$(dir $$@)
	$$(LOG) "  AS    $$(notdir $$<)"
	$$(Q)$$(AS) $$(ASFLAGS) -c "$$<" -o $$@
endef
$(foreach d,$(sort $(dir $(PORT_SOURCES) $(ASM_SOURCES))),$(eval $(call PORT_SOURCE_RULE,$(d))))
endif

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

# One rule per overlay drivers/ directory that core_drivers.mk names.
define TILES_OVERLAY_DRIVER_RULE
$$(BUILD_DIR)/tiles/%.o: $(1)%.c $$(GEN_HEADERS)
	$$(Q)mkdir -p $$(dir $$@)
	$$(LOG) "  CC    $$(notdir $$<)"
	$$(Q)$$(CC) $$(CFLAGS) -c "$$<" -o $$@
endef
$(foreach d,$(sort $(dir $(TILES_OVERLAY_DRIVERS))),$(eval $(call TILES_OVERLAY_DRIVER_RULE,$(d))))
endif

# BLE and shared radio (wpan) sources (relaxed warnings — vendor headers are noisy)
ifeq ($(PORT),stm32)
ifeq ($(BLE_ENABLED),1)
$(BUILD_DIR)/sdk/ble/%.o: $(SDK_DIR)sdk/ble/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -c $< -o $@

endif
ifneq ($(filter 1,$(BLE_ENABLED) $(THREAD_ENABLED)),)
$(BUILD_DIR)/sdk/wpan/%.o: $(SDK_DIR)sdk/wpan/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -c $< -o $@
endif

# Thread glue (sdk/thread) and core_thread.c: our code, full warnings.
ifeq ($(THREAD_ENABLED),1)
$(BUILD_DIR)/sdk/thread/%.o: $(SDK_DIR)sdk/thread/%.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/sdk/core/core_thread.o: $(SDK_DIR)sdk/core/core_thread.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@
endif

ifeq ($(BLE_ENABLED),1)
# ble_update.c calls the copier's su_ota_* functions, which run from SRAM
# (.ota_ram), out of BL range. Plain calls get linker veneers, which ld puts at
# the end of .text, behind the project's code, so every program edit moved them
# and changed this file's flash page too. Long calls load the copier's fixed
# SRAM address instead (see OBJECTS for why the order matters).
$(BUILD_DIR)/sdk/ble/ble_update.o: CFLAGS += -mlong-calls

# core_ble.c — in sdk/core/ but only compiled for BLE builds
$(BUILD_DIR)/sdk/core/core_ble.o: $(SDK_DIR)sdk/core/core_ble.c $(GEN_HEADERS)
	$(Q)mkdir -p $(dir $@)
	$(LOG) "  CC    $(notdir $<)"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@
endif
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

# ---- STM32 flash and debug targets ----
# OpenOCD / CubeProgrammer, USB DFU, serial update, the DFU bootloader, gdb and
# SWO: PORT = stm32 only. Another port's fragment defines its own `flash`.
ifeq ($(PORT),stm32)

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
#
# The firmware triggers on DTR *falling* while the line is at 1200 baud
# (sdk/hal/hal_usb_cdc.c, the 1200-baud-touch check). So DTR has to be asserted
# first and then dropped. `stty ... 1200 hupcl` only drops it at close and does
# not reliably produce that edge — measured 2026-09-23 on a Core.ST.L4, where
# stty left the board running normally and dfu-util then sat waiting. pyserial
# drives the edge explicitly and entered DFU in about a second.

# Wait for the ROM bootloader to show up, and say so plainly if it never does.
# Without this, a touch that silently missed leaves dfu-util waiting, which
# reads like a slow flash instead of a no-op.
define DFU_WAIT
@_n=0; \
echo "  DFU   Waiting for DFU device..."; \
while [ $$_n -lt 15 ]; do \
	if dfu-util -l 2>/dev/null | grep -q "0483:df11"; then break; fi; \
	sleep 1; _n=$$((_n+1)); \
done; \
if ! dfu-util -l 2>/dev/null | grep -q "0483:df11"; then \
	echo "  DFU   No DFU device after 15 s — the board is not in the bootloader."; \
	echo "  DFU   The touch needs DTR to fall while the port is at 1200 baud; the"; \
	echo "  DFU   stty fallback often misses that edge."; \
	echo "  DFU     pip install pyserial     (then retry)"; \
	echo "  DFU     or enter DFU by hand: hold BOOT0 while plugging in USB"; \
	exit 1; \
fi
endef

# Pick the Core's port by USB identity, not by globbing /dev. A CoreProbe is a
# Core.ST.L4 running probe firmware: same vendor id, its own CDC port, and it
# matches the glob. A hackathon participant flashed a Core image onto one that
# way (2026-10). Prints the port, or COREPROBE when the only match is a probe.
CORE_PORT_PY = import sys; from serial.tools import list_ports; \
c=[p for p in list_ports.comports() if p.vid==0x1209]; \
pr=[p for p in c if p.pid==0xDA01 or (p.product or "").startswith("CoreProbe")]; \
ok=[p for p in c if p not in pr]; \
print(ok[0].device if len(ok)==1 else ("COREPROBE" if pr and not ok else ""))

# The probe's own port, for a deliberate probe update (COREPROBE_UPDATE=1).
PROBE_PORT_PY = import sys; from serial.tools import list_ports; \
pr=[p for p in list_ports.comports() if p.vid==0x1209 \
and (p.pid==0xDA01 or (p.product or "").startswith("CoreProbe"))]; \
print(pr[0].device if len(pr)==1 else "")

# Assert DTR, drop it, close — the edge the firmware watches for.
DFU_TOUCH_PY = import serial,sys,time; p=serial.Serial(sys.argv[1],1200); p.dtr=True; time.sleep(0.25); p.dtr=False; time.sleep(0.25); p.close()

ifeq ($(HOST_OS),windows)
# Windows has no /dev entry for a USB-CDC port, so there is nothing to glob and
# stty cannot address COM ports. Do the touch through pyserial when the user
# names the port, and otherwise say plainly that this step is manual.
define DFU_TOUCH
@if [ -n "$(SERIAL_PORT)" ]; then \
	echo "  DFU   Triggering reboot via 1200-baud touch on $(SERIAL_PORT)..."; \
	$(PYTHON) -c "$(DFU_TOUCH_PY)" "$(SERIAL_PORT)" \
		|| { echo "  DFU   Touch failed — it needs pyserial (pip install pyserial)"; \
		     echo "  DFU   or the port name is wrong (Device Manager -> Ports (COM & LPT))."; }; \
else \
	echo "  DFU   Windows: no automatic 1200-baud touch — COM ports have no /dev entry."; \
	echo "  DFU     name the port:  make flash-dfu SERIAL_PORT=COM5   (needs pyserial)"; \
	echo "  DFU     or enter DFU by hand: hold BOOT0 while plugging in USB"; \
	echo "  DFU   dfu-util on Windows also needs the WinUSB driver bound to the device"; \
	echo "  DFU   (install it once with Zadig). 'make doctor' checks the rest."; \
fi
$(DFU_WAIT)
endef
else
define DFU_TOUCH
@TTY="$(SERIAL_PORT)"; \
if [ -z "$$TTY" ]; then TTY=$$($(PYTHON) -c '$(CORE_PORT_PY)' 2>/dev/null \
	|| ls $(SERIAL_GLOB) 2>/dev/null | head -1); fi; \
if [ "$$TTY" = "COREPROBE" ]; then \
	if [ "$(COREPROBE_UPDATE)" = "1" ]; then \
		TTY=$$($(PYTHON) -c '$(PROBE_PORT_PY)' 2>/dev/null); \
		if [ -z "$$TTY" ]; then \
			echo "  DFU   COREPROBE_UPDATE=1 but the probe's port could not be resolved"; \
			echo "  DFU   (more than one probe attached? name it with SERIAL_PORT=)."; \
			exit 1; \
		fi; \
		echo "  DFU   Updating the CoreProbe itself (COREPROBE_UPDATE=1)."; \
	else \
		echo "  DFU   The only Bergsonne board on USB is a CoreProbe, not a Core."; \
		echo "  DFU   A CoreProbe is a Core.ST.L4 running probe firmware, so it shares the"; \
		echo "  DFU   USB vendor id and looks like a Core here. Flashing this image onto it"; \
		echo "  DFU   would replace the probe firmware."; \
		echo "  DFU   If you meant to update the probe, build from"; \
		echo "  DFU   projects/CoreProbe/firmware, whose Makefile passes COREPROBE_UPDATE=1."; \
		exit 1; \
	fi; \
fi; \
if [ -n "$$TTY" ]; then \
	echo "  DFU   Triggering reboot via 1200-baud touch on $$TTY..."; \
	$(PYTHON) -c "$(DFU_TOUCH_PY)" "$$TTY" 2>/dev/null \
		|| { echo "  DFU   pyserial missing — falling back to stty, which often misses"; \
		     echo "  DFU   the DTR edge. pip install pyserial if this does not work."; \
		     stty $(STTY_DEV) "$$TTY" 1200 hupcl; }; \
else \
	echo "  DFU   No serial port matched $(SERIAL_GLOB) — expecting the board to be"; \
	echo "  DFU   in DFU already (a virgin board arrives that way)."; \
fi
$(DFU_WAIT)
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

# ---- Flash over USB serial (no bootloader, no driver) ----
# Hands the running Core to its SRAM flasher over the CDC port (Core.ST.L4,
# Core.ST.L4.2 and Core.ST.H5, ROM-DFU layout). No dfu-util, and on Windows no
# Zadig / WinUSB. Needs the firmware on the Core to be built with serial update;
# if it predates it, this says so — run `make flash-dfu` once. SERIAL_PORT picks
# a port when more than one Core is plugged in. The DEV_ID stops an image built
# for one Core from being written to another (core.mk's SERIAL_UPDATE_DEV_ID,
# the part's DBGMCU device id). Protocol: docs/serial-update-protocol.md.
.PHONY: flash-serial
flash-serial: $(TARGET).bin
	@$(PYTHON) "$(SDK_DIR)tools/serial_update.py" $(if $(SERIAL_PORT),--port "$(SERIAL_PORT)") $(if $(SERIAL_UPDATE_DEV_ID),--dev-id $(SERIAL_UPDATE_DEV_ID)) "$<"

endif

# ---- Reset over USB serial ----
# Restarts the running program (core_reset()) through the remote reset frame
# the Core's USB serial port takes (docs/serial-update-protocol.md §10). No
# build, no flash. Exit 2: the firmware predates the reset frame.
.PHONY: reset-serial
reset-serial:
	@$(PYTHON) "$(SDK_DIR)tools/serial_update.py" --reset $(if $(SERIAL_PORT),--port "$(SERIAL_PORT)")

ifeq ($(PORT),stm32)

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

endif

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
	_check $(PREFIX)gcc required "$(GCC_HINT)"; \
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
