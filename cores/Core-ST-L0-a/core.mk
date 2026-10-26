# Core-ST-L0-a: STM32L011. Public name Core.ST.L0 resolves here.
#
# core.mk contract. The SDK Makefile includes cores/<stem>/core.mk (or an
# overlay's <overlay>/cores/<stem>/core.mk, for a Core that is not public) for
# TILE=<stem>, at the point where the per-Core settings used to be hard-coded.
# A core.mk sets, with recursive `=`:
#   PORT, ARCH, TOOLCHAIN       SDK port (stm32), CPU architecture (cortex-m)
#                               and toolchain (arm-none-eabi). The STM32 parts
#                               of the Makefile apply only to PORT = stm32.
#   MCU_FAMILY, MCU_PART        HAL family directory and the -D part define
#   PROBE_RS_CHIP, OPENOCD_CFG  debug probe target
#   CPU, FPU, FLOAT_ABI         -mcpu / -mfpu / -mfloat-abi (leave FPU unset
#                               for no FPU: the Makefile tests `ifdef FPU`)
#   LDSCRIPT, STARTUP           link script and startup file
#   LDSCRIPT_ROMDFU             link script for "bootloader": "rom" (app at
#                               0x08000000): the standard one where that
#                               serves (stm32; ROM_DFU=1 refuses without it)
#   LDSCRIPT_APP                optional: link script for the parked custom-
#                               bootloader layout (app at 0x08002000);
#                               BOOTLOADER=1 refuses without it (stm32)
#   SERIAL_UPDATE_DEV_ID        optional: the part's device id, which `make
#                               flash-serial` checks (Cores with the
#                               serial-update flasher)
#   HAS_BLE_ST, HAS_THREAD_ST   optional: 1 = ST's BLE / Thread stacks build
#                               for this Core, with LDSCRIPT_BLE,
#                               LDSCRIPT_THREAD and LDSCRIPT_BLE_THREAD;
#                               unset, "ble" / "thread" in config.json is an
#                               error (stm32)
#   FLASH_TOOL                  optional; `make flash` defaults to openocd
#   PORT_MK                     optional: a port's make fragment, included
#                               right after this file (non-stm32 ports only)
# Paths are $(SDK_DIR)-relative here; an overlay Core uses $(CORE_DIR) (this
# file's directory) for its own files. Overlay Cores ship the same file.

PORT          = stm32
ARCH          = cortex-m
TOOLCHAIN     = arm-none-eabi

MCU_FAMILY    = stm32l0xx
MCU_PART      = STM32L011xx
PROBE_RS_CHIP = STM32L011K4
CPU           = cortex-m0plus
# No FPU: FPU and FLOAT_ABI stay unset.
LDSCRIPT      = $(SDK_DIR)sdk/device/stm32l011e4.ld
STARTUP       = $(SDK_DIR)sdk/device/stm32l0xx/startup_stm32l011xx.s
# No USB, so no USB DFU: "bootloader": "rom" builds the standard layout.
LDSCRIPT_ROMDFU = $(SDK_DIR)sdk/device/stm32l011e4.ld
OPENOCD_CFG   = $(SDK_DIR)sdk/debug/stm32l0.cfg
