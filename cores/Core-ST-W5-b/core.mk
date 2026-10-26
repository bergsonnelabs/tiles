# Core-ST-W5-b: STM32WBA55. Public name Core.ST.W5 resolves here.
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

MCU_FAMILY    = stm32wbaxx
MCU_PART      = STM32WBA55xx
PROBE_RS_CHIP = STM32WBA55CG
CPU           = cortex-m33
FPU           = fpv5-sp-d16
FLOAT_ABI     = hard
LDSCRIPT      = $(SDK_DIR)sdk/device/stm32wba55hg.ld
STARTUP       = $(SDK_DIR)sdk/device/stm32wbaxx/startup_stm32wba55xx.s
# No USB, so no USB DFU: "bootloader": "rom" builds the standard layout.
LDSCRIPT_ROMDFU = $(SDK_DIR)sdk/device/stm32wba55hg.ld
HAS_BLE_ST    = 1
HAS_THREAD_ST = 1
LDSCRIPT_BLE  = $(SDK_DIR)sdk/device/stm32wba55hg_ble.ld
LDSCRIPT_THREAD = $(SDK_DIR)sdk/device/stm32wba55hg_thread.ld
LDSCRIPT_BLE_THREAD = $(SDK_DIR)sdk/device/stm32wba55hg_ble_thread.ld
OPENOCD_CFG   = $(SDK_DIR)sdk/debug/stm32wba.cfg
FLASH_TOOL    = cubeprog
