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
OPENOCD_CFG   = $(SDK_DIR)sdk/debug/stm32wba.cfg
FLASH_TOOL    = cubeprog
