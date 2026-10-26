# Core-ST-H5-a: STM32H523. Public name Core.ST.H5 resolves here.
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

MCU_FAMILY    = stm32h5xx
MCU_PART      = STM32H523xx
PROBE_RS_CHIP = STM32H523HEYxT
CPU           = cortex-m33
FPU           = fpv5-sp-d16
FLOAT_ABI     = hard
LDSCRIPT      = $(SDK_DIR)sdk/device/stm32h523he.ld
STARTUP       = $(SDK_DIR)sdk/device/stm32h5xx/startup_stm32h523xx.s
OPENOCD_CFG   = $(SDK_DIR)sdk/debug/stm32h5.cfg
