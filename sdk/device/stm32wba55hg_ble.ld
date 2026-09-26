/* Linker script for STM32WBA55HG — BLE builds */
/* 1MB Flash, 128KB SRAM */
/* Larger stack + init_array sections required by BLE stack binary */

/* The top 24 KB are reserved, and FLASH ends below them so no image can
 * overlap them (they survive reflashing: flash-serial, Studio, probe-rs and
 * CubeProgrammer erase only the pages an image covers):
 *   0x080FA000-0x080FDFFF  pages 125-126  core_nvm flash emulation (core_nvm.h)
 *   0x080FE000-0x080FFFFF  page 127       BLE bond store (sdk/ble/nvm_stub.c)
 * Both scripts (BLE and not) reserve both, so switching a project's BLE
 * setting never overwrites either. */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1000K  /* 1024K - core_nvm - bonds */
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* core_nvm checks this at run time: a custom linker script without the
 * reservation fails with CORE_NVM_ERR_LAYOUT instead of erasing code. */
__core_nvm_start = 0x080FA000;
__core_nvm_end   = 0x080FE000;

/* Entry point */
ENTRY(Reset_Handler)

/* Highest address of the stack — top of SRAM */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* BLE stack uses deep call chains — 12KB stack minimum */
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x3000;

SECTIONS
{
    /* Vector table and code go into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM exception handling — needed by some C library functions */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    /* Static constructors/destructors — required by BLE stack binary */
    .preinit_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* Used by startup to initialize .data */
    _sidata = LOADADDR(.data);

    /* Initialized data — loaded from FLASH, lives in SRAM */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >SRAM AT> FLASH

    /* Uninitialized data — zero-filled in SRAM */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >SRAM

    /* Persistent RAM — NOT zero-filled by startup (it lives past _ebss), so its
     * contents survive a warm system reset (lost only on power loss). Use for crash
     * logs / breadcrumbs that must outlive the reset. Place vars with
     * __attribute__((section(".noinit"))). */
    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        _snoinit = .;
        *(.noinit)
        *(.noinit*)
        . = ALIGN(4);
        _enoinit = .;
    } >SRAM

    /* Heap grows up, stack grows down — check they don't collide */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >SRAM

    /* Discard debug sections from standard libraries */
    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
