/* Linker script for STM32L422TB — ROM DFU mode
 *
 * Application at 0x08000000, up to 124KB (no custom bootloader; the top 4KB
 * is core_nvm, below).
 * DFU updates use the factory ROM bootloader in system memory.
 *
 * The last 16 bytes of SRAM are reserved for the DFU magic value
 * (noinit region at 0x20009FF0). The startup code does not zero this
 * area, so it survives system reset for bootloader signaling. */

/* The last two 2 KB pages, 0x0801F000-0x0801FFFF, are core_nvm's flash
 * emulation (sdk/core/core_nvm.h). FLASH ends below them so no image can
 * overlap them: they survive reflashing, since flash-serial, DFU and SWD
 * tools erase only the pages an image covers. */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 124K   /* 128K - core_nvm */
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 40K - 16
}

/* core_nvm checks this at run time: a custom linker script without the
 * reservation fails with CORE_NVM_ERR_LAYOUT instead of erasing code. */
__core_nvm_start = 0x0801F000;
__core_nvm_end   = 0x08020000;

/* Entry point */
ENTRY(Reset_Handler)

/* Highest address of the stack — top of usable SRAM (below noinit) */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Minimum heap and stack sizes */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

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
