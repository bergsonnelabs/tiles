/* Linker script for STM32H523HE — application behind bootloader
 *
 * Bootloader occupies first 8KB (sector 0): 0x08000000–0x08001FFF
 * Application starts at 0x08002000 (504KB available)
 * Last 16 bytes of SRAM reserved for DFU magic (noinit region) */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08002000, LENGTH = 512K - 8K - 16K   /* top 16 KB: core_nvm */
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 272K - 16
}

/* As in stm32h523he.ld: core_nvm's sectors above the image. */
__core_nvm_start = 0x0807C000;

ENTRY(Reset_Handler)

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Empty in C builds, but crtbegin's frame_dummy holds its address: here,
     * ahead of .text, rather than as an orphan behind it, where it would move. */
    .eh_frame :
    {
        KEEP(*(.eh_frame))
    } >FLASH

    /* Link order = flash order. The project's own code goes LAST, so a small
     * program edit changes only the last few flash pages, and SWD flash (and
     * a Bluetooth delta update) skip the rest; tools/page_diff.py counts them.
     *   1. .init / .fini (crti/crtn: fixed code)
     *   2. prebuilt archives: libc, libgcc
     *   3. every object, in command-line order (Makefile, OBJECTS): the SDK,
     *      the generated objects (coregen), the startup file, whose reset code
     *      holds the image's layout, and then the project's own sources.
     * Each file's read-only data travels with its code, so the project's
     * constants and strings land at the end too. Nothing is matched by path:
     * an object passed in another order still links, just less well placed. */
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.init))
        KEEP(*(.fini))
        *.a:*(.text .text.* .rodata .rodata.*)
        *(.text .text.* .rodata .rodata.* .glue_7 .glue_7t)
        *(.text*)                       /* odd names (.text_foo), if any */

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* Anything the .text rule didn't take (nothing, in practice). */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *.a:*(.data .data.*)            /* archives first, project last (see .text) */
        *(.data .data.*)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >SRAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *.a:*(.bss .bss.* COMMON)       /* archives first, project last (see .text) */
        *(.bss .bss.*)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >SRAM

    .noinit (NOLOAD) : { . = ALIGN(4); *(.noinit) *(.noinit*) . = ALIGN(4); } >SRAM

    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >SRAM

    /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
