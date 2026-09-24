/* Serial-update flasher — STM32L422, runs from SRAM.
 *
 * The whole image (vectors, code, rodata, data) is copied by the app to
 * SU_FLASHER_BASE and executed there. .bss follows and is zeroed by su_main.
 * SRAM map: su_protocol.h. The asserts keep the image clear of the handoff
 * block the app writes at 0x20007000. */

MEMORY
{
    IMAGE (rwx) : ORIGIN = 0x20000000, LENGTH = 0x7000
}

ENTRY(su_main)

SECTIONS
{
    .vectors : { KEEP(*(.vectors)) } > IMAGE

    .text : {
        . = ALIGN(4);
        *(.text .text*)
        *(.rodata .rodata*)
        . = ALIGN(4);
    } > IMAGE

    .data : {
        . = ALIGN(4);
        *(.data .data*)
        . = ALIGN(4);
        _image_end = .;
    } > IMAGE

    .bss (NOLOAD) : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss* COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > IMAGE

    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) }
}

ASSERT(ORIGIN(IMAGE) == 0x20000000, "flasher must be linked at SU_FLASHER_BASE")
ASSERT(_ebss <= 0x20007000, "flasher image + .bss overlaps the handoff block")
ASSERT(su_vectors == 0x20000000, "vector table must be first")
