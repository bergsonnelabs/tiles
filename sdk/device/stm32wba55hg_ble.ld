/* Linker script for STM32WBA55HG — BLE builds */
/* 1MB Flash, 128KB SRAM */
/* Larger stack + init_array sections required by BLE stack binary */

/* The top 32 KB are reserved, and FLASH ends below them so no image can
 * overlap them (they survive reflashing: flash-serial, Studio, probe-rs and
 * CubeProgrammer erase only the pages an image covers):
 *   0x080F8000-0x080F9FFF  page 124      board key + BLE-update install marker
 *                                        (docs/ble-update-protocol.md §7)
 *   0x080FA000-0x080FDFFF  pages 125-126  core_nvm flash emulation (core_nvm.h)
 *   0x080FE000-0x080FFFFF  page 127       BLE bond store (sdk/ble/nvm_stub.c)
 * Both scripts (BLE and not) reserve all of them, so switching a project's
 * BLE setting never overwrites any. */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 992K   /* 1024K - update page - core_nvm - bonds */
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* core_nvm checks this at run time: a custom linker script without the
 * reservation fails with CORE_NVM_ERR_LAYOUT instead of erasing code. */
__core_nvm_start = 0x080FA000;
__core_nvm_end   = 0x080FE000;

/* The BLE update's reserved page (sdk/serial_update/su_ota.h SU_OTA_RES_OFF). */
__ble_update_page = 0x080F8000;

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

    /* BLE update (docs/ble-update-protocol.md §8): everything that must
     * survive an install being copied over this app lives in page 0, which is
     * rewritten last. Reset_Handler and the boot hook run from flash here; the
     * copier is linked to run from SRAM and stored right behind them. */
    .ota_boot :
    {
        . = ALIGN(4);
        KEEP(*(.text.Reset_Handler))
        KEEP(*(.ota_boot .ota_boot.*))
        . = ALIGN(4);
    } >FLASH

    /* The copier and its data: the boot hook copies this to SRAM at every
     * boot, before .data / .bss exist. The vector table goes first (VTOR
     * needs 128-byte alignment). */
    .ota_ram :
    {
        . = ALIGN(128);
        __ota_ram_start = .;
        KEEP(*(.ota_ram.vtab))
        *(.ota_ram .ota_ram.*)
        . = ALIGN(4);
        __ota_ram_end = .;
    } >SRAM AT>FLASH
    __ota_ram_load = LOADADDR(.ota_ram);

    /* Static constructors/destructors — required by BLE stack binary. Ahead
     * of .text so their addresses, which libc's __libc_init_array holds,
     * don't move when the program grows (see .text). */
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
     *   2. prebuilt archives: libc, libgcc, the ST BLE stack and link layer
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
        *.a:*(.data .data.*)            /* archives first, project last (see .text) */
        *(.data .data.*)
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
        *.a:*(.bss .bss.* COMMON)       /* archives first, project last (see .text) */
        *(.bss .bss.*)
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

/* The copier must sit entirely in page 0 (it is what survives the copy). */
ASSERT(__ota_ram_load + (__ota_ram_end - __ota_ram_start) <= ORIGIN(FLASH) + 8K,
       "BLE update: the page-0 boot section (.ota_boot + .ota_ram) overflows flash page 0")
ASSERT(__ota_ram_end > __ota_ram_start, "BLE update: the copier (.ota_ram) is missing")

/* Where this image ends in flash: the BLE update stages above it. */
__flash_image_end = LOADADDR(.data) + SIZEOF(.data);
