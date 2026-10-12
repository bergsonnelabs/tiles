/* Linker script for STM32WBA55HG — Thread builds (THREAD_ENABLED) */
/* 1MB Flash, 128KB SRAM */
/* The plain script (stm32wba55hg.ld) plus the OpenThread settings pages and a
 * larger stack. */

/* The top 48 KB are reserved, and FLASH ends below them so no image can
 * overlap them (they survive reflashing: Studio, probe-rs and CubeProgrammer
 * erase only the pages an image covers):
 *   0x080F4000-0x080F7FFF  pages 122-123  OpenThread settings (sdk/thread/
 *                                        thread_settings.c): dataset, frame
 *                                        counters, network info
 *   0x080F8000-0x080F9FFF  page 124      board key + BLE-update install marker
 *                                        (docs/ble-update-protocol.md §7)
 *   0x080FA000-0x080FDFFF  pages 125-126  core_nvm flash emulation (core_nvm.h)
 *   0x080FE000-0x080FFFFF  page 127       BLE bond store (sdk/ble/nvm_stub.c)
 * Pages 124-127 are reserved by every W5 script, as before. Pages 122-123
 * by this one and stm32wba55hg_ble_thread.ld (which also keeps them out of
 * the BLE update's staging area): the plain and BLE scripts end FLASH at
 * 992K, and a BLE update from such an image may stage over them (Thread
 * settings are then lost, nothing else). */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 976K   /* 1024K - Thread settings - update page - core_nvm - bonds */
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* core_nvm checks this at run time: a custom linker script without the
 * reservation fails with CORE_NVM_ERR_LAYOUT instead of erasing code. */
__core_nvm_start = 0x080FA000;
__core_nvm_end   = 0x080FE000;

/* The BLE update's reserved page (sdk/serial_update/su_ota.h SU_OTA_RES_OFF). */
__ble_update_page = 0x080F8000;

/* OpenThread settings (sdk/thread/thread_settings.c; core_nvm.c's NMI handler
 * absorbs ECC errors from a torn write here). */
__thread_settings_start = 0x080F4000;
__thread_settings_end   = 0x080F8000;

/* Entry point */
ENTRY(Reset_Handler)

/* Highest address of the stack — top of SRAM */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* OpenThread and mbedTLS have deep call chains (ST's Thread applications
 * reserve 4 KB); 8 KB here for margin. OpenThread keeps its own heap and
 * message pool in .bss, so the C heap stays small. */
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x2000;

SECTIONS
{
    /* Vector table and code go into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Static constructors/destructors (the startup calls __libc_init_array).
     * Ahead of .text so their addresses, which libc's __libc_init_array
     * holds, don't move when the program grows (see .text). */
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
     *   2. prebuilt archives: libc, libgcc, OpenThread, mbedTLS, the link layer
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

    /* Persistent RAM — NOT zero-filled by startup (lives past _ebss), so its contents
     * survive a warm system reset (lost only on power loss). For crash logs / breadcrumbs
     * that must outlive a reset. Place vars with __attribute__((section(".noinit"))). */
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

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= __thread_settings_start,
       "Thread: FLASH overlaps the OpenThread settings pages")
