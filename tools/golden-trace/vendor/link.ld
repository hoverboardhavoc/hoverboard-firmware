/* The link script for a vendor-body ELF (golden-trace, vendor.rs). The emulator enters gt_entry
 * directly and loads every segment at its run address, so there is no vector table and no startup
 * copy: code and constants in flash, data and zero-initialised data in RAM. */

ENTRY(gt_entry)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

SECTIONS
{
    .text : {
        KEEP(*(.text.gt_entry))
        *(.text*)
        *(.rodata*)
    } > FLASH

    .data : {
        *(.data*)
    } > RAM

    .bss (NOLOAD) : {
        *(.bss*)
        *(COMMON)
    } > RAM

    /DISCARD/ : {
        *(.ARM.exidx*)
    }
}
