/* Linker script for the BLE-stress C firmware on the bench master GD32F103C8 (medium density:
 * 64 KiB flash @ 0x08000000, 20 KiB SRAM @ 0x20000000). Mirrors the FLASH/SRAM origins of
 * runtime-hal/harness/build_assets/gd-spl/gd32f10x/link.ld; the LENGTHs are the real part's, and the
 * SECTIONS carry the data-init/bss symbols the C startup needs (the harness ld has no real crt). */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    _sidata = LOADADDR(.data);

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT > FLASH

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM

    . = ALIGN(4);
    _end = .;

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.attributes)
    }
}
