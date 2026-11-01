/* The per-chip images' layout (specs/hal.md, sections 5 and 6). build.rs fills the two @..@ lengths
 * from the chip facts the hal crate's build script publishes for the compiled chip (its `links`
 * metadata: flash size, page size, SRAM size), so the numbers have one owner. The universal image
 * links memory.x instead.
 *
 * What differs from memory.x, and why:
 * - FLASH is the chip's flash less the store's two pages at the top (written by the FMC at absolute
 *   addresses the store derives from the same descriptor, `store::geometry`), so an image that would
 *   overlap the store fails at LINK.
 * - RAM is the chip's SRAM, not the smallest part's: a per-chip image is linked to its own part.
 * - There is no RAM vector table and no detect-probe table: the image runs from its chip's flash
 *   vector table (`hal`, cortex-m-rt's `device` feature) and never writes VTOR. So there is no 256 B
 *   alignment gap after the mailbox carve to fill, and the DMA ring lives in cortex-m-rt's `.uninit`
 *   like the other uninitialised statics.
 *
 * Kept from memory.x unchanged: the SWD mailbox carve below 0x2000_0240, and the `.hotcode` window
 * at the bottom of flash right after the 512 B vector table (the GD32F1x0 is zero-wait only in its
 * first 32 KiB; see memory.x for the measurements and for why each anchor is here). The vector
 * table is 512 B on every chip (`hal` pads it), so `.hotcode` starts at the same 0x0800_0200.
 */
MEMORY
{
  FLASH (rx!w) : ORIGIN = 0x08000000, LENGTH = @FLASH_LENGTH@
  RAM   (rwx)  : ORIGIN = 0x20000240, LENGTH = @RAM_LENGTH@
}

SECTIONS
{
  .hotcode ORIGIN(FLASH) + 0x200 :
  {
    . = ALIGN(4);
    *(.hotcode .hotcode.*)
    . = ALIGN(4);
    /* The image's own SysTick vector (the universal image's was the HAL's `systick_handler`); the
     * period vector's trampolines (`ADC0_1`, `ADC_CMP`) carry `#[link_section = ".hotcode"]`. */
    *(.text.*SysTick*)
    *(.text.*control_task_cb*)
    *(.text.*input_task_cb*)
    *(.text.*fsm_step*)
    *(.text.*route_emits*)
    *(.text.*route_handback*)
    *(.text.*systick_tick_cb*)
    *(.text.*_4link*)
    *(.text.*4base5crc16*)
    *(.text.*5store5store*7compact*)
    *(.text.*5store5store13scan_frontier*)
    *(.text.*5store6record12parse_header*)
    *(.text.*5store6record12is_committed*)
    . = ALIGN(4);
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH
} INSERT BEFORE .text;

_stext = ADDR(.hotcode) + SIZEOF(.hotcode);

ASSERT(ADDR(.hotcode) + SIZEOF(.hotcode) <= 0x08008000,
  "memory-chip.x: .hotcode overflows the GD32F1x0 zero-wait first 32 KiB of flash (see memory.x)");

/* The two budgets CI's per-chip size gate reads (absolute symbols; they occupy nothing). */
__image_flash_budget = LENGTH(FLASH);
__image_ram_budget = LENGTH(RAM);
