/* The universal-binary layout (specs/firmware.md, "Memory layout"; specs/swd-mailbox.md, "Memory
 * layout", the reserved-region carve).
 *
 * The SAME linked image runs on every part, so both regions are sized to the SMALLEST part (the
 * F130: 64 KiB flash, 8 KiB RAM). The bigger parts (F103 20 KiB RAM; 12-FET 256 KiB flash / 48 KiB
 * RAM) just have headroom the image does not use.
 *
 * FLASH here is only the region cortex-m-rt LINKS the code + vector table into (low flash). The
 * store region is NOT linked: it is the top two DETECTED pages of flash, written by the FMC at
 * absolute addresses the store derives from the runtime-detected Chip (0x0800_F800 on a 64 KiB part,
 * 0x0803_F000 on the 256 KiB 12-FET), beyond this length. FmcFlash uses absolute addresses and is
 * not bounded by this length.
 *
 * The SWD mailbox occupies a FIXED reserved region [0x2000_0000, 0x2000_0240) at the bottom of SRAM
 * (the bridge reads it cold at a version-stable base). It is reserved simply by starting the linked
 * RAM ABOVE it (ORIGIN = 0x2000_0240): cortex-m-rt's `.data`/`.bss`/stack all live at/above
 * 0x2000_0240, so nothing the linker places touches the mailbox, and the firmware writes its header
 * through a fixed `swd_mailbox::MAILBOX_BASE` pointer (the `store-test`/`test-shared` reserved-region
 * idiom: no `.mailbox` section, no INSERT). A `.mailbox`-section carve was rejected: `INSERT AFTER .bss`
 * drags `__ebss` down to the mailbox region end (0x2000_0230), BELOW `__sbss` (0x2000_0400), and
 * cortex-m-rt 0.7.5's equality-terminated `.bss` zero loop (`cmp __ebss,__sbss; beq; stm r0!; b`) then
 * never terminates and stores upward off the end of RAM -> a bus fault before `main`. 0x240 (576 B)
 * holds the 560-B mailbox with headroom.
 */
MEMORY
{
  /* 62K, NOT the 64K the part has: the top 2 pages [0x0800_F800, 0x0801_0000) on a 64 KiB part are
   * the store region (specs/decision-flash-budget.md, "memory.x store reserve"; specs/storage-layer.md).
   * The store is NOT linked here (FmcFlash writes it at absolute detected-top-of-flash addresses, beyond
   * this length), so a 63.5 KiB image would link clean and then have its tail OVERWRITTEN by the first
   * runtime store append. Dropping LENGTH to 64K - 2K makes an over-large image fail at LINK, not at the
   * first flash write. Byte-identical .bin vs LENGTH=64K (LENGTH only bounds the region; placement is
   * from ORIGIN upward, unchanged), proven at the slice-C landing. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K - 2K     /* 62K usable; top 2 pages reserved for the store */
  RAM   : ORIGIN = 0x20000240, LENGTH = 8K - 0x240   /* smallest part, less the reserved mailbox region [0x2000_0000, 0x2000_0240) */
}

/* The high-alignment RAM tables, packed (the slice-7 RAM-budget fix; round-11 stack slice). The
 * firmware's RAM vector table (align 512, 512 B) and the HAL's detect-probe vector table would
 * otherwise punch ~1.6 KiB of pure alignment gaps into .data/.bss (measured: 628 + 284 + 776 B of
 * gaps around them at the default placement). Packing them back-to-back at the first 512 boundary
 * after the mailbox carve leaves one unavoidable 448 B gap (0x240 -> 0x400) instead.
 *
 * Round 11 shrank the detect-probe table from 1 KiB (256 words, align 1024) to 128 B (16 words,
 * align 128): the probe runs in an IRQ-less window where only the 16 system vectors are reachable,
 * so the never-reachable external-IRQ slots were pure waste (runtime-hal detect::probe). The tables
 * are ordered largest-alignment-first (RAM_VECTORS align 512 THEN PROBE_VECTOR_TABLE align 128) so
 * the shrink lands as reclaimed stack, not an ALIGN gap: the section is now 512 + 128 = 640 B
 * (0x400 -> 0x680) instead of 1536 B (0x400 -> 0xA00), lowering __ebss / the stack floor by 896 B.
 *
 * NOLOAD and NOT zero-initialized (outside cortex-m-rt's __sbss..__ebss): safe because both
 * tables are fully written before first use (irq::install assigns the whole slots array before
 * the VTOR flip; the detect probe copies the whole active table into its own before flipping),
 * the same discipline .uninit sections rely on. The ASSERT fails the link loudly if either
 * symbol's section name drifts and the pattern stops matching (the gaps would silently return).
 */
/* The F1x0 zero-wait flash window (specs/motor-integration.md, "Hot-path flash placement").
 *
 * MEASURED on silicon 2026-07-26: the GD32F1x0's flash is zero-wait only for the FIRST 32 KiB. The
 * user manual states it outright (GD32F1x0 Rev3.6 section 2.2: "No waiting time within 32K bytes
 * when CPU executes instruction. A long delay when fetch 32K ~ 64K bytes date from flash", and
 * section 1.3: "Read accesses to the preceding 32 pages can be performed 32 bits per cycle without
 * any wait state"). It is an ADDRESS-range property, not a frequency one, and it is NOT what
 * FMC_WS/FMC_WSEN control: sweeping WSCNT changes nothing. The F10x has the same architecture with
 * a 256 KiB boundary, so a 59 KiB image is entirely inside its fast window and pays nothing, which
 * is exactly the "family factor" the diagnostic round measured and could not attribute.
 *
 * Priced in situ on the F130 (bench-evidence/2026-07-26/locality): 256 straight-line nops above the
 * boundary cost 8.8 cycles each instead of 1, and the 16 kHz period ISR cost 17,166 cycles per
 * period linked above it against 596 linked below it: 29x, with the F103 at 600 for the same
 * binary. Below the line the two families measure the SAME, which is the check that the mechanism
 * is the boundary and nothing else.
 *
 * So the hot path is PLACED, not left to link order: everything entered per PWM period (16 kHz) or
 * per control tick (250 Hz) goes in `.hotcode` at the bottom of flash, and `_stext` follows it, so
 * the cold remainder takes whatever is left. The ASSERT at the end of this file is the guard: if
 * `.hotcode` ever grows past 0x0800_8000 the link FAILS rather than silently returning the F1x0 to
 * a 29x ISR. The wildcards are matched against rustc's per-function `.text.<mangled>` sections;
 * they are deliberately name-anchored, so a rename shows up as a measurable regression, not a
 * silent one (the required-symbol guard in ci.yml / tools/flash.sh covers the ISR itself).
 *
 * `.rodata` rides here too, and it is NOT optional: the round measured the whole `.rodata` section
 * out of the window and the ISR went 596 -> 1,822 cycles. Every hot-path table load is a flash DATA
 * read and pays the same boundary.
 */
SECTIONS
{
  .hotcode ORIGIN(FLASH) + 0x200 :
  {
    . = ALIGN(4);
    *(.hotcode .hotcode.*)
    . = ALIGN(4);
    *(.text.*adc_isr*)
    *(.text.*call_control_handler*)
    *(.text.*systick_handler*)
    *(.text.*control_task_cb*)
    *(.text.*input_task_cb*)
    *(.text.*fsm_step*)
    *(.text.*route_emits*)
    *(.text.*route_handback*)
    *(.text.*systick_tick_cb*)
    *(.text.*_4link*)
    *(.text.*_5reasm*)
    . = ALIGN(4);
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH
} INSERT BEFORE .text;

_stext = ADDR(.hotcode) + SIZEOF(.hotcode);

SECTIONS
{
  .ramtables (NOLOAD) :
  {
    . = ALIGN(512);
    *(.bss.*RAM_VECTORS*)
    . = ALIGN(128);
    *(.bss.*PROBE_VECTOR_TABLE*)
  } > RAM
} INSERT BEFORE .data;

ASSERT(SIZEOF(.ramtables) >= 640, "memory.x: .ramtables lost its tables (symbol/section rename?)");

/* The whole point of the section above: hot code that spills past 32 KiB silently costs the F1x0
 * ~29x on the 16 kHz ISR. Fail the LINK instead. */
ASSERT(ADDR(.hotcode) + SIZEOF(.hotcode) <= 0x08008000,
  "memory.x: .hotcode overflows the GD32F1x0 zero-wait first 32 KiB of flash (see the header above)");
