/* The image layout (specs/hal.md, sections 5 and 6; specs/swd-mailbox.md, "Memory layout", the
 * reserved-region carve). A template: build.rs fills the two @..@ lengths from the chip facts the hal
 * crate's build script publishes for the compiled chip (its `links` metadata: flash size, page size,
 * SRAM size), so the numbers have one owner, and writes the result to OUT_DIR/memory.x.
 *
 * FLASH is the chip's flash less the store's two pages at the top. The store is NOT linked: the FMC
 * writes it at absolute addresses the store derives from the same descriptor (`store::geometry`), so
 * an image that would overlap it fails at LINK rather than having its tail overwritten by the first
 * store append (specs/decision-flash-budget.md, "memory.x store reserve"; specs/storage-layer.md).
 *
 * RAM is the chip's SRAM above the SWD mailbox's FIXED reserved region [0x2000_0000, 0x2000_0240) at
 * the bottom (the bridge reads it cold at a version-stable base). It is reserved by starting the
 * linked RAM above it: cortex-m-rt's `.data`/`.bss`/`.uninit`/stack all live at or above
 * 0x2000_0240, and the firmware writes the header through the fixed `swd_mailbox::MAILBOX_BASE`
 * pointer. A `.mailbox`-section carve was rejected: `INSERT AFTER .bss` drags `__ebss` below
 * `__sbss`, and cortex-m-rt 0.7.5's equality-terminated `.bss` zero loop then never terminates and
 * stores off the end of RAM, a bus fault before `main`. 0x240 (576 B) holds the 564 B mailbox (a
 * const-assert in swd-mailbox fails the BUILD if the region outgrows the carve).
 *
 * The image runs from its chip's flash vector table (`hal`, cortex-m-rt's `device` feature) and never
 * writes VTOR, so there is no RAM vector table; the inter-board DMA ring lives in cortex-m-rt's
 * `.uninit` with the other uninitialised statics. The vector table is 512 B on every chip (`hal`
 * pads it), so `.hotcode` starts at 0x0800_0200.
 */
MEMORY
{
  FLASH (rx!w) : ORIGIN = 0x08000000, LENGTH = @FLASH_LENGTH@
  RAM   (rwx)  : ORIGIN = 0x20000240, LENGTH = @RAM_LENGTH@
}

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
 * the cold remainder takes whatever is left. The ASSERT after the section is the guard: if
 * `.hotcode` ever grows past 0x0800_8000 the link FAILS rather than silently returning the F1x0 to
 * a 29x ISR.
 *
 * TWO placement mechanisms, and the first is preferred where it is available:
 *
 *   1. `#[link_section = ".hotcode"]` at the definition, matched by the `*(.hotcode .hotcode.*)`
 *      wildcard below. Rename-proof (the attribute travels with the item) and self-documenting at
 *      the definition. `firmware::service_loop`, the 250 Hz steady-state loop, is placed this way.
 *      It requires `#[inline(never)]` alongside it: `link_section` names where the emitted symbol
 *      lands and does not stop LLVM inlining the body back into a caller that lives elsewhere.
 *   2. Name-anchored `.text.<mangled>` wildcards, for items this crate cannot attribute (the HAL's
 *      and other crates' symbols). Anchors go STALE SILENTLY on a rename: the item quietly drops
 *      out of the window and the F1x0 regresses ~29x with the link still green. So an anchor list
 *      is maintained against the ELF, never against memory, and the required-symbol guard in
 *      ci.yml / tools/flash.sh covers the ISR itself. Verify with:
 *          arm-none-eabi-nm -S -n <elf>   # everything below `_stext` is inside `.hotcode`
 *      Two anchors were pruned on 2026-07-27 for matching nothing: `call_control_handler` (no such
 *      symbol exists anywhere in the image) and `_5reasm` (the reassembler's mangled name is
 *      `..._4link5reasm...`, so the leading underscore never matched; it is in the window via the
 *      `_4link` anchor, which is what actually placed it all along).
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
    /* The SysTick vector; the period vector's trampolines (`ADC0_1`, `ADC_CMP`) carry
     * `#[link_section = ".hotcode"]`. */
    *(.text.*SysTick*)
    *(.text.*control_task_cb*)
    *(.text.*input_task_cb*)
    *(.text.*fsm_step*) /* matches nothing while fsm_step fully inlines into run_shell; kept as the loud standalone-emission fallback: if inlining ever regresses, the symbol lands HOT instead of silently cold */
    *(.text.*route_emits*)
    *(.text.*route_handback*)
    *(.text.*_4link*)
    /* base::crc16's shared fold body, on the framer's path above: it used to be inlined into
     * `link` and placed by the anchor on the line above, and an attribute at its definition would
     * make `.hotcode` an orphan section in every bench/emulator image that links `base` but
     * declares no such section (see the comment there). */
    *(.text.*4base5crc16*)
    /* The store's compaction path and the two record-codec helpers it walks the log with.
     *
     * `compact` is the one cold-path function with a hard real-time deadline: it repacks a full
     * page inside the firmware's 500 ms free watchdog window, and it is all loops (a page scan, a
     * validating walk, a supersede scan per source record). A LOOP above the boundary re-fetches
     * its body from flash every iteration, which is far worse than the 8.8 cycles/instruction the
     * straight-line measurement above prices: the same compaction measured 199 ms linked cold and
     * 65 ms linked here (`specs/bench-evidence/2026-10-23/store-compact-window/`), and the page
     * scan alone went 108 ms -> 4 ms.
     *
     * `parse_header` and `is_committed` are on the 250 Hz control task's arm-time row read as
     * well (see their own comments in `shared/store/src/record.rs`), so they meet the ordinary
     * tick-rate bar for this window on their own account.
     *
     * Anchors and not `#[link_section]`: `shared/store` is linked by the bench and emulator images
     * too (`motorcontroller/store-test`), whose memory.x declares no `.hotcode`, exactly as described for
     * base::crc16 above. tools/image-guard-syms.sh carries all four names in the hot-window
     * membership guard, so a rename that silently unanchors them FAILS the build. */
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
  "memory.x: .hotcode overflows the GD32F1x0 zero-wait first 32 KiB of flash");

/* The two budgets CI's per-chip size gate reads (absolute symbols; they occupy nothing). */
__image_flash_budget = LENGTH(FLASH);
__image_ram_budget = LENGTH(RAM);
