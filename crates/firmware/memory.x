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
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K          /* smallest part; store writes the detected top-of-flash at runtime, beyond this length */
  RAM   : ORIGIN = 0x20000240, LENGTH = 8K - 0x240   /* smallest part, less the reserved mailbox region [0x2000_0000, 0x2000_0240) */
}

/* The high-alignment RAM tables, packed (the slice-7 RAM-budget fix). The HAL's 1 KiB
 * detect-probe vector table (align 1024) and the firmware's RAM vector table (align 512) would
 * otherwise punch ~1.6 KiB of pure alignment gaps into .data/.bss (measured: 628 + 284 + 776 B
 * of gaps around them at the default placement). Packing them back-to-back at the first 1024
 * boundary after the mailbox carve leaves one unavoidable 448 B gap (0x240 -> 0x400) instead.
 *
 * NOLOAD and NOT zero-initialized (outside cortex-m-rt's __sbss..__ebss): safe because both
 * tables are fully written before first use (irq::install assigns the whole slots array before
 * the VTOR flip; the detect probe copies the entire active table into its own before flipping),
 * the same discipline .uninit sections rely on. The ASSERT fails the link loudly if either
 * symbol's section name drifts and the pattern stops matching (the gaps would silently return).
 */
SECTIONS
{
  .ramtables (NOLOAD) :
  {
    . = ALIGN(1024);
    *(.bss.*PROBE_VECTOR_TABLE*)
    . = ALIGN(512);
    *(.bss.*RAM_VECTORS*)
  } > RAM
} INSERT BEFORE .data;

ASSERT(SIZEOF(.ramtables) >= 1536, "memory.x: .ramtables lost its tables (symbol/section rename?)");
