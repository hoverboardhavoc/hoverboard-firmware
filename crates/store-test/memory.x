/* The reserved-RAM-tail layout (see specs/test-harness.md, "Why a reserved RAM tail").
 *
 * Identical to dummy-test's memory.x: RAM is shrunk to end at 0x2000_1F00, leaving the top 256
 * bytes free for the result (RESULT_ADDR = 0x2000_1F00) and command (CMD_ADDR = 0x2000_1FF0)
 * words, OUTSIDE the linked region so they survive reset and the startup .bss clear. RAM is sized
 * to the smallest part (the F130, 8 KiB) so one image is valid on every part.
 *
 * FLASH here is only the region cortex-m-rt LINKS the firmware code + vector table into (low flash).
 * The store region itself is NOT linked: it is the top two DETECTED pages of flash, written by the
 * FMC at absolute addresses the store derives from the Chip (0x0800_F800 on a 64 KiB 1 KiB-page
 * part, 0x0803_F000 on the 256 KiB 2 KiB-page chip2k part). The store-test image is small (well
 * under 64 KiB) so the linked code never reaches the 1 KiB-page store region at the top of 64 KiB.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00   /* 8 KiB - 256 B reserved tail @ 0x2000_1F00 */
}
