/* The reserved-RAM-tail layout (see specs/test-harness.md, "Why a reserved RAM tail").
 *
 * RAM is shrunk to end at 0x2000_1F00, leaving the top 256 bytes free for the result
 * (RESULT_ADDR = 0x2000_1F00) and command (CMD_ADDR = 0x2000_1FF0) words. Pinning a
 * fixed-address struct at the RAM origin, ahead of .data/.bss, collides with cortex-m-rt's
 * startup: it miscomputes __ebss and the .bss clear runs off the end of RAM into a bus fault.
 * Keeping the tail OUTSIDE the linked region also means the input the host writes to CMD_ADDR
 * survives reset and the startup .bss clear, so the device reads it after reset.
 *
 * One image must be valid on every part, so RAM is sized to the smallest (the F130, 8 KiB) and
 * nothing is placed past 8 KiB.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00   /* 8 KiB - 256 B reserved tail @ 0x2000_1F00 */
}
