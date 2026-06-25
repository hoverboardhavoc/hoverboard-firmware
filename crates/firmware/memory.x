/* The universal-binary layout (specs/firmware.md, "Memory layout").
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
 * Unlike the test-image crates (dummy-test / store-test), firmware reserves no RAM tail: it is the
 * shipping binary, not a harness fixture, and has no host-written result/command channel.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K    /* smallest part; store writes the detected top-of-flash at runtime, beyond this length */
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K     /* smallest part */
}
