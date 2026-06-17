/* Linked for the SMALLEST part so one binary runs on every target (see spec/core.md "Memory model").
   Smallest: GD32F130C8, 64 KiB flash @ 0x08000000, 8 KiB RAM @ 0x20000000. The F103C8 (20 KiB) and
   RCT6 (48 KiB) simply leave their extra RAM unused; linking for 20K faults the F130 at reset (its SP
   would point past 8K). */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}
