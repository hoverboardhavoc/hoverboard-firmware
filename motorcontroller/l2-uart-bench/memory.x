/* L2 Tier-2 bench image layout. Like firmware / ble-loopback: link into low flash, size to the
 * smallest part of the pair (the F130 slave: 64 KiB flash, 8 KiB RAM). The SWD-readable result is the
 * `LINK_OBS` static the linker places in RAM (read it by `nm` symbol), so no reserved RAM tail is
 * needed. One linked image, valid on both the F103 master and the F130 slave.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K    /* smallest part */
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K     /* smallest part */
}
