/* L2 Tier-3 BLE bench image layout. Like ble-loopback / l2-uart-bench: link into low flash, size to the
 * smallest part (64 KiB flash, 8 KiB RAM). The SWD-readable result is the `LINK_OBS` static the linker
 * places in RAM (read by `nm` symbol); no reserved RAM tail. The bench master is the F103.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K
}
