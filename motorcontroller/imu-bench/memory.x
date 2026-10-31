/* IMU + attitude bench image layout. Like firmware / ble-loopback / l2-uart-bench: link into low
 * flash, size to the smallest part (the F130 slave, which carries the clone IMU: 64 KiB flash,
 * 8 KiB RAM). The SWD-readable result is the `IMU_BENCH_OBS` static the linker places in RAM (read
 * it by `nm` symbol), so no reserved RAM tail is needed.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K    /* smallest part */
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K     /* smallest part */
}
