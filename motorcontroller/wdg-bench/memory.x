/* Watchdog reload-contract bench image layout. Like firmware / ble-loopback / l2-uart-bench: link
 * into low flash, size to the smallest part (64 KiB flash, 8 KiB RAM), one image valid on both
 * families. The SWD-readable result is the `WDG_BENCH_OBS` static the linker places in RAM (read /
 * poke it by `nm` symbol), so no reserved RAM tail is needed; the startup .data/.bss init clearing
 * the block on every reset is exactly what the proof protocol wants (a poked stall knob does not
 * survive the watchdog reset, so the rebooted loop runs healthy again).
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K    /* smallest part */
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K     /* smallest part */
}
