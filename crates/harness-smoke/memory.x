/* Reserved-RAM-tail layout for the harness subjects, valid on every fleet part.
 *
 * FLASH 64 KiB and RAM 8 KiB are the SMALLEST fleet sizes (the C8s), so one image links for all
 * parts; the F103 has 20 KiB RAM and the 12-FET more, but using the small sizes never places
 * anything past what every part has (placing anything past 8 KiB faults the F130 at reset).
 *
 * RAM length is 8 KiB MINUS a reserved 256-byte tail: cortex-m-rt lays .data/.bss + stack within
 * the shrunk region, so it never touches the tail. The subject writes its result struct to the
 * fixed RESULT_ADDR = 0x2000_1F00 (start of the tail) and reads the command word from
 * CMD_ADDR = 0x2000_1FF0 (top of the same tail). These addresses are defined in harness-abi (the
 * single source of truth, mirrored by harness_abi::MEMORY_X); the readers use the constants
 * directly, no nm symbol resolution (the size-optimised release ELF drops .symtab).
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x1F00 /* 8 KiB - 256 B harness tail @ 0x2000_1F00 */
}
