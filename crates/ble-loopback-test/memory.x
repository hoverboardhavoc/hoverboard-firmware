/* The byte-loopback bench image layout. Like `firmware`, it links into low flash and reserves no RAM
 * tail: it has no host-written result/command channel (the harness oracle is the phone, which scores the
 * echoed BLE stream, not a SWD RAM struct). One linked image, sized to the smallest part (the F130:
 * 64 KiB flash, 8 KiB RAM), like every image here.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K    /* smallest part */
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K     /* smallest part */
}
