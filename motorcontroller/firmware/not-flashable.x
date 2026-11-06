/* The `oversize-link` build (firmware/Cargo.toml, build.rs): FLASH and RAM above are a measurement
 * region larger than the chip's, so an image over its budget links and its size can be read. This
 * image is NEVER flashed. Its code past the budget would overwrite the store's pages, and past the
 * chip's flash it is not there at all. hoverboard-cli refuses any ELF defining this symbol, in
 * `board flash`, `board update` and `firmware pack` (tools/hoverboard-cli/src/elf.rs,
 * `NOT_FLASHABLE`). */
__image_not_flashable = 1;
