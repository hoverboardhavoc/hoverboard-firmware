/* The RAM loader for the GD32F1x0 and GD32F10x flash controller (specs/hoverboard-bridge.md,
 * "Flashing", "The RAM loader"). Modelled on OpenOCD's stm32f1x loader
 * (contrib/loaders/flash/stm32/stm32f1x.S): the bridge streams halfwords into a ring in SRAM over
 * SWD, and this programs them in ascending order while the bridge writes the next chunk.
 *
 * Assembled, from bridge/, with:
 *
 *   mkdir -p target/loader
 *   arm-none-eabi-as -mcpu=cortex-m3 -mthumb -o target/loader/f1x.o loader/f1x.S
 *   arm-none-eabi-objcopy -O binary target/loader/f1x.o loader/f1x.bin
 *
 * The layout is owned by core/src/loader.rs (the job, the simulated board and the tests read it
 * there); the two addresses below are the ones this code needs as literals, and a host test checks
 * that f1x.bin carries them.
 *
 *   0x2000_0000  this code (under 128 B)
 *   0x2000_0080  wp: the bridge's write pointer into the ring; 0 ends the run
 *   0x2000_0084  rp: the loader's read pointer; 0 after an error
 *   0x2000_0400  the 4 KiB ring, to 0x2000_1400
 *   0x2000_2000  the stack top (the smallest part's 8 KiB; this code does not use it)
 *
 * In:  r0 ring start, r1 ring end, r2 flash address, r3 halfword count.
 * Out: r0 FMC_STAT, r2 the next flash address (on an error, the failing halfword's), then bkpt.
 *
 * The run ends when every halfword is programmed, or when wp reads 0 (the bridge writes it once
 * rp has caught up with the last chunk). An error (PGERR or WPERR) stops at the failing halfword
 * and sets rp to 0, which the bridge sees the next time it reads rp.
 *
 * A halfword that reads 0xFFFF is left as the erase left it, as the bridge's own programming did.
 */

	.syntax unified
	.cpu cortex-m3
	.thumb

	.equ	WP_ADDR,	0x20000080
	.equ	FMC_BASE,	0x40022000
	.equ	FMC_STAT,	0x0C
	.equ	FMC_CTL,	0x10
	.equ	CTL_PG,		0x01
	.equ	STAT_BUSY,	0x01
	.equ	STAT_ERR,	0x14		/* PGERR | WPERR */

	.text
	.global	_start
	.thumb_func
_start:
	ldr	r4, =WP_ADDR		/* r4 = &wp; rp is at r4 + 4 */
	ldr	r7, =FMC_BASE
	movs	r5, #CTL_PG
	str	r5, [r7, #FMC_CTL]	/* CTL = PG */
	mov	r5, r0			/* rp = ring start */
next:
	cbz	r3, done		/* every halfword programmed */
wait:
	ldr	r6, [r4]		/* wp */
	cbz	r6, done		/* wp == 0 ends the run */
	cmp	r6, r5
	beq	wait			/* the ring is empty */
	ldrsh	r6, [r5]
	cmn	r6, #1
	beq	advance			/* 0xFFFF: leave the erased cell alone */
	strh	r6, [r2]
busy:
	ldr	r6, [r7, #FMC_STAT]
	tst	r6, #STAT_BUSY
	bne	busy
	tst	r6, #STAT_ERR
	bne	error
advance:
	adds	r2, #2
	adds	r5, #2
	subs	r3, #1
	cmp	r5, r1
	it	hs
	movhs	r5, r0			/* wrap rp at the ring's end */
	str	r5, [r4, #4]		/* publish rp */
	b	next
error:
	movs	r5, #0
	str	r5, [r4, #4]		/* rp = 0: the bridge stops feeding */
	b	exit
done:
	ldr	r6, [r7, #FMC_STAT]
exit:
	movs	r5, #0
	str	r5, [r7, #FMC_CTL]	/* clear PG */
	mov	r0, r6
	bkpt	#0

	.align	2
	.ltorg
