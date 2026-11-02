/* The RAM loader for the GD32F1x0, GD32F10x and MM32SPIN05 flash controllers
 * (specs/hoverboard-bridge.md, "Flashing", "The RAM loader"; specs/bridge-chip-support.md,
 * section 5). Modelled on OpenOCD's stm32f1x loader (contrib/loaders/flash/stm32/stm32f1x.S): the
 * bridge streams halfwords into a ring in SRAM over SWD, and this programs them in ascending order
 * while the bridge writes the next chunk.
 *
 * ARMv6-M only (.cpu cortex-m0), which the Cortex-M3 also runs, so one binary serves every row of
 * the part table: no cbz, no it blocks, no immediates on tst or cmn, no ldrsh with an offset. The
 * masks the tests need are held in low registers.
 *
 * Assembled, from bridge/, with:
 *
 *   mkdir -p target/loader
 *   arm-none-eabi-as -mcpu=cortex-m0 -mthumb -o target/loader/f1x.o loader/f1x.S
 *   arm-none-eabi-objcopy -O binary target/loader/f1x.o loader/f1x.bin
 *
 * The layout is owned by core/src/loader.rs (the job, the simulated board and the tests read it
 * there); the two addresses below are the ones this code needs as literals, and a host test checks
 * that f1x.bin carries them. The ring's length and the stack top are the part's, passed in r1 and
 * SP; this code carries no layout constant beyond the two.
 *
 *   0x2000_0000  this code (at most 128 B)
 *   0x2000_0080  wp: the bridge's write pointer into the ring; 0 ends the run
 *   0x2000_0084  rp: the loader's read pointer; 0 after an error
 *   0x2000_0400  the ring: 4 KiB on the 64 KiB parts, 2 KiB on the 4 KiB-SRAM parts
 *   SP           the end of the part's SRAM, capped at 0x2000_2000 (this code does not use it)
 *
 * In:  r0 ring start, r1 ring end, r2 flash address, r3 halfword count.
 * Out: r0 FMC_STAT, r2 the next flash address (on an error, the failing halfword's), then bkpt.
 *
 * Registers while it runs: r8 ring start, r9 ring end, r0 0xFFFF (the erased halfword), r1 the
 * error mask, r2 the flash address, r3 the count, r4 &wp, r5 rp, r6 scratch, r7 the controller.
 *
 * The run ends when every halfword is programmed, or when wp reads 0 (the bridge writes it once
 * rp has caught up with the last chunk). An error (PGERR or WPERR) stops at the failing halfword
 * and sets rp to 0, which the bridge sees the next time it reads rp.
 *
 * A halfword that reads 0xFFFF is left as the erase left it, as the bridge's own programming did.
 */

	.syntax unified
	.cpu cortex-m0
	.thumb

	.equ	WP_ADDR,	0x20000080
	.equ	FMC_BASE,	0x40022000
	.equ	FMC_STAT,	0x0C
	.equ	FMC_CTL,	0x10
	.equ	CTL_PG,		0x01
	.equ	STAT_ERR,	0x14		/* PGERR | WPERR */

	.text
	.global	_start
	.thumb_func
_start:
	mov	r8, r0			/* ring start */
	mov	r9, r1			/* ring end */
	ldr	r4, =WP_ADDR		/* r4 = &wp; rp is at r4 + 4 */
	ldr	r7, =FMC_BASE
	movs	r5, #CTL_PG
	str	r5, [r7, #FMC_CTL]	/* CTL = PG */
	mov	r5, r0			/* rp = ring start */
	ldr	r0, =0xFFFF		/* the erased halfword */
	movs	r1, #STAT_ERR
next:
	cmp	r3, #0
	beq	done			/* every halfword programmed */
wait:
	ldr	r6, [r4]		/* wp */
	cmp	r6, #0
	beq	done			/* wp == 0 ends the run */
	cmp	r6, r5
	beq	wait			/* the ring is empty */
	ldrh	r6, [r5]
	cmp	r6, r0
	beq	advance			/* 0xFFFF: leave the erased cell alone */
	strh	r6, [r2]
busy:
	ldr	r6, [r7, #FMC_STAT]
	lsls	r6, r6, #31		/* BUSY into the flags */
	bne	busy
	ldr	r6, [r7, #FMC_STAT]
	tst	r6, r1
	bne	error
advance:
	adds	r2, #2
	adds	r5, #2
	subs	r3, #1
	cmp	r5, r9
	blo	publish
	mov	r5, r8			/* wrap rp at the ring's end */
publish:
	str	r5, [r4, #4]		/* publish rp */
	b	next
error:
	movs	r5, #0
	str	r5, [r4, #4]		/* rp = 0: the bridge stops feeding */
done:
	ldr	r6, [r7, #FMC_STAT]
	movs	r5, #0
	str	r5, [r7, #FMC_CTL]	/* clear PG */
	mov	r0, r6
	bkpt	#0

	.align	2
	.ltorg
