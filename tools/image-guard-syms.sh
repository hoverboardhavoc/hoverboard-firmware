# Each image's release guards, keyed by image name. ONE copy, read by every consumer.
#
# Four things read these guards and all four must agree, which is why they are a file and not a
# literal in each:
#
#   - tools/flash.sh, the flash-time guard, which refuses to program an image that fails them;
#   - .github/workflows/ci.yml, the build-time guard on the real release ELF;
#   - `hoverboard-cli firmware pack` (tools/hoverboard-cli/src/elf.rs, `guards_for`), which copies
#     an image's guards into the bundle manifest, so `board update` checks the image against them
#     (specs/firmware-bundle.md section 3.3);
#   - tools/tests/flash-guard-harness.sh, which builds its fabricated images FROM these names, so
#     the harness cannot rot behind a list it does not know about.
#
# They were literals in the first two until 2026-10-24, and they had already drifted: CI carried
# `4base5crc16` and flash.sh did not, and the harness's third copy was six symbols behind both, which
# made every one of its healthy-image cases fail. A list whose whole purpose is to fail on an
# unnoticed change cannot be maintained in three places by hand.
#
# FORMAT: `image_guards <image-name>` sets three variables for that image and returns 1 for a name
# it does not know. The image names are the ones the image crate's Cargo.toml gives, one
# `[package.metadata.hoverboard-image.<name>]` table per image (motorcontroller/firmware/Cargo.toml).
#
#   IMAGE_GUARD_FLOOR     the release code floor: `.text` + `.hotcode` must be at least this many B.
#   IMAGE_GUARD_REQ_SYMS  comma-separated `grep -E` patterns over `arm-none-eabi-nm` output that must
#                         each match.
#   IMAGE_GUARD_HOT_SYMS  comma-separated patterns whose first nm match must sit below 0x08008000;
#                         empty for an image with no hot-window gate.
#
# Comma, not space, because a whole list crosses an ssh argument boundary as one token. hoverboard-cli
# reads this file as text: an image's arm is its `<name>)` line up to its `;;` line, and inside it
# each `IMAGE_GUARD_<KEY>=<value>` line, the value single-quoted or a bare number. Keep that shape.
#
# flash.sh's imu-bench profile is not an image: it guards a bench validator, has one consumer and no
# hot-window half, and stays where its profile is.

image_guards() {
  IMAGE_GUARD_FLOOR=''
  IMAGE_GUARD_REQ_SYMS=''
  IMAGE_GUARD_HOT_SYMS=''
  case "$1" in
    mainboard-gd32f103c8)
      # The mainboard image for the GD32F103C8 (specs/hal.md section 6). 54,132 B of code (.text +
      # .hotcode) when healthy, measured at the retirement of the universal image.
      IMAGE_GUARD_FLOOR=40000

      # ----------------------------------------------------------------------------------------------
      # REQUIRED SYMBOLS: must RESOLVE in the image, anywhere.
      #
      # The control stack the live firmware must contain, and the two link vectors: the USART1 handler and
      # the DMA channel vector of its receive ring (the hal descriptor's `dma_rx`, DMA0 channel 5 here).
      # Both are reached only through the flash vector table, so an image that dropped either still links.
      # Paths are matched by their length-prefixed segments (`..5motor2hw..`), which are valid under both
      # legacy and v0 mangling (rustc 1.97 flipped the default to v0 and the old `17h` hash anchor broke
      # CI closed, 2026-07-22).
      #
      # The motor symbols (motor-integration slice 3) are here on the same reasoning: the 16 kHz period ISR
      # is reached only through a registered function pointer and its state through a `static mut` the ISR
      # alone touches, so dropping either leaves a configured, counting timer with nothing stepping the
      # commutator. An image that links, flashes, and has no motor.
      IMAGE_GUARD_REQ_SYMS='T main$,T SysTick$,T USART1$,T DMA0_Channel5$,B CTRL_OBS$,B INJECT_UART_LINE_ERROR$,5motor2hw10period_isr,5motor2hw5MOTOR,5motor7PERIODS,5motor9OBS_STATE,5motor7OBS_CAL,3arm2hw4GATE,3arm2hw5ARMED'

      # ----------------------------------------------------------------------------------------------
      # HOT-WINDOW MEMBERSHIP (audit D7): each must RESOLVE, and the FIRST nm line matching the pattern
      # must sit BELOW 0x08008000, the F1x0 zero-wait flash boundary the perf story rests on.
      #
      # First, not every: nm's default order is by name, so a second symbol matching the same pattern (a
      # `run_shell` and a `run_shell_cold` split out above the boundary) is not checked. Tighten the
      # pattern if that ever matters; the gate is aimed at a named symbol moving, not at every match.
      #
      # A symbol that VANISHES (inlined or renamed) fails loudly, so the change is conscious: a silent drop
      # out of the window is an 8.8x fetch regression with CI green, which is the fsm_step near-miss.
      # fsm_step itself is absent from this list because it fully inlines into run_shell. The ADC and
      # SysTick handlers are named by their vector symbols: their bodies inline into them.
      #
      # `4base5fixed3div` is here for BOTH halves, and the missing half matters more:
      #   - above the boundary: the image's only `Fix` division body, entered 9 times per 250 Hz control
      #     tick, so left to link order it lands ~0x0800a4xx and all 9 pay 2 wait states with no prefetch.
      #     Placed by `#[link_section]` at the definition, which is rename-proof, but a memory.x edit can
      #     still evict it.
      #   - absent: `base::fixed::div` exists as a symbol only because of its `#[inline(never)]`. Drop that
      #     and LLVM re-inlines a ~730 B 128-bit division at all four call sites, the image grows ~8.6 KB,
      #     and nothing else fails.
      #
      # `4base5fixed3mul` is the same two reasons one step later on the same tick: the only `Fix`
      # multiplication body, entered 41 times per 250 Hz tick from `attitude::Mahony::update_dt`, and its
      # `#[inline(never)]` IS the -872 B shrink-round-2 lever (specs/decision-flash-budget.md). Drop the
      # attribute and LLVM re-inlines a ~34 B 128-bit widening multiply at all 41 sites.
      #
      # `4base5fixed4sqrt` is the third of the family: the only `Fix` square-root body, entered twice per
      # 250 Hz tick from the Mahony filter's two normalizations, and its `#[inline(never)]` IS the -200 B
      # shrink-round-4 lever. Drop the attribute and LLVM re-inlines a ~200 B Newton iteration at both.
      #
      # `4base5crc16` is here for the MECHANISM rather than a measurement. The L2 framer's CRC fold is
      # outlined and placed in the zero-wait window by a NAME anchor in motorcontroller/firmware/memory.x
      # (`*(.text.*4base5crc16*)`), and a name anchor goes stale silently on a rename. Without an entry
      # here the fold drifts above the boundary and refetches per byte at 2 wait states, with nothing
      # failing. This is the entry CI had and flash.sh did not.
      #
      # The four store entries are the same mechanism and they are why the list earns its upkeep.
      # `Store::compact` is the one cold-path function with a hard real-time deadline: it repacks a full
      # store page inside the 500 ms free watchdog, and it is all loops, which is the worst thing to put
      # above the boundary because a loop re-fetches its body every iteration. The same compaction measured
      # 199 ms linked cold against 60 ms linked in the window, its 1 KiB page scan 108 ms against 0.4 ms
      # (specs/bench-evidence/2026-10-23/store-compact-window/). All four are placed by NAME anchors in
      # memory.x; `parse_header` and `is_committed` are on the 250 Hz arm-time row read as well.
      # `Store::digest` (FmcFlashE6digest) and the `Type::fixed_len` it calls per row are the walk a persisted config write runs three times: linked
      # cold, its loop crossed a 512 B flash boundary on feat/tones and the three took about 1 s against
      # the 500 ms watchdog (specs/bench-work/tones/digest-slow-report.md).
      IMAGE_GUARD_HOT_SYMS='5motor2hw10period_isr,12service_loop,control_task_cb,input_task_cb,9run_shell,T ADC0_1$,T SysTick$,route_emits,route_handback,4base5fixed3div,4base5fixed3mul,4base5fixed4sqrt,4base5crc16,FmcFlashE7compact,5store13scan_frontier,6record12parse_header,6record12is_committed,FmcFlashE6digest,4Type9fixed_len'
      ;;
    mainboard-gd32f130c8)
      # The mainboard image for the GD32F130C8: mainboard-gd32f103c8's lists, every reason there, with
      # this chip's vector names (the USART1 receive ring on DMA channels 3-4, the ADC on ADC_CMP).
      # 54,232 B of code when healthy.
      IMAGE_GUARD_FLOOR=40000
      IMAGE_GUARD_REQ_SYMS='T main$,T SysTick$,T USART1$,T DMA_Channel3_4$,B CTRL_OBS$,B INJECT_UART_LINE_ERROR$,5motor2hw10period_isr,5motor2hw5MOTOR,5motor7PERIODS,5motor9OBS_STATE,5motor7OBS_CAL,3arm2hw4GATE,3arm2hw5ARMED'
      IMAGE_GUARD_HOT_SYMS='5motor2hw10period_isr,12service_loop,control_task_cb,input_task_cb,9run_shell,T ADC_CMP$,T SysTick$,route_emits,route_handback,4base5fixed3div,4base5fixed3mul,4base5fixed4sqrt,4base5crc16,FmcFlashE7compact,5store13scan_frontier,6record12parse_header,6record12is_committed,FmcFlashE6digest,4Type9fixed_len'
      ;;
    mainboard-gd32f103rc)
      # The 12-FET's two-motor image for the GD32F103RC (specs/twelve-fet.md section 3.3):
      # mainboard-gd32f103c8's lists, every reason there, on the same ADC0_1 vector and USART1/DMA0
      # channel 5 receive ring. 56,468 B of code at its first build (2026-11-02), against the
      # F103C8's 50,516 B at the same commit; the floor is the F103C8's ratio of it.
      IMAGE_GUARD_FLOOR=44000
      IMAGE_GUARD_REQ_SYMS='T main$,T SysTick$,T USART1$,T DMA0_Channel5$,B CTRL_OBS$,B INJECT_UART_LINE_ERROR$,5motor2hw10period_isr,5motor2hw5MOTOR,5motor7PERIODS,5motor9OBS_STATE,5motor7OBS_CAL,3arm2hw4GATE,3arm2hw5ARMED'
      IMAGE_GUARD_HOT_SYMS='5motor2hw10period_isr,12service_loop,control_task_cb,input_task_cb,9run_shell,T ADC0_1$,T SysTick$,route_emits,route_handback,4base5fixed3div,4base5fixed3mul,4base5fixed4sqrt,4base5crc16,FmcFlashE7compact,5store13scan_frontier,6record12parse_header,6record12is_committed'
      ;;
    *) return 1 ;;
  esac
}
