# The integrated image's guard symbol lists. ONE copy, sourced by every consumer.
#
# Three things read these lists and all three must agree, which is why they are a file and not a
# literal in each:
#
#   - tools/flash.sh, the flash-time guard, which refuses to program an image that fails them;
#   - .github/workflows/ci.yml, the build-time guard on the real release ELF;
#   - tools/tests/flash-guard-harness.sh, which builds its fabricated images FROM these names, so
#     the harness cannot rot behind a list it does not know about.
#
# They were literals in the first two until 2026-10-24, and they had already drifted: CI carried
# `4base5crc16` and flash.sh did not, and the harness's third copy was six symbols behind both, which
# made every one of its healthy-image cases fail. A list whose whole purpose is to fail on an
# unnoticed change cannot be maintained in three places by hand.
#
# FORMAT: comma-separated `grep -E` patterns over `arm-none-eabi-nm` output. Comma, not space,
# because the whole list crosses an ssh argument boundary as one token.
#
# Only the `integrated` profile is here. The other flash.sh profiles (imu-bench, probe,
# motor-verify, commutation-example) guard bench validators built from other repos, have one
# consumer each and no hot-window half, and stay where their profile is.

# ------------------------------------------------------------------------------------------------
# REQUIRED SYMBOLS: must RESOLVE in the image, anywhere.
#
# The control stack the live firmware must contain, PLUS the six detect fault-path functions. Those
# six are matched by the length-prefixed path `..5probe<len><name>`, which is valid under BOTH legacy
# and v0 mangling (rustc 1.97 flipped the default to v0 and the old `17h` hash anchor broke CI
# closed, 2026-07-22). They are `#[inline(never)]` and host-untestable, so an LTO or codegen change
# that dropped or inlined away the bus-fault-safe probe would strip the fleet's ONLY runtime
# chip-identity path, in an image that still links and still flashes (the class the round-7a gutted
# image showed can pass both).
#
# The motor symbols (motor-integration slice 3) are here on the same reasoning: the 16 kHz period ISR
# is reached only through a registered function pointer and its state through a `static mut` the ISR
# alone touches, so dropping either leaves a configured, counting timer with nothing stepping the
# commutator. An image that links, flashes, and has no motor.
IMAGE_GUARD_REQ_SYMS='T main$,T SysTick$,usart1_rx_isr,dma_rx_isr,B CTRL_OBS$,B INJECT_UART_LINE_ERROR$,5probe3run,5probe12probe_family,5probe15probe_candidate,5probe13probe_present,5probe14measure_counts,5probe15scratch_present,5motor2hw10period_isr,5motor2hw5MOTOR,5motor7PERIODS,5motor9OBS_STATE,5motor7OBS_CAL,3arm2hw4GATE,3arm2hw5ARMED'

# ------------------------------------------------------------------------------------------------
# HOT-WINDOW MEMBERSHIP (audit D7): each must RESOLVE, and the FIRST nm line matching the pattern
# must sit BELOW 0x08008000, the F1x0 zero-wait flash boundary the perf story rests on.
#
# First, not every: nm's default order is by name, so a second symbol matching the same pattern (a
# `run_shell` and a `run_shell_cold` split out above the boundary) is not checked. Tighten the
# pattern if that ever matters; the gate is aimed at a named symbol moving, not at every match.
#
# A symbol that VANISHES (inlined or renamed) fails loudly, so the change is conscious: a silent drop
# out of the window is an 8.8x fetch regression with CI green, which is the fsm_step near-miss.
# fsm_step itself is absent from this list because it fully inlines into run_shell.
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
# outlined and placed in the zero-wait window by a NAME anchor in crates/firmware/memory.x
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
IMAGE_GUARD_HOT_SYMS='5motor2hw10period_isr,12service_loop,control_task_cb,input_task_cb,9run_shell,7adc_isr,15systick_handler,systick_tick_cb,route_emits,route_handback,4base5fixed3div,4base5fixed3mul,4base5fixed4sqrt,4base5crc16,FmcFlashE7compact,5store13scan_frontier,6record12parse_header,6record12is_committed'
