#!/usr/bin/env python3
"""Read and decode a hal bench validator's result block (motorcontroller/hal-bench).

    tools/hal-bench.py <cli> <host:port> <board> <elf>
    tools/hal-bench.py --gdb <host:port> <elf>

Runs `<cli> --tcp <host:port> board read <board> <HB_RESULT> 256` (or, with --gdb, reads the
block through the bridge's GDB stub at <host:port>), where HB_RESULT is the symbol's address in
<elf>, and prints each named word. The layouts are the ones each validator's
source documents; word 0 is the magic (0x48420001 when the run completed), word 1 the validator.
"""
import subprocess
import sys

MAGIC = 0x48420001
NAMES = {
    1: ["idcode", "flash_size_word", "desc_flash_kib", "mismatch_mask", "uid0", "uid1"],
    2: ["scratch_base", "page", "erase_codes", "erased_before", "program", "readback_wrong",
        "reprogram", "cell_after", "out_of_range", "misaligned", "final_erase", "erased_after",
        "verdict"],
    3: ["side", "received", "wrong", "line_errors", "ring_laps", "tx_timeouts", "sent", "verdict", "to_after_writes", "to_after_flush",
        "to_after_token"],
    4: ["rcu_ctl", "rcu_cfg0", "fmc_ws", "clock_verdict", "i2c_new", "who_am_i", "bus_clear",
        "who_am_i_after", "adc_calibrate", "vrefint", "temperature", "scl_level"],
    5: ["ctl0", "ctl1", "chctl0", "chctl1", "chctl2", "psc", "car", "crep", "ch3cv", "cchp",
        "timer_fail_mask", "injected_new", "eoic", "inj0", "inj1", "period_ipr", "systick_pri",
        "priority_verdict", "moe_after", "verdict",
        # Words 22..42: TIMER7 (0 on a chip without it) and the ISQ read-back.
        "t7_ctl0", "t7_ctl1", "t7_chctl0", "t7_chctl1", "t7_chctl2", "t7_psc", "t7_car", "t7_crep",
        "t7_ch3cv", "t7_cchp", "t7_smcfg", "t7_cnt", "t7_fail_mask", "isq", "isq_wrong", "inj2",
        "inj3", "t7_ctl0_after", "t0_cnt_after", "t7_cnt_after", "t7_moe_after"],
}
VALIDATOR = {1: "probe", 2: "fmc", 3: "serial", 4: "coldpath", 5: "motor-verify"}
LABELS = ["Timer0", "Timer7", "Adc0", "Adc1", "Adc2", "I2c1", "Usart2"]


def symbol(elf, name):
    out = subprocess.run(["arm-none-eabi-nm", elf], capture_output=True, text=True, check=True)
    for line in out.stdout.splitlines():
        f = line.split()
        if len(f) == 3 and f[2] == name:
            return int(f[0], 16)
    sys.exit(f"{name} not in {elf}")


def read_gdb(hostport, addr):
    out = subprocess.run(["arm-none-eabi-gdb", "-nx", "-batch", "-ex", f"target remote {hostport}",
                          "-ex", f"x/64wx {addr:#x}"], capture_output=True, text=True, timeout=60)
    w = []
    for line in out.stdout.splitlines():
        if line.startswith("0x") and ":" in line:
            w += [int(x, 16) for x in line.split(":", 1)[1].split()]
    if len(w) < 64:
        sys.exit(f"gdb read failed: {out.stdout}{out.stderr}")
    return w


def main():
    if sys.argv[1] == "--gdb":
        hostport, elf = sys.argv[2:4]
        addr = symbol(elf, "HB_RESULT")
        w = read_gdb(hostport, addr)
    else:
        cli, hostport, board, elf = sys.argv[1:5]
        addr = symbol(elf, "HB_RESULT")
        out = subprocess.run([cli, "--tcp", hostport, "board", "read", board, hex(addr), "256"],
                             capture_output=True, text=True)
        data = bytearray()
        for line in out.stdout.splitlines():
            if ":" in line:
                data += bytes(int(b, 16) for b in line.split(":", 1)[1].split()[:16])
        if len(data) < 256:
            sys.exit(f"read failed: {out.stdout}{out.stderr}")
        w = [int.from_bytes(data[i:i + 4], "little") for i in range(0, 256, 4)]
    print(f"HB_RESULT @ {addr:#010x}: magic {w[0]:#010x} ({'complete' if w[0] == MAGIC else 'NOT COMPLETE'})")
    vid = w[1]
    print(f"validator {VALIDATOR.get(vid, vid)}")
    for i, n in enumerate(NAMES.get(vid, []), start=2):
        print(f"  w{i:<2} {n:<16} {w[i]:#010x} ({w[i]})")
    if vid == 1:
        for i, lab in enumerate(LABELS):
            o = 8 + 4 * i
            flags = w[o + 1]
            print(f"  {lab:<7} sticky {flags & 1} writeback {flags >> 2 & 1} "
                  f"present {flags >> 3 & 1} | descriptor {w[o + 3]} | scratch reset {w[o + 2]:#010x}")
        print(f"  w40 dbg_after      {w[40]:#010x}\n  w41 dbg_before     {w[41]:#010x}")


if __name__ == "__main__":
    main()
