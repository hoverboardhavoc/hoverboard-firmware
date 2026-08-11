/* BLE link stress firmware (Slice 1): GigaDevice SPL, no Rust, no L3, no runtime-hal.
 *
 * Strips L3/Rust/runtime-hal out of the BLE path: bring the onboard CC2541 module up to transparent data
 * mode with the exact `crates/ble::Module::bring_up` AT sequence, then byte-faithfully echo every RX byte
 * (the most direct raw-link test; it naturally reproduces the bridge's coalesce/re-chunk). An SWD-readable
 * `BLE_STRESS_OBS` block records the bring-up outcome (AT answered? which attempt? captured RX bytes) and
 * the echo-phase counters (frames echoed, RX bytes, overruns), modeled on the Rust firmware's
 * `BLE_PROBE_OBS` (crates/firmware/src/main.rs).
 *
 * ONE source, TWO board wirings, selected at compile time by `BOARD` (see the variant block below).
 * A forked copy was rejected: the AT contract, the pacing constants, the observation block and the echo
 * framer are the thing under test and must stay bit-identical across boards, or a bench result on one
 * board says nothing about the other. Only the wiring differs, so only the wiring is conditional.
 *
 * Clock: 72 MHz from IRC8M via PLL (= REFERENCE_72M_IRC8M), so the USART baud divisor matches the real
 * firmware on both families. Busy-spin, NEVER wfi (GD32 SWD-lockout rule); no motor code, nothing arms
 * a bridge.
 */
#include <stdint.h>
#include <string.h>

/* ---- Board wiring variant ------------------------------------------------------------------------
 *
 *   BOARD_BENCH   (default) GD32F103 bench master. CC2541 on USART2 PB10/PB11, F10x GPIO model
 *                           (gpio_init + AFIO), FMC 2 wait states at 72 MHz.
 *   BOARD_OFFROAD           GD32F130 offroad master. CC2541 on USART0 PB6(TX)/PB7(RX) AF0, F1x0 GPIO
 *                           model (gpio_mode_set + per-pin gpio_af_set), FMC 1 wait state at 72 MHz,
 *                           SELF_HOLD (PB12) driven high first.
 *
 * Sources: specs/offroad-pinmap.md section 4.1 (USART0 = the CC2541, PB6/PB7, 9600 8N1, read out of the
 * stock image: `gpio_af_set(GPIOB, 6, AF0)` / `(GPIOB, 7, AF0)`) and GD32F130xx Datasheet Rev3.7
 * Table 2-10 (PB6 AF0 = USART0_TX, PB7 AF0 = USART0_RX).
 *
 * The AF number is load-bearing and there is no safe default. On the SAME PB6/PB7 pair, AF1 is
 * I2C0_SCL/I2C0_SDA, which is what the BENCH F130 uses those pins for. Muxing the wrong AF transmits
 * into a pin wired to a different peripheral and presents exactly as a dead module: bytes go out, the
 * module never answers, and nothing in the observation block distinguishes it from a module that is not
 * there. This cost two bench rounds already (specs/silicon-queue.md: an old-HAL probe baked AF1 and
 * `GPIOB_AFSEL0` read back 0x11000000). Verify the mux over SWD before believing any negative:
 *
 *     GPIOB_AFSEL0 @ 0x48000420 must read 0x00000000   (PB6 = bits[27:24], PB7 = bits[31:28], both AF0)
 *
 * BENCH SAFETY: do NOT flash the offroad variant to a bench F130. PB6/PB7 is that board's hardware I2C0
 * IMU bus (specs/bench-evidence/2026-08-02/usart0/hal-usart0-pb6pb7-slice.md).
 */
#define BOARD_BENCH   0
#define BOARD_OFFROAD 1

#ifndef BOARD
#define BOARD BOARD_BENCH
#endif

#if BOARD == BOARD_OFFROAD

#include "gd32f1x0.h"

#define BLE_USART      USART0
#define BLE_RCU_USART  RCU_USART0
#define BLE_TX_PIN     GPIO_PIN_6
#define BLE_RX_PIN     GPIO_PIN_7
/* Power latch: a rail on this board sits behind PB12 (specs/offroad-pinmap.md, "self-hold / power
 * latch": `gpio_bit_set(GPIOB,0x1000)` in stock). Held high before anything else runs. */
#define SELF_HOLD_PIN  GPIO_PIN_12

#elif BOARD == BOARD_BENCH

#include "gd32f10x.h"

#define BLE_USART      USART2
#define BLE_RCU_USART  RCU_USART2
#define BLE_TX_PIN     GPIO_PIN_10
#define BLE_RX_PIN     GPIO_PIN_11

#else
#error "BOARD must be BOARD_BENCH or BOARD_OFFROAD"
#endif

/* ---- AT contract: byte-identical to crates/ble::at ------------------------------------------------ */
static const char AT_PROBE[]   = "AT\r\n";
static const char AT_OK[]      = "AT+OK\r\n"; /* the EXACT 7-byte reply that advances the probe */
/* The advertised name this image sets, on BOTH boards. It is deliberately NOT the integrated
 * firmware's name: a board running this image is a raw byte-echo, not a link peer, and the two must
 * never be confused on a scan list. Point the phone app's target-name setting at `hb-stress` to talk
 * to it (specs/ble-session.md). */
static const char AT_NAME[]    = "AT+NAME=hb-stress\r\n";
/* 80 (~100 ms), NOT 16 (~20 ms). The CC2541's slow 8051 bridge can't service a fast connection interval
 * (TI: fails ~7.5 ms, works ~100 ms). With 16 the module's own L2CAP request asks interval_max=15
 * (18.75 ms, confirmed on the wire 2026-06-29) and a modern phone honors it, so the module can't keep up
 * and drops the data PDUs (rx_bytes=0). 80 -> the module requests ~100 ms (mapping 16->15 confirmed), the
 * phone honors the slow range, the bridge keeps up. Android-8 stays slow on its own, which is why it works. */
#ifndef CON_INTERVAL_VAL
#define CON_INTERVAL_VAL 80
#endif
#define _STR2(x) #x
#define _STR(x) _STR2(x)
static const char AT_CON_INT[] = "AT+CON_INTERVAL=" _STR(CON_INTERVAL_VAL) "\r\n";
static const char AT_ADV_INT[] = "AT+ADV_INTERVAL=32\r\n";
static const char AT_SET[]     = "AT+SET=1\r\n";       /* SET=1 BEFORE MODE=DATA (order is load-bearing) */
static const char AT_MODE[]    = "AT+MODE=DATA\r\n";

/* Diagnostic (BRINGUP_STYLE==3): send one candidate command after the probe and capture its raw reply in
 * at_rx, so we can see if this OEM module accepts a factory-reset / query. Rebuild per candidate. */
#ifndef DIAG_CMD
#define DIAG_CMD "AT+RENEW\r\n"
#endif
static const char AT_DIAG[] = DIAG_CMD;

#define OK_LEN 7 /* strlen("AT+OK\r\n") */

/* H-A variant switch: does setting CON_INTERVAL/ADV_INTERVAL at bring-up break the BLE data path?
 * 1 = spec-faithful (set both, the committed behavior); 0 = skip them (module-default connection params).
 * Diagnostic only; the committed firmware is BRINGUP_SET_CON_INTERVAL=1. */
#ifndef BRINGUP_SET_CON_INTERVAL
#define BRINGUP_SET_CON_INTERVAL 1
#endif

/* Bring-up style (diagnostic): what does the GD32 say to the module at boot?
 *   0 = NOTHING (RoboDurden-like: no AT at all; module left in its POR state)
 *   1 = probe + MODE=DATA only (no NAME / intervals / SET=1; no config writes)
 *   2 = full bring-up (the committed behavior: probe, NAME, [intervals], SET=1, MODE=DATA)
 * Motivation: a similar module bridged the OnePlus fine under RoboDurden firmware, which sends NO AT
 * commands; our full bring-up (and the NVM config it persists via SET=1) is an untested delta. */
#ifndef BRINGUP_STYLE
#define BRINGUP_STYLE 2
#endif

/* ---- Bring-up pacing: mirrors crates/ble + crates/firmware --------------------------------------- */
#define STEP_MS               248U /* per-command RX-drain window (ble::STEP_MS) */
#define MODE_DRAIN_MS         120U /* short drain after MODE=DATA, then stop reading (ble::MODE_DRAIN_MS) */
#define POLL_US               200U /* RX poll granularity, faster than ~1 ms/byte at 9600 (ble::POLL_US) */
#define POLLS_PER_MS          (1000U / POLL_US)
#define COLD_BOOT_SETTLE_MS   500U /* cold CC2541 is not UART-ready for ~hundreds of ms (firmware const) */
#define PROBE_ATTEMPTS        16U  /* AT-probe attempts after the settle (firmware BLE_PROBE_ATTEMPTS) */

/* ---- SWD diagnostic block ------------------------------------------------------------------------ */
#define OBS_RX_CAP 64U

typedef struct {
    uint32_t magic;              /* MAGIC once written (live marker, not stale RAM) */
    uint32_t at_attempts;        /* AT attempts issued this boot */
    uint32_t at_matched_attempt; /* 1-based attempt AT+OK arrived on (0 = never) */
    uint32_t at_answered;        /* 1 = AT+OK seen, 0 = silent / not-ready / already in data mode */
    uint32_t at_rx_total;        /* total RX bytes seen during the whole bring-up */
    uint32_t at_rx_len;          /* bytes captured into at_rx (<= OBS_RX_CAP) */
    uint32_t frames_echoed;      /* whole 0x5A/len-framed frames echoed (echo phase) */
    uint32_t rx_bytes_total;     /* total RX bytes in the echo phase */
    uint32_t rx_overruns;        /* USART overrun-flag events */
    /* H-B diagnostic (root cause: does the UART RX line carry the phone's bytes after MODE=DATA?). */
    uint32_t echo_stat_accum;    /* OR of (STAT0 & 0x3E) over echo loop: RBNE|IDLEF|ORERR|NERR|FERR ever seen */
    uint32_t echo_loop_iters;    /* echo-loop poll iterations (huge = loop alive; 0 = never reached echo phase) */
    uint8_t  at_rx[OBS_RX_CAP];  /* first OBS_RX_CAP bring-up RX bytes (spot AT+OK\r\n vs garbage) */
} ble_stress_obs_t;

/* "BLES" little-endian: read as 0x53454C42 by `mdw` (low byte 'B'=0x42 first). A fixed un-mangled global
 * symbol the evaluator resolves with `nm <elf> | grep BLE_STRESS_OBS` and reads over SWD. */
#define BLE_STRESS_MAGIC 0x53454C42U

volatile ble_stress_obs_t BLE_STRESS_OBS __attribute__((used));

/* ---- Cycle-counter delays (DWT CYCCNT @ 72 MHz) -------------------------------------------------- */
#define SYSCLK_HZ 72000000U

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SYSCLK_HZ / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) {
        /* busy-wait */
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000U);
    }
}

/* ---- Power latch --------------------------------------------------------------------------------
 * Offroad only, and it must run BEFORE the clock tree: a rail on that board is held up by PB12, so a
 * board running off its own battery drops out from under the firmware if the latch is not driven early.
 * The bench master is bench-powered and its committed image never drove PB12; leaving it alone keeps
 * the bench variant behaviourally identical to the validated one. */
static void self_hold_assert(void)
{
#if BOARD == BOARD_OFFROAD
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SELF_HOLD_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SELF_HOLD_PIN);
    gpio_bit_set(GPIOB, SELF_HOLD_PIN);
#endif
}

/* ---- Clock: 72 MHz IRC8M->PLL (REFERENCE_72M_IRC8M, matches the clock snippet) -------------------
 * IRC8M/2 = 4 MHz, x18 = 72 MHz on both families, so 9600 baud divides exactly (72e6/9600 = 7500).
 * The FMC wait states are the one number that is NOT shared: measured on silicon 2026-07-26, the F10x
 * needs 2 at 72 MHz and the F1x0 needs 1 (runtime-hal src/clock.rs). */
static void clock_72m_irc8m(void)
{
#if BOARD == BOARD_OFFROAD
    fmc_wscnt_set(WS_WSCNT_1);
    RCU_CTL0 |= RCU_CTL0_IRC8MEN;
    while (0U == (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) {
    }
    RCU_CFG0 &= ~RCU_CFG0_AHBPSC;
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    RCU_CFG0 &= ~RCU_CFG0_APB2PSC;
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    RCU_CFG0 &= ~RCU_CFG0_APB1PSC;
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;
    RCU_CFG0 &= ~RCU_CFG0_PLLSEL; /* clear = IRC8M/2 drives the PLL */
    RCU_CFG0 &= ~RCU_CFG0_PLLMF;  /* F1x0's PLLMF mask already carries bit 27 (PLLMF4) */
    RCU_CFG0 |= RCU_PLL_MUL18;
    RCU_CTL0 |= RCU_CTL0_PLLEN;
    while (0U == (RCU_CTL0 & RCU_CTL0_PLLSTB)) {
    }
#else
    fmc_wscnt_set(WS_WSCNT_2);
    RCU_CTL |= RCU_CTL_IRC8MEN;
    while (0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
    }
    RCU_CFG0 &= ~RCU_CFG0_AHBPSC;
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    RCU_CFG0 &= ~RCU_CFG0_APB2PSC;
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    RCU_CFG0 &= ~RCU_CFG0_APB1PSC;
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;
    RCU_CFG0 &= ~RCU_CFG0_PLLSEL;
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= RCU_PLL_MUL18;
    RCU_CTL |= RCU_CTL_PLLEN;
    while (0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }
#endif
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;
    while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS)) {
    }
}

/* ---- BLE USART: 9600 8N1 on this board's CC2541 pins --------------------------------------------
 * BOARD_BENCH:   USART2 PB10(TX)/PB11(RX), F10x GPIO model (mode carries the AF, AFIO clock needed).
 * BOARD_OFFROAD: USART0 PB6(TX)/PB7(RX),  F1x0 GPIO model (mode and AF are separate registers; the
 *                per-pin AF MUST be set to AF0 or the pins stay on I2C0). */
static void ble_usart_init(void)
{
    /* Clocks first, then the pin mux. The call ORDER here is the committed bench image's, unchanged,
     * so the bench variant still assembles byte-for-byte identical to the validated one. */
    rcu_periph_clock_enable(RCU_GPIOB);
#if BOARD == BOARD_BENCH
    /* F10x only: the AFIO block gates the alternate-function pin config. F1x0 has no such block (the
     * AF mux is a plain per-pin GPIO register), so there is nothing to clock. */
    rcu_periph_clock_enable(RCU_AF);
#endif
    rcu_periph_clock_enable(BLE_RCU_USART);

#if BOARD == BOARD_OFFROAD
    /* AF0 on BOTH pins first, then alternate-function mode. RX is driven by the module, so it is
     * pulled up rather than left floating: an idle UART line must read high. */
    gpio_af_set(GPIOB, GPIO_AF_0, BLE_TX_PIN | BLE_RX_PIN);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, BLE_TX_PIN);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BLE_RX_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BLE_TX_PIN | BLE_RX_PIN);
#else
    /* F10x carries the AF in the pin mode itself; there is no per-pin AF register to set. */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BLE_TX_PIN);        /* TX */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BLE_RX_PIN);  /* RX */
#endif

    usart_deinit(BLE_USART);
    usart_baudrate_set(BLE_USART, 9600U);
    usart_word_length_set(BLE_USART, USART_WL_8BIT);
    usart_stop_bit_set(BLE_USART, USART_STB_1BIT);
    usart_parity_config(BLE_USART, USART_PM_NONE);
    usart_receive_config(BLE_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(BLE_USART, USART_TRANSMIT_ENABLE);
    usart_enable(BLE_USART);
}

/* ---- Polled USART primitives -------------------------------------------------------------------- */

/* Finish clearing a pending overrun. The two families do NOT clear ORERR the same way and this is the
 * one owner of that difference:
 *   F10x  a STAT read followed by a DATA read clears it, so by the time a caller gets here it is
 *         already done and there is nothing left to do.
 *   F1x0  the newer INTC-based USART does NOT clear ORERR on a DATA read. It is cleared by writing
 *         OREC into USART_INTC. Skipping it latches polled RX dead on the first overrun: RBNE never
 *         reasserts and the link looks hung with the module fine. (Same trap as runtime-hal's
 *         try_read_byte, which does not clear ORE.)
 * Both callers below read STAT before reading DATA, which is the half the F10x needs. */
static void usart_clear_overrun(void)
{
#if BOARD == BOARD_OFFROAD
    usart_flag_clear(BLE_USART, USART_FLAG_ORERR);
#endif
}

static int usart_rx_ready(void)
{
    /* RBNE (byte available) or ORERR (overrun) pending: either way a read makes progress. */
    return (RESET != usart_flag_get(BLE_USART, USART_FLAG_RBNE)) ||
           (RESET != usart_flag_get(BLE_USART, USART_FLAG_ORERR));
}

/* Read one byte, clearing an overrun the family-correct way (STAT read by usart_flag_get, then DATA read
 * by usart_data_receive, then the F1x0's INTC write). Returns the byte; `*overran` set if ORERR was
 * pending. */
static uint8_t usart_rx_byte(int *overran)
{
    *overran = (RESET != usart_flag_get(BLE_USART, USART_FLAG_ORERR)) ? 1 : 0;
    uint8_t b = (uint8_t) usart_data_receive(BLE_USART);
    if (*overran) {
        usart_clear_overrun();
    }
    return b;
}

static void usart_tx_byte(uint8_t b)
{
    while (RESET == usart_flag_get(BLE_USART, USART_FLAG_TBE)) {
    }
    usart_data_transmit(BLE_USART, b);
}

static void usart_write(const char *s)
{
    while (*s) {
        usart_tx_byte((uint8_t) *s++);
    }
    while (RESET == usart_flag_get(BLE_USART, USART_FLAG_TC)) {
    }
}

/* Tee one bring-up RX byte into the diagnostic block. */
static void obs_push_at_rx(uint8_t b)
{
    BLE_STRESS_OBS.at_rx_total++;
    if (BLE_STRESS_OBS.at_rx_len < OBS_RX_CAP) {
        BLE_STRESS_OBS.at_rx[BLE_STRESS_OBS.at_rx_len++] = b;
    }
}

/* Poll RX promptly through a budget_ms window (fixed poll count, like ble::drain_until_ok), draining and
 * teeing every byte, and report whether the exact 7-byte AT+OK\r\n appeared in the drained stream. */
static int drain_until_ok(uint32_t budget_ms)
{
    uint8_t window[OK_LEN];
    uint32_t filled = 0;
    int saw_ok = 0;
    uint32_t polls = budget_ms * POLLS_PER_MS;

    for (uint32_t i = 0; i < polls; i++) {
        if (usart_rx_ready()) {
            int overran;
            uint8_t b = usart_rx_byte(&overran);
            obs_push_at_rx(b);
            if (filled < OK_LEN) {
                window[filled++] = b;
            } else {
                memmove(window, window + 1, OK_LEN - 1);
                window[OK_LEN - 1] = b;
            }
            if (filled == OK_LEN && memcmp(window, AT_OK, OK_LEN) == 0) {
                saw_ok = 1;
            }
        } else {
            delay_us(POLL_US);
        }
    }
    return saw_ok;
}

/* AT bring-up, mirroring crates/ble::Module::bring_up exactly:
 *   probe (resend AT until exact AT+OK) -> NAME -> CON_INTERVAL -> ADV_INTERVAL -> SET=1 -> MODE=DATA.
 * Returns 1 if the module answered AT (then it has been configured + advertising), 0 if silent. */
static int ble_bring_up(void)
{
#if BRINGUP_STYLE == 0
    return 0; /* say NOTHING (RoboDurden condition): no probe, no config, no MODE=DATA */
#else
    int answered = 0;
    for (uint32_t attempt = 1; attempt <= PROBE_ATTEMPTS; attempt++) {
        BLE_STRESS_OBS.at_attempts = attempt;
        usart_write(AT_PROBE);
        if (drain_until_ok(STEP_MS)) {
            BLE_STRESS_OBS.at_matched_attempt = attempt;
            BLE_STRESS_OBS.at_answered = 1;
            answered = 1;
            break;
        }
    }
    if (!answered) {
        return 0;
    }

#if BRINGUP_STYLE == 3
    /* Diagnostic: reset at_rx capture, send the candidate command, capture its raw reply, then stop.
     * at_rx (read over SWD) shows AT+OK / AT+ERR=n / silence for the candidate. */
    BLE_STRESS_OBS.at_rx_len = 0;
    BLE_STRESS_OBS.at_rx_total = 0;
    usart_write(AT_DIAG);
    drain_until_ok(STEP_MS);
    drain_until_ok(STEP_MS); /* extra window for slow / multi-line replies */
    return 1;
#endif

#if BRINGUP_STYLE == 2
    usart_write(AT_NAME);
    drain_until_ok(STEP_MS);
#if BRINGUP_SET_CON_INTERVAL
    usart_write(AT_CON_INT);
    drain_until_ok(STEP_MS);
    usart_write(AT_ADV_INT);
    drain_until_ok(STEP_MS);
#endif
    usart_write(AT_SET); /* -> advertises; SET=1 double-acks, the full window clears both */
    drain_until_ok(STEP_MS);
#endif /* BRINGUP_STYLE == 2 */
    usart_write(AT_MODE); /* -> transparent; short drain then STOP (further bytes are data) */
    drain_until_ok(MODE_DRAIN_MS);
    return 1;
#endif /* BRINGUP_STYLE */
}

/* Byte-faithful echo: every RX byte echoed unmodified, in order. The SOF(0x5A)/len stream is parsed ONLY
 * to COUNT whole frames for the diagnostic (the echo does not depend on the parse). */
static void echo_loop(void)
{
    /* Minimal wire-frame counter: SOF(0x5A), len, then `len` body bytes (frag-hdr..chunk), then 2 CRC
     * bytes = one whole frame. Counting the CRC bytes keeps a CRC byte that happens to be 0x5A from
     * falsely re-starting the framer. */
    enum { S_SOF, S_LEN, S_BODY, S_CRC } st = S_SOF;
    uint32_t remaining = 0;
    uint32_t crc_left = 0;

    for (;;) {
        /* Read the status register ONCE per poll, BEFORE any DATA read: this is the first half of the
         * family-correct ORERR clear (read STAT, then read DATA, then on F1x0 write INTC). Accumulate
         * the RX-relevant flags (bits 1..5: FERR|NERR|ORERR|IDLEF|RBNE) so the SWD diag shows whether
         * ANY byte/edge reached the UART RX line after MODE=DATA, independent of whether the read logic
         * acted. The two families put this register at different offsets (F10x STAT0 @0x00, F1x0 STAT
         * @0x1C) but assign the SAME bit positions, and both SPLs spell the accessor `USART_STAT`, so
         * the masks below are portable as written. */
        uint32_t stat = USART_STAT(BLE_USART);
        BLE_STRESS_OBS.echo_loop_iters++;
        BLE_STRESS_OBS.echo_stat_accum |= (stat & 0x3EU);

        /* RBNE (bit 5) = byte waiting; ORERR (bit 3) = an overrun is pending. Either way, reading DATA
         * (after the STAT0 read above) makes progress AND clears ORERR the family-correct way, so polled
         * RX never latches dead on an overrun. */
        if (stat & ((1U << 5) | (1U << 3))) {
            if (stat & (1U << 3)) {
                BLE_STRESS_OBS.rx_overruns++;
            }
            uint8_t b = (uint8_t) usart_data_receive(BLE_USART); /* DATA read: clears RBNE (STAT read first) */
            if (stat & (1U << 3)) {
                usart_clear_overrun(); /* F1x0 needs the INTC write on top of the DATA read; F10x is done */
            }
            BLE_STRESS_OBS.rx_bytes_total++;
            usart_tx_byte(b); /* echo unmodified, in order */

            switch (st) {
            case S_SOF:
                if (b == 0x5A) {
                    st = S_LEN;
                }
                break;
            case S_LEN:
                if (b == 0U) {
                    st = S_SOF; /* invalid len; resync */
                } else {
                    remaining = b; /* len = frag-hdr..chunk; 2 CRC bytes follow the body */
                    st = S_BODY;
                }
                break;
            case S_BODY:
                if (--remaining == 0U) {
                    crc_left = 2U;
                    st = S_CRC;
                }
                break;
            case S_CRC:
                if (--crc_left == 0U) {
                    BLE_STRESS_OBS.frames_echoed++; /* whole SOF/len/body/CRC frame seen */
                    st = S_SOF;
                }
                break;
            }
        }
    }
}

int main(void)
{
    self_hold_assert(); /* first: on the offroad board a rail depends on it (no-op on the bench board) */
    clock_72m_irc8m();
    dwt_init();
    ble_usart_init();

    /* Start a fresh boot's diagnostic record. */
    BLE_STRESS_OBS.magic = BLE_STRESS_MAGIC;
    BLE_STRESS_OBS.at_attempts = 0;
    BLE_STRESS_OBS.at_matched_attempt = 0;
    BLE_STRESS_OBS.at_answered = 0;
    BLE_STRESS_OBS.at_rx_total = 0;
    BLE_STRESS_OBS.at_rx_len = 0;
    BLE_STRESS_OBS.frames_echoed = 0;
    BLE_STRESS_OBS.rx_bytes_total = 0;
    BLE_STRESS_OBS.rx_overruns = 0;
    BLE_STRESS_OBS.echo_stat_accum = 0;
    BLE_STRESS_OBS.echo_loop_iters = 0;

    /* Cold CC2541 is not UART-ready for the first few hundred ms; let it settle before the first AT. */
    delay_ms(COLD_BOOT_SETTLE_MS);

    ble_bring_up();

    echo_loop(); /* never returns; busy-spin, NEVER wfi */
    return 0;
}
