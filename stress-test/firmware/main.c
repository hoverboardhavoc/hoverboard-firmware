/* BLE link stress firmware (Slice 1): GD32F103, GigaDevice SPL, no Rust, no L3, no runtime-hal.
 *
 * Strips L3/Rust/runtime-hal out of the BLE path: bring the onboard CC2541 module up to transparent data
 * mode with the exact `crates/ble::Module::bring_up` AT sequence, then byte-faithfully echo every RX byte
 * (the most direct raw-link test; it naturally reproduces the bridge's coalesce/re-chunk). An SWD-readable
 * `BLE_STRESS_OBS` block records the bring-up outcome (AT answered? which attempt? captured RX bytes) and
 * the echo-phase counters (frames echoed, RX bytes, overruns), modeled on the Rust firmware's
 * `BLE_PROBE_OBS` (crates/firmware/src/main.rs).
 *
 * Clock: 72 MHz from IRC8M via PLL (= REFERENCE_72M_IRC8M), so the USART baud divisor matches the real
 * firmware. USART: USART2 PB10(TX)/PB11(RX), 9600 8N1 (the hoverboard's BLE pins/baud). Busy-spin, NEVER
 * wfi (GD32 SWD-lockout rule); no motor code, nothing arms a bridge.
 */
#include <stdint.h>
#include <string.h>
#include "gd32f10x.h"

/* ---- AT contract: byte-identical to crates/ble::at ------------------------------------------------ */
static const char AT_PROBE[]   = "AT\r\n";
static const char AT_OK[]      = "AT+OK\r\n"; /* the EXACT 7-byte reply that advances the probe */
static const char AT_NAME[]    = "AT+NAME=hb-stress\r\n";
static const char AT_CON_INT[] = "AT+CON_INTERVAL=16\r\n";
static const char AT_ADV_INT[] = "AT+ADV_INTERVAL=32\r\n";
static const char AT_SET[]     = "AT+SET=1\r\n";       /* SET=1 BEFORE MODE=DATA (order is load-bearing) */
static const char AT_MODE[]    = "AT+MODE=DATA\r\n";

#define OK_LEN 7 /* strlen("AT+OK\r\n") */

/* H-A variant switch: does setting CON_INTERVAL/ADV_INTERVAL at bring-up break the BLE data path?
 * 1 = spec-faithful (set both, the committed behavior); 0 = skip them (module-default connection params).
 * Diagnostic only; the committed firmware is BRINGUP_SET_CON_INTERVAL=1. */
#ifndef BRINGUP_SET_CON_INTERVAL
#define BRINGUP_SET_CON_INTERVAL 1
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

/* ---- Clock: 72 MHz IRC8M->PLL (REFERENCE_72M_IRC8M, matches the clock snippet) ------------------- */
static void clock_72m_irc8m(void)
{
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
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;
    while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS)) {
    }
}

/* ---- USART2 PB10/PB11 9600 8N1 ------------------------------------------------------------------- */
static void usart2_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_USART2);

    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);   /* TX */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11); /* RX */

    usart_deinit(USART2);
    usart_baudrate_set(USART2, 9600U);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
}

/* ---- Polled USART primitives -------------------------------------------------------------------- */
static int usart_rx_ready(void)
{
    /* RBNE (byte available) or ORERR (overrun) pending: either way a read makes progress. */
    return (RESET != usart_flag_get(USART2, USART_FLAG_RBNE)) ||
           (RESET != usart_flag_get(USART2, USART_FLAG_ORERR));
}

/* Read one byte, clearing an overrun the family-correct way (STAT read by usart_flag_get, then DATA read
 * by usart_data_receive). Returns the byte; `*overran` set if ORERR was pending. */
static uint8_t usart_rx_byte(int *overran)
{
    *overran = (RESET != usart_flag_get(USART2, USART_FLAG_ORERR)) ? 1 : 0;
    return (uint8_t) usart_data_receive(USART2);
}

static void usart_tx_byte(uint8_t b)
{
    while (RESET == usart_flag_get(USART2, USART_FLAG_TBE)) {
    }
    usart_data_transmit(USART2, b);
}

static void usart_write(const char *s)
{
    while (*s) {
        usart_tx_byte((uint8_t) *s++);
    }
    while (RESET == usart_flag_get(USART2, USART_FLAG_TC)) {
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
    usart_write(AT_MODE); /* -> transparent; short drain then STOP (further bytes are data) */
    drain_until_ok(MODE_DRAIN_MS);
    return 1;
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
        /* Read STAT0 ONCE per poll, BEFORE any DATA read: this is the first half of the family-correct
         * ORERR clear (read STAT0, then read DATA). Accumulate the RX-relevant flags (bits 1..5:
         * FERR|NERR|ORERR|IDLEF|RBNE) so the SWD diag shows whether ANY byte/edge reached the UART RX
         * line after MODE=DATA, independent of whether the read logic acted. */
        uint32_t stat = USART_STAT(USART2);
        BLE_STRESS_OBS.echo_loop_iters++;
        BLE_STRESS_OBS.echo_stat_accum |= (stat & 0x3EU);

        /* RBNE (bit 5) = byte waiting; ORERR (bit 3) = an overrun is pending. Either way, reading DATA
         * (after the STAT0 read above) makes progress AND clears ORERR the family-correct way, so polled
         * RX never latches dead on an overrun. */
        if (stat & ((1U << 5) | (1U << 3))) {
            if (stat & (1U << 3)) {
                BLE_STRESS_OBS.rx_overruns++;
            }
            uint8_t b = (uint8_t) usart_data_receive(USART2); /* DATA read: clears RBNE/ORERR (STAT0 read first) */
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
    clock_72m_irc8m();
    dwt_init();
    usart2_init();

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
