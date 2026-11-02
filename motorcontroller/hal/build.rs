// SPDX-License-Identifier: GPL-3.0-or-later
//! The chip table: which register model each peripheral of the compiled chip uses, and the chip's
//! vendor vector list (specs/hal.md, sections 3.1 and 5).
//!
//! The chip is the one chip cargo feature that is enabled (src/lib.rs refuses none or two). From
//! it this script:
//!
//! - emits one `hal_<peripheral> = "<model>"` cfg per peripheral whose model differs between the
//!   chips, so a model module is compiled in with `#[cfg(hal_gpio = "crl")]` and no source file
//!   lists chips;
//! - writes `interrupts.rs` (the `Interrupt` enum and the flash `__INTERRUPTS` table) and
//!   `device.x` (a weak `DefaultHandler` alias per named vector) into `OUT_DIR`, both from the one
//!   list below, which is the vendor startup file's vector list for the part;
//! - owns the chip's memory sizes (flash, page, SRAM): it writes them into `memory.rs`, which the
//!   descriptor includes, and publishes them as this package's `links` metadata (`flash_kib`,
//!   `page_bytes`, `ram_kib`), which an image's build script reads as `DEP_HAL_*` to size its
//!   linker regions. One owner for the numbers the descriptor, the store placement and the link
//!   all use.

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;

/// One chip: its feature name, its memory sizes, its model per peripheral, and its vendor vector
/// list.
struct ChipRow {
    feature: &'static str,
    /// Flash size, KiB.
    flash_kib: u32,
    /// Flash page size, bytes.
    page_bytes: u32,
    /// SRAM size, KiB.
    ram_kib: u32,
    /// `(cfg name, model)` for every peripheral with more than one model.
    models: &'static [(&'static str, &'static str)],
    /// The device vectors in IRQ-number order, `None` where the vendor table holds a reserved 0.
    vectors: &'static [Option<&'static str>],
    /// Device-interrupt entries in the flash table (the list padded with reserved words).
    table_entries: usize,
}

/// GD32F10x medium density, from GigaDevice's `startup_gd32f10x_md.s` (GD32F10x CMSIS, the pinned
/// GD32Firmware commit): IRQ 0 (`WWDGT`) through 42 (`USBD_WKUP`), five reserved words and `EXMC`
/// at 48, exactly as the vendor table lays them out.
const F10X_MD: &[Option<&str>] = &[
    Some("WWDGT"),
    Some("LVD"),
    Some("TAMPER"),
    Some("RTC"),
    Some("FMC"),
    Some("RCU"),
    Some("EXTI0"),
    Some("EXTI1"),
    Some("EXTI2"),
    Some("EXTI3"),
    Some("EXTI4"),
    Some("DMA0_Channel0"),
    Some("DMA0_Channel1"),
    Some("DMA0_Channel2"),
    Some("DMA0_Channel3"),
    Some("DMA0_Channel4"),
    Some("DMA0_Channel5"),
    Some("DMA0_Channel6"),
    Some("ADC0_1"),
    Some("USBD_HP_CAN0_TX"),
    Some("USBD_LP_CAN0_RX0"),
    Some("CAN0_RX1"),
    Some("CAN0_EWMC"),
    Some("EXTI5_9"),
    Some("TIMER0_BRK"),
    Some("TIMER0_UP"),
    Some("TIMER0_TRG_CMT"),
    Some("TIMER0_Channel"),
    Some("TIMER1"),
    Some("TIMER2"),
    Some("TIMER3"),
    Some("I2C0_EV"),
    Some("I2C0_ER"),
    Some("I2C1_EV"),
    Some("I2C1_ER"),
    Some("SPI0"),
    Some("SPI1"),
    Some("USART0"),
    Some("USART1"),
    Some("USART2"),
    Some("EXTI10_15"),
    Some("RTC_Alarm"),
    Some("USBD_WKUP"),
    None,
    None,
    None,
    None,
    None,
    Some("EXMC"),
];

/// GD32F10x high density, from `startup_gd32f10x_hd.s`: the medium-density list through 42, then
/// the TIMER7, ADC2, EXMC, SDIO, TIMER4..6, SPI2, UART3/4 and DMA1 vectors through 59.
const F10X_HD: &[Option<&str>] = &[
    Some("WWDGT"),
    Some("LVD"),
    Some("TAMPER"),
    Some("RTC"),
    Some("FMC"),
    Some("RCU"),
    Some("EXTI0"),
    Some("EXTI1"),
    Some("EXTI2"),
    Some("EXTI3"),
    Some("EXTI4"),
    Some("DMA0_Channel0"),
    Some("DMA0_Channel1"),
    Some("DMA0_Channel2"),
    Some("DMA0_Channel3"),
    Some("DMA0_Channel4"),
    Some("DMA0_Channel5"),
    Some("DMA0_Channel6"),
    Some("ADC0_1"),
    Some("USBD_HP_CAN0_TX"),
    Some("USBD_LP_CAN0_RX0"),
    Some("CAN0_RX1"),
    Some("CAN0_EWMC"),
    Some("EXTI5_9"),
    Some("TIMER0_BRK"),
    Some("TIMER0_UP"),
    Some("TIMER0_TRG_CMT"),
    Some("TIMER0_Channel"),
    Some("TIMER1"),
    Some("TIMER2"),
    Some("TIMER3"),
    Some("I2C0_EV"),
    Some("I2C0_ER"),
    Some("I2C1_EV"),
    Some("I2C1_ER"),
    Some("SPI0"),
    Some("SPI1"),
    Some("USART0"),
    Some("USART1"),
    Some("USART2"),
    Some("EXTI10_15"),
    Some("RTC_Alarm"),
    Some("USBD_WKUP"),
    Some("TIMER7_BRK"),
    Some("TIMER7_UP"),
    Some("TIMER7_TRG_CMT"),
    Some("TIMER7_Channel"),
    Some("ADC2"),
    Some("EXMC"),
    Some("SDIO"),
    Some("TIMER4"),
    Some("SPI2"),
    Some("UART3"),
    Some("UART4"),
    Some("TIMER5"),
    Some("TIMER6"),
    Some("DMA1_Channel0"),
    Some("DMA1_Channel1"),
    Some("DMA1_Channel2"),
    Some("DMA1_Channel3_4"),
];

/// GD32F1x0, from `startup_gd32f1x0.s` (one startup file for the whole series): IRQ 0 through 73,
/// with the vendor's reserved words where it has them.
const F1X0: &[Option<&str>] = &[
    Some("WWDGT"),
    Some("LVD"),
    Some("RTC"),
    Some("FMC"),
    Some("RCU"),
    Some("EXTI0_1"),
    Some("EXTI2_3"),
    Some("EXTI4_15"),
    Some("TSI"),
    Some("DMA_Channel0"),
    Some("DMA_Channel1_2"),
    Some("DMA_Channel3_4"),
    Some("ADC_CMP"),
    Some("TIMER0_BRK_UP_TRG_COM"),
    Some("TIMER0_Channel"),
    Some("TIMER1"),
    Some("TIMER2"),
    Some("TIMER5_DAC"),
    None,
    Some("TIMER13"),
    Some("TIMER14"),
    Some("TIMER15"),
    Some("TIMER16"),
    Some("I2C0_EV"),
    Some("I2C1_EV"),
    Some("SPI0"),
    Some("SPI1"),
    Some("USART0"),
    Some("USART1"),
    None,
    Some("CEC"),
    None,
    Some("I2C0_ER"),
    None,
    Some("I2C1_ER"),
    Some("I2C2_EV"),
    Some("I2C2_ER"),
    Some("USBD_LP"),
    Some("USBD_HP"),
    None,
    None,
    None,
    Some("USBDWakeUp"),
    Some("CAN0_TX"),
    Some("CAN0_RX0"),
    Some("CAN0_RX1"),
    Some("CAN0_SCE"),
    Some("SLCD"),
    Some("DMA_Channel5_6"),
    None,
    None,
    Some("SPI2"),
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    Some("CAN1_TX"),
    Some("CAN1_RX0"),
    Some("CAN1_RX1"),
    Some("CAN1_SCE"),
];

/// MM32SPIN05 (the MM32SPIN0x "q" series), from MindMotion's `startup_mm32spin0x_q_keil.s`
/// (LibSamples V1.13, `Device/MM32SPIN0x_q/Source/KEIL_StartAsm`): IRQ 0 (`WWDG`) through 31, the
/// vendor's reserved words where it has them. The vendor names its handlers `*_IRQHandler`; the
/// stems are kept (`ADC1_COMP`, `UART1`, `DMA1_Channel2_3`).
const MM32SPIN0X_Q: &[Option<&str>] = &[
    Some("WWDG"),
    Some("PVD"),
    None,
    Some("FLASH"),
    Some("RCC"),
    Some("EXTI0_1"),
    Some("EXTI2_3"),
    Some("EXTI4_15"),
    Some("HWDIV"),
    Some("DMA1_Channel1"),
    Some("DMA1_Channel2_3"),
    Some("DMA1_Channel4_5"),
    Some("ADC1_COMP"),
    Some("TIM1_BRK_UP_TRG_COM"),
    Some("TIM1_CC"),
    Some("TIM2"),
    Some("TIM3"),
    None,
    None,
    Some("TIM14"),
    None,
    Some("TIM16"),
    Some("TIM17"),
    Some("I2C1"),
    None,
    Some("SPI1"),
    Some("SPI2"),
    Some("UART1"),
    Some("UART2"),
    None,
    None,
    None,
];

/// The peripherals with more than one model, and the models each takes. The cfg names are declared
/// to rustc's check-cfg from this list.
const MODEL_CFGS: &[(&str, &[&str])] = &[
    // CRL/CRH nibbles on APB2 vs CTL/OMODE/OSPD/PUD with the per-pin AFSEL mux on AHB; the MM32
    // has the CRL/CRH nibbles on AHB with a per-pin AFRL/AFRH mux.
    ("hal_gpio", &["crl", "ctl", "mm32"]),
    // The F10x RCC and the F1x0 RCU: the enable-bit map, the flash wait states, the ADC clock.
    // The MM32 RCC: no PLL, the HSI's 72 MHz output, the ADC clock divider in the ADC itself.
    ("hal_clock", &["f10x", "f1x0", "mm32"]),
    // The F10x USART (one DATA register) and the F1x0 USART (separate RDATA/TDATA, the ISR/ICR
    // style status).
    // The MM32 UART (`TDR`/`RDR`/`CSR`/`ISR`/`IER`/`ICR`/`GCR`/`CCR`/`BRR`/`FRA`).
    ("hal_usart", &["f10x", "f1x0", "mm32"]),
    // The F10x ADC carries the ADC0/ADC1 sync mode (`CTL0.SYNCM`, bits 19:16, used on ADC0 only)
    // and the second instance it pairs with; the F1x0 ADC has one instance and no SYNCM field
    // (gd32f10x_adc.h, gd32f1x0_adc.h). Every other ADC register and bit the HAL programs is the
    // same on both, so the regular and injected groups are one model.
    // The MM32 ADC (`ADCFG`/`ADCR`/`ADCHS`/`ADSTA`, a data register per channel, no injected
    // group) is its own model.
    ("hal_adc", &["f10x", "f1x0", "mm32"]),
    // The DMA request map: which channel carries each USART's receive and transmit, and so which
    // vector (the F10x one vector per channel, the F1x0 channels grouped in pairs).
    // The MM32 DMA1 has the F1x0 request map for its two UARTs and its own vector names.
    ("hal_dma", &["f10x", "f1x0", "mm32"]),
    // The vendor vector list the chip links.
    ("hal_vectors", &["f10x_md", "f10x_hd", "f1x0", "mm32"]),
    // The GD32 event-model I2C and the MM32's DesignWare-style I2C (`TAR`, the FIFOs, `TX_ABRT`).
    ("hal_i2c", &["gd", "mm32"]),
    // The debug block: `DBG_CTL0` at `0xE004_2004` on the GD32, `DBGMCU_CR` at `0x4001_3404`.
    ("hal_dbg", &["gd", "mm32"]),
    // The GD32 SPI. The MM32 SPI is another model this HAL does not carry (no image uses SPI).
    ("hal_spi", &["gd"]),
    // The core: the Cortex-M3 (the DWT cycle counter, four priority bits) or the Cortex-M0 (no
    // DWT, two priority bits).
    ("hal_core", &["m3", "m0"]),
];

const CHIPS: &[ChipRow] = &[
    ChipRow {
        feature: "gd32f103c8",
        flash_kib: 64,
        page_bytes: 1024,
        ram_kib: 20,
        models: &[
            ("hal_gpio", "crl"),
            ("hal_clock", "f10x"),
            ("hal_usart", "f10x"),
            ("hal_adc", "f10x"),
            ("hal_dma", "f10x"),
            ("hal_vectors", "f10x_md"),
            ("hal_i2c", "gd"),
            ("hal_dbg", "gd"),
            ("hal_spi", "gd"),
            ("hal_core", "m3"),
        ],
        vectors: F10X_MD,
        table_entries: GD32_TABLE_ENTRIES,
    },
    ChipRow {
        feature: "gd32f103rc",
        flash_kib: 256,
        page_bytes: 2048,
        ram_kib: 48,
        models: &[
            ("hal_gpio", "crl"),
            ("hal_clock", "f10x"),
            ("hal_usart", "f10x"),
            ("hal_adc", "f10x"),
            ("hal_dma", "f10x"),
            ("hal_vectors", "f10x_hd"),
            ("hal_i2c", "gd"),
            ("hal_dbg", "gd"),
            ("hal_spi", "gd"),
            ("hal_core", "m3"),
        ],
        vectors: F10X_HD,
        table_entries: GD32_TABLE_ENTRIES,
    },
    ChipRow {
        feature: "gd32f130c8",
        flash_kib: 64,
        page_bytes: 1024,
        ram_kib: 8,
        models: &[
            ("hal_gpio", "ctl"),
            ("hal_clock", "f1x0"),
            ("hal_usart", "f1x0"),
            ("hal_adc", "f1x0"),
            ("hal_dma", "f1x0"),
            ("hal_vectors", "f1x0"),
            ("hal_i2c", "gd"),
            ("hal_dbg", "gd"),
            ("hal_spi", "gd"),
            ("hal_core", "m3"),
        ],
        vectors: F1X0,
        table_entries: GD32_TABLE_ENTRIES,
    },
    ChipRow {
        feature: "mm32spin05",
        flash_kib: 32,
        page_bytes: 1024,
        ram_kib: 4,
        models: &[
            ("hal_gpio", "mm32"),
            ("hal_clock", "mm32"),
            ("hal_usart", "mm32"),
            ("hal_adc", "mm32"),
            ("hal_dma", "mm32"),
            ("hal_vectors", "mm32"),
            ("hal_i2c", "mm32"),
            ("hal_dbg", "mm32"),
            ("hal_core", "m0"),
        ],
        vectors: MM32SPIN0X_Q,
        table_entries: MM32_TABLE_ENTRIES,
    },
];

/// Device-interrupt entries in the GD32 flash tables. With the 16 system words the `.vector_table`
/// section is 128 words, 512 B, so `.text` starts on a 512 B boundary. That alignment is a measured
/// silicon constraint carried over from `motorcontroller/vectors`: these parts run 2 flash wait
/// states with no cache or prefetch, and a shorter table moved the hot paths and cost the F130
/// control ticks (silicon, 2026-07-26: 74 entries 0.9960, 96 entries 0.9997, 128 total 1.0000).
const GD32_TABLE_ENTRIES: usize = 112;

/// The MM32SPIN05's device entries: its vendor list, 32 words. cortex-m-rt allows at most 32 device
/// vectors on a Cortex-M0 (ARMv6-M has 32 external interrupts), and the part's flash and its
/// zero-wait behaviour carry no alignment finding like the GD32's, so its table is not padded.
const MM32_TABLE_ENTRIES: usize = 32;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    for (name, values) in MODEL_CFGS {
        let list: Vec<String> = values.iter().map(|v| format!("\"{v}\"")).collect();
        println!(
            "cargo:rustc-check-cfg=cfg({name}, values({}))",
            list.join(", ")
        );
    }

    println!("cargo:rustc-check-cfg=cfg(hal_chip)");
    let enabled: Vec<&ChipRow> = CHIPS
        .iter()
        .filter(|c| env::var_os(format!("CARGO_FEATURE_{}", c.feature.to_uppercase())).is_some())
        .collect();
    // None or two: emit nothing and let src/lib.rs's compile_error! name the problem.
    let [chip] = enabled.as_slice() else {
        return;
    };
    // Exactly one chip: the register models and drivers are compiled (the `pincap` feature alone
    // builds only the host's pin tables).
    println!("cargo:rustc-cfg=hal_chip");
    for (name, model) in chip.models {
        println!("cargo:rustc-cfg={name}=\"{model}\"");
    }

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    fs::write(
        out.join("interrupts.rs"),
        interrupts_rs(chip.vectors, chip.table_entries),
    )
    .expect("interrupts.rs");
    let page = match chip.page_bytes {
        1024 => "PageSize::K1",
        2048 => "PageSize::K2",
        other => panic!("no PageSize for a {other} B page"),
    };
    fs::write(
        out.join("memory.rs"),
        format!(
            "/// Flash size, KiB (hal/build.rs, the chip table).\n\
             pub(crate) const FLASH_KIB: u16 = {};\n\
             /// Flash page size (hal/build.rs, the chip table).\n\
             pub(crate) const FLASH_PAGE: PageSize = {page};\n\
             /// SRAM size, KiB (hal/build.rs, the chip table).\n\
             pub(crate) const RAM_KIB: u16 = {};\n",
            chip.flash_kib, chip.ram_kib
        ),
    )
    .expect("memory.rs");
    println!("cargo:flash_kib={}", chip.flash_kib);
    println!("cargo:page_bytes={}", chip.page_bytes);
    println!("cargo:ram_kib={}", chip.ram_kib);
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("arm") {
        fs::write(out.join("device.x"), device_x(chip.vectors)).expect("device.x");
        println!("cargo:rustc-link-search={}", out.display());
    }
}

fn interrupts_rs(vectors: &[Option<&str>], table_entries: usize) -> String {
    assert!(
        vectors.len() <= table_entries,
        "vector list longer than the flash table"
    );
    let mut s = String::new();
    s.push_str("/// The compiled chip's device interrupts, numbered as the vendor's `IRQn_Type` and named\n");
    s.push_str("/// as its startup file's `*_IRQHandler` stems. A handler is defined in the image by this\n");
    s.push_str("/// name (`#[interrupt] fn ADC0_1()`).\n");
    s.push_str("#[derive(Clone, Copy, Debug, PartialEq, Eq)]\n");
    s.push_str("#[allow(non_camel_case_types)]\n#[repr(u16)]\npub enum Interrupt {\n");
    for (n, v) in vectors.iter().enumerate() {
        if let Some(name) = v {
            let _ = writeln!(s, "    #[allow(missing_docs)]\n    {name} = {n},");
        }
    }
    s.push_str("}\n\n");
    s.push_str("// SAFETY: every variant is a device IRQ number of the compiled chip, from its vendor table.\n");
    s.push_str("unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {\n");
    s.push_str("    #[inline]\n    fn number(self) -> u16 {\n        self as u16\n    }\n}\n\n");

    s.push_str("#[cfg(target_arch = \"arm\")]\nextern \"C\" {\n");
    for name in vectors.iter().flatten() {
        let _ = writeln!(s, "    fn {name}();");
    }
    s.push_str("}\n\n");
    s.push_str("/// One flash vector-table word: a handler or a reserved zero.\n");
    s.push_str("#[cfg(target_arch = \"arm\")]\n#[doc(hidden)]\npub union Vector {\n");
    s.push_str("    handler: unsafe extern \"C\" fn(),\n    reserved: u32,\n}\n\n");
    s.push_str("/// The device part of the flash vector table, which cortex-m-rt's `device` feature links.\n");
    let _ = writeln!(
        s,
        "#[cfg(target_arch = \"arm\")]\n#[doc(hidden)]\n#[link_section = \".vector_table.interrupts\"]\n#[no_mangle]\npub static __INTERRUPTS: [Vector; {table_entries}] = ["
    );
    for n in 0..table_entries {
        match vectors.get(n).copied().flatten() {
            Some(name) => {
                let _ = writeln!(s, "    Vector {{ handler: {name} }},");
            }
            None => s.push_str("    Vector { reserved: 0 },\n"),
        }
    }
    s.push_str("];\n");
    s
}

fn device_x(vectors: &[Option<&str>]) -> String {
    let mut s = String::from(
        "/* Generated by the hal crate's build.rs from the chip's vendor vector list. */\n",
    );
    for name in vectors.iter().flatten() {
        let _ = writeln!(s, "PROVIDE({name} = DefaultHandler);");
    }
    s
}
