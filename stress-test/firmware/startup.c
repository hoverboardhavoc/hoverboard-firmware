/* Cortex-M3 (GD32F103) startup: vector table + reset path (data copy, bss zero, call main).
 *
 * The runtime-hal `build_assets/gd-spl/gd32f10x/startup.S` the spec points at is a snippet-EXTRACTION
 * harness stub: its `_start` jumps straight to `regcmp_test` and never copies .data, zeroes .bss, nor
 * calls `main` (it exists only to give the ELF a valid vector table for the register-capture tool). It
 * cannot boot a real firmware. This is the same clean GCC Cortex-M3 crt pattern the spec's other
 * reference uses (`../BalanceAgain/tools/recompile/startup.c`), in C for clarity, with the full 16-entry
 * core vector table the GD32 boots from at 0x08000000.
 *
 * No peripheral interrupts are used (the firmware is fully polled / busy-spin), so only the core
 * exception vectors are populated; every exception falls into a busy-spin Default_Handler (NEVER wfi).
 */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0U;
    }
    main();
    for (;;) {
        /* busy-spin; main() never returns */
    }
}

void Default_Handler(void)
{
    for (;;) {
        /* busy-spin on any unexpected exception (NEVER wfi: GD32 SWD-lockout rule) */
    }
}

#define WEAK __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK;
void HardFault_Handler(void) WEAK;
void MemManage_Handler(void) WEAK;
void BusFault_Handler(void) WEAK;
void UsageFault_Handler(void) WEAK;
void SVC_Handler(void) WEAK;
void DebugMon_Handler(void) WEAK;
void PendSV_Handler(void) WEAK;
void SysTick_Handler(void) WEAK;

__attribute__((section(".isr_vector"), used))
void (*const g_vectors[])(void) = {
    (void (*)(void)) & _estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};
