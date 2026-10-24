#!/usr/bin/env python3
"""hangdump.py PORT CTRL_OBS_HEX SECONDS: poll the tick word; when it freezes, dump core/peripheral state repeatedly."""
import sys, time, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from swdobs import Ocd
port, base, secs = int(sys.argv[1]), int(sys.argv[2], 16), float(sys.argv[3])
o = Ocd("localhost", port)
REGS = [("ICSR",0xE000ED04),("SHCSR",0xE000ED24),("CFSR",0xE000ED28),("HFSR",0xE000ED2C),("MMFAR",0xE000ED34),("BFAR",0xE000ED38),
        ("DHCSR",0xE000EDF0),("SYST_CSR",0xE000E010),("SYST_CVR",0xE000E018),("CYCCNT",0xE0001004),
        ("NVIC_ISER0",0xE000E100),("NVIC_ISER1",0xE000E104),("NVIC_ISPR0",0xE000E200),("NVIC_IABR0",0xE000E300),("NVIC_IABR1",0xE000E304),
        ("RCU_CTL",0x40021000),("RCU_CFG0",0x40021004),("RCU_APB2RST",0x4002100C),("RCU_APB1RST",0x40021010),("RCU_AHBEN",0x40021014),("RCU_APB2EN",0x40021018),("RCU_APB1EN",0x4002101C),("RCU_RSTSCK",0x40021024),
        ("FWDGT_CTL",0x40003000),("FWDGT_PSC",0x40003004),("FWDGT_RLD",0x40003008),("FWDGT_STAT",0x4000300C),
        ("DMA_INTF",0x40020000),("DMA_CH3CTL",0x40020044),("DMA_CH3CNT",0x40020048),("DMA_CH3PADDR",0x4002004C),("DMA_CH3MADDR",0x40020050),
        ("DMA_CH4CTL",0x40020058),("DMA_CH4CNT",0x4002005C),
        ("USART1_STAT",0x40004400),("USART1_CTL0",0x40004404),("USART1_CTL2",0x4000440C),("USART1_BAUD",0x40004410),
        ("DBG_CTL0",0xE0042004),("TIMER0_CCHP",0x40012C44),("TIMER0_CTL0",0x40012C00),("TIMER0_DMAINTEN",0x40012C0C)]
def dump(tag):
    print(f"--- {tag} t={time.time()-t0:.2f}")
    for n,a in REGS:
        try: v=o.read_words(a,1)[0]; print(f"{n:14s} {v:08x}")
        except Exception as e: print(f"{n:14s} ERR {e}")
    sys.stdout.flush()
t0=time.time(); last=None; same=0; dumped=0
while time.time()-t0 < secs:
    w=o.read_words(base,39)
    tick=w[2]
    print(f"{time.time()-t0:.2f} boot={w[1]} tick={tick} tm={w[11]:08x} sf={w[12]:08x} ms={w[20]:08x} fault={w[23]:08x} stack={w[30]:08x} rst={w[38]:08x}"); sys.stdout.flush()
    if tick==last: same+=1
    else: same=0
    last=tick
    if same>=2 and dumped<3:
        dump(f"FROZEN #{dumped}"); dumped+=1
    time.sleep(0.15)
o.close()
