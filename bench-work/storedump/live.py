import os, sys, struct
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from parse import parse_header, committed, rec_size, PAGE, PAGE_HEADER_LEN
NAMES = {0x01:"node.address",0x02:"link.set",0x10:"device.name",0x20:"motor.current_limit",
 0x21:"motor.method",0x22:"control.mode",0x23:"control.rider_required",0x24:"control.battery_floor",
 0x25:"control.frame",0x26:"control.side",0x40:"board.self_hold",0x41:"board.vbatt",
 0x42:"board.buzzer",0x43:"led.green",0x44:"led.orange",0x45:"led.red",0x46:"pad.a",0x47:"pad.b",
 0x48:"imu.scl_pin",0x49:"imu.sda_pin",0x4A:"motor.hall_a",0x4B:"motor.hall_b",0x4C:"motor.hall_c",
 0x4D:"motor.gate_hi_a",0x4E:"motor.gate_hi_b",0x4F:"motor.gate_hi_c",0x50:"motor.gate_lo_a",
 0x51:"motor.gate_lo_b",0x52:"motor.gate_lo_c",0x54:"motor.sense_a",0x55:"motor.sense_b",
 0x60:"imu.model",0x61:"imu.gyro_bias",0x62:"motor.direction",0x63:"motor.align_offset",
 0x64:"motor.dead_time",0x65:"imu.axis_sign",0x66:"motor.current_sense",0x67:"motor.current_cal",
 0x68:"imu.axis_role",0x69:"board.vbatt_cal",0x6B:"motor.noise_floor",0x70:"attitude.level_trim",
 0x71:"control.gain_a",0x72:"control.gain_b",0x73:"control.drive_lean",0x74:"control.gain_max",
 0x75:"control.drive_turn"}
def dec(tag, b):
    if tag==1: return str(b[0])
    if tag==2: return str(int.from_bytes(b,"little"))
    if tag==3: return str(int.from_bytes(b,"little"))
    if tag==5: return str(int.from_bytes(b,"little",signed=True))
    if tag==6: return str(int.from_bytes(b,"little",signed=True))
    if tag==10: return repr(b.decode("utf-8","replace"))
    return b.hex()
for who, f in (("SLAVE", "slave-f130.bin"), ("MASTER", "master-f103.bin")):
    r = open(os.path.join(HERE,f),"rb").read()
    cur = PAGE_HEADER_LEN; latest = {}
    while True:
        st,h = parse_header(r,cur)
        if st!="good": break
        nxt = cur+rec_size(h['ln'])
        if nxt>PAGE: break
        if committed(r,cur,h):
            latest[(h['field_id'],h['index'])]=(h['tag'], r[cur+8:cur+8+h['ln']])
        cur=nxt
        if cur==PAGE: break
    print(f"--- {who}: {len(latest)} live keys")
    for (fid,idx) in sorted(latest):
        tag,b = latest[(fid,idx)]
        print(f"  0x{fid:02X}:{idx}  {NAMES.get(fid,'?'):24s} = {dec(tag,b)}")
