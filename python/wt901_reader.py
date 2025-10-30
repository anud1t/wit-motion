import argparse
import struct
import time
import serial


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_read_holding_regs(addr: int, reg: int, count: int) -> bytes:
    frame = bytes([
        addr & 0xFF,
        0x03,
        (reg >> 8) & 0xFF,
        reg & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])
    crc = crc16_modbus(frame)
    return frame + struct.pack('<H', crc)


def parse_payload(payload: bytes):
    # payload layout for 12 regs (24 bytes):
    # AccX, AccY, AccZ, GyrX, GyrY, GyrZ, HX, HY, HZ, AngX, AngY, AngZ (each int16)
    vals = struct.unpack('>12h', payload)
    acc = tuple(round(v / 32768.0 * 16.0, 3) for v in vals[0:3])
    gyr = tuple(round(v / 32768.0 * 2000.0, 3) for v in vals[3:6])
    # vendor Python scaled mag with *13/1000; here we keep raw uT-like scaling similar
    mag = tuple(round(v * 13 / 1000.0, 3) for v in vals[6:9])
    ang = tuple(round(v / 32768.0 * 180.0, 3) for v in vals[9:12])
    return acc, gyr, ang, mag


def parse_temp(payload: bytes):
    # TEMP at 0x40 as int16, scale per vendor convention: value/100? WT901 often 0.01 C per LSB
    v = struct.unpack('>h', payload)[0]
    return round(v / 100.0, 2)


def parse_quat(payload: bytes):
    q = struct.unpack('>4h', payload)
    return tuple(round(v / 32768.0, 6) for v in q)


def build_write_single(addr: int, reg: int, value: int) -> bytes:
    frame = bytes([
        addr & 0xFF,
        0x06,
        (reg >> 8) & 0xFF,
        reg & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])
    crc = crc16_modbus(frame)
    return frame + struct.pack('<H', crc)


def main():
    ap = argparse.ArgumentParser(description="WT901C485 Modbus reader")
    ap.add_argument('--port', default='/dev/ttyCH341USB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x50)
    ap.add_argument('--interval', type=float, default=0.2)
    # Optional config writes
    ap.add_argument('--set-rate', type=str, default=None, help='Set output rate: 2,5,10,20,50,100,200 (Hz)')
    ap.add_argument('--set-baud', type=int, default=None, help='Set baud code: 4800,9600,19200,38400,57600,115200,230400,460800,921600')
    ap.add_argument('--enable-q', action='store_true', help='Enable quaternion output in RSW')
    ap.add_argument('--disable-q', action='store_true', help='Disable quaternion output in RSW')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.3)

    # Optional configuration writes (unlock -> write -> save)
    def write_reg(reg: int, val: int):
        ser.write(build_write_single(args.addr, reg, val))
        time.sleep(0.05)

    if args.set_rate or args.set_baud or args.enable_q or args.disable_q:
        # unlock
        write_reg(0x0069, 0xB588)
        if args.set_rate:
            rate_map = {
                '2': 0x04, '5': 0x05, '10': 0x06, '20': 0x07, '50': 0x08,
                '100': 0x09, '200': 0x0B,
            }
            code = rate_map.get(args.set_rate)
            if code is not None:
                write_reg(0x0003, code)
        if args.set_baud:
            baud_map = {4800:1,9600:2,19200:3,38400:4,57600:5,115200:6,230400:7,460800:8,921600:9}
            code = baud_map.get(args.set_baud)
            if code is not None:
                write_reg(0x0004, code)
        if args.enable_q or args.disable_q:
            # Read current RSW (function 03)
            ser.write(build_read_holding_regs(args.addr, 0x0002, 1))
            time.sleep(0.02)
            resp = ser.read(3)
            payload = b''
            if len(resp) == 3 and resp[1] == 0x03:
                payload = ser.read(resp[2])
                _ = ser.read(2)
            if len(payload) == 2:
                rsw = struct.unpack('>H', payload)[0]
                if args.enable_q:
                    rsw |= 0x200
                if args.disable_q:
                    rsw &= ~0x200
                write_reg(0x0002, rsw & 0x0FFF)
        # save
        write_reg(0x0000, 0x0000)
    try:
        while True:
            req = build_read_holding_regs(args.addr, 0x0034, 12)
            ser.write(req)
            time.sleep(0.02)
            # Expected response: addr, 0x03, bytecount(24), 24 data bytes, CRC(2)
            hdr = ser.read(3)
            if len(hdr) < 3:
                time.sleep(args.interval)
                continue
            r_addr, func, bytecount = hdr[0], hdr[1], hdr[2]
            if r_addr != (args.addr & 0xFF) or func != 0x03 or bytecount != 24:
                # drain and continue
                ser.reset_input_buffer()
                time.sleep(args.interval)
                continue
            payload = ser.read(bytecount)
            crc = ser.read(2)
            frame_wo_crc = bytes(hdr) + payload
            calc = crc16_modbus(frame_wo_crc)
            if len(payload) != 24 or len(crc) != 2 or calc != struct.unpack('<H', crc)[0]:
                time.sleep(args.interval)
                continue
            acc, gyr, ang, mag = parse_payload(payload)

            # Temperature (0x0040, 1 reg)
            temp_c = None
            ser.write(build_read_holding_regs(args.addr, 0x0040, 1))
            th = ser.read(3)
            if len(th) == 3 and th[1] == 0x03 and th[2] == 2:
                tp = ser.read(2)
                _ = ser.read(2)
                if len(tp) == 2:
                    temp_c = parse_temp(tp)

            # Quaternion (0x0051..0x0054)
            quat = None
            ser.write(build_read_holding_regs(args.addr, 0x0051, 4))
            qh = ser.read(3)
            if len(qh) == 3 and qh[1] == 0x03 and qh[2] == 8:
                qp = ser.read(8)
                _ = ser.read(2)
                if len(qp) == 8:
                    quat = parse_quat(qp)

            # Pretty, compact output
            acc_s = f"({acc[0]:6.3f},{acc[1]:6.3f},{acc[2]:6.3f})"
            gyr_s = f"({gyr[0]:7.3f},{gyr[1]:7.3f},{gyr[2]:7.3f})"
            ang_s = f"({ang[0]:6.3f},{ang[1]:6.3f},{ang[2]:6.3f})"
            mag_s = f"({mag[0]:7.3f},{mag[1]:7.3f},{mag[2]:7.3f})"
            extra = ""
            if temp_c is not None:
                extra += f" T:{temp_c:5.2f}C"
            if quat is not None:
                extra += f" q:({quat[0]:.4f},{quat[1]:.4f},{quat[2]:.4f},{quat[3]:.4f})"
            print(f"acc:{acc_s} gyr:{gyr_s} ang:{ang_s} mag:{mag_s}{extra}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()


