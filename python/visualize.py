import argparse
import time
import struct
import threading
from collections import deque

import numpy as np
import serial
from serial import SerialException
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


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
    vals = struct.unpack('>12h', payload)
    acc_g = np.array([v / 32768.0 * 16.0 for v in vals[0:3]], dtype=float)
    gyr_dps = np.array([v / 32768.0 * 2000.0 for v in vals[3:6]], dtype=float)
    mag_u = np.array([v * 13 / 1000.0 for v in vals[6:9]], dtype=float)
    ang_deg = np.array([v / 32768.0 * 180.0 for v in vals[9:12]], dtype=float)
    return acc_g, gyr_dps, ang_deg, mag_u


def parse_quat(payload: bytes):
    q = struct.unpack('>4h', payload)
    return np.array([v / 32768.0 for v in q], dtype=float)  # [w, x, y, z]


def quat_to_rotmat(q):
    # q = [w, x, y, z]
    w, x, y, z = q
    n = w*w + x*x + y*y + z*z
    if n == 0:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    R = np.array([
        [1 - (yy + zz),     xy - wz,         xz + wy],
        [xy + wz,           1 - (xx + zz),   yz - wx],
        [xz - wy,           yz + wx,         1 - (xx + yy)],
    ], dtype=float)
    return R


G = 9.80665


class ReaderThread(threading.Thread):
    def __init__(self, port: str, baud: int, addr: int, maxlen: int = 500):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.addr = addr
        self.ser = None
        self.running = True
        self.latest = {
            'acc_g': np.zeros(3),
            'lin_acc_earth': np.zeros(3),
            'quat': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.t = deque(maxlen=maxlen)
        self.lin_acc_hist = deque(maxlen=maxlen)  # earth frame m/s^2

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.3)
        except SerialException as e:
            print(f"Error: cannot open serial port {self.port} ({e}).\n"
                  f"- Check permissions: ls -l {self.port}\n"
                  f"- If group is 'dialout', add your user: sudo usermod -aG dialout $USER; re-login\n"
                  f"- Or temporary: sudo chmod a+rw {self.port}")
            self.running = False
            return
        try:
            while self.running:
                # Read combined payload (0x0034.. count 12)
                self.ser.write(build_read_holding_regs(self.addr, 0x0034, 12))
                time.sleep(0.01)
                hdr = self.ser.read(3)
                if len(hdr) != 3 or hdr[1] != 0x03 or hdr[2] != 24:
                    self.ser.reset_input_buffer()
                    continue
                payload = self.ser.read(24)
                _ = self.ser.read(2)  # CRC ignored after length check
                if len(payload) != 24:
                    continue
                acc_g, gyr_dps, ang_deg, mag_u = parse_payload(payload)

                # Quaternion
                self.ser.write(build_read_holding_regs(self.addr, 0x0051, 4))
                qh = self.ser.read(3)
                quat = None
                if len(qh) == 3 and qh[1] == 0x03 and qh[2] == 8:
                    qp = self.ser.read(8)
                    _ = self.ser.read(2)
                    if len(qp) == 8:
                        quat = parse_quat(qp)
                if quat is None:
                    quat = self.latest['quat']

                # Linear acceleration (earth frame): R * (acc_body_mps2) - g
                R_be = quat_to_rotmat(quat)  # body -> earth
                acc_body_mps2 = acc_g * G
                acc_earth = R_be @ acc_body_mps2
                lin_acc_earth = acc_earth - np.array([0.0, 0.0, G])

                self.latest = {
                    'acc_g': acc_g,
                    'lin_acc_earth': lin_acc_earth,
                    'quat': quat,
                }
                self.t.append(time.time())
                self.lin_acc_hist.append(lin_acc_earth.copy())
        except Exception as e:
            # Graceful stop without noisy traceback in UI loop
            print(f"Serial read loop stopped: {e}")
            self.running = False
        finally:
            try:
                self.ser.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description='Live visualize WT901 IMU')
    ap.add_argument('--port', default='/dev/ttyCH341USB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x50)
    ap.add_argument('--window', type=float, default=10.0, help='seconds for history plots')
    args = ap.parse_args()

    # Preflight: check we can open the serial port to provide a friendly error
    try:
        _ser = serial.Serial(args.port, args.baud, timeout=0.3)
        _ser.close()
    except SerialException as e:
        print(f"Error: cannot open serial port {args.port} ({e}).\n"
              f"Fix: add your user to 'dialout' and re-login:\n"
              f"  sudo usermod -aG dialout $USER && newgrp dialout\n"
              f"Or temporarily grant access until unplug/reboot:\n"
              f"  sudo chmod a+rw {args.port}")
        return

    reader = ReaderThread(args.port, args.baud, args.addr, maxlen=int(args.window * 100))
    reader.start()

    plt.style.use('ggplot')
    fig = plt.figure(figsize=(12, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection='3d')
    axg = fig.add_subplot(3, 2, 2)
    axX = fig.add_subplot(3, 2, 4)
    axY = fig.add_subplot(3, 2, 6)

    # 3D body axes with labels and legend
    axes_lines = [
        ax3d.plot([0, 1], [0, 0], [0, 0], color='r', lw=2, label='X (red)')[0],
        ax3d.plot([0, 0], [0, 1], [0, 0], color='g', lw=2, label='Y (green)')[0],
        ax3d.plot([0, 0], [0, 0], [0, 1], color='b', lw=2, label='Z (blue)')[0],
    ]
    ax3d.set_title('Orientation (body axes)')
    ax3d.set_xlabel('X')
    ax3d.set_ylabel('Y')
    ax3d.set_zlabel('Z')
    ax3d.set_xlim(-1, 1)
    ax3d.set_ylim(-1, 1)
    ax3d.set_zlim(-1, 1)
    ax3d.set_box_aspect([1, 1, 1])
    ax3d.legend(loc='upper left')

    # G-force bar chart for linear acceleration magnitude and components
    bars = axg.bar(['Ex', 'Ey', 'Ez', '|E|/g'], [0, 0, 0, 0], color=['#e74c3c', '#27ae60', '#2980b9', '#7f8c8d'])
    axg.set_ylim(-2.0, 2.0)
    axg.set_title('Linear acceleration (earth frame, g)')

    # Time series for Ex and Ey (longitudinal/lateral if aligned)
    t_hist = deque(maxlen=reader.lin_acc_hist.maxlen)
    Ex_line, = axX.plot([], [], 'r-')
    Ey_line, = axY.plot([], [], 'g-')
    axX.set_ylabel('Ex (m/s^2)')
    axY.set_ylabel('Ey (m/s^2)')
    axY.set_xlabel('time (s)')

    def update(_):
        latest = reader.latest
        q = latest['quat']
        R = quat_to_rotmat(q)
        # Body axes endpoints in earth frame
        ex = R @ np.array([1, 0, 0])
        ey = R @ np.array([0, 1, 0])
        ez = R @ np.array([0, 0, 1])
        for line, vec in zip(axes_lines, (ex, ey, ez)):
            line.set_data([0, vec[0]], [0, vec[1]])
            line.set_3d_properties([0, vec[2]])

        lin_e = latest['lin_acc_earth']
        vals_g = [lin_e[0]/G, lin_e[1]/G, lin_e[2]/G, np.linalg.norm(lin_e)/G]
        for b, v in zip(bars, vals_g):
            b.set_height(v)

        # History
        if len(reader.t) > 1:
            t0 = reader.t[0]
            t = np.array(reader.t) - t0
            E = np.array(reader.lin_acc_hist)
            Ex_line.set_data(t, E[:, 0])
            Ey_line.set_data(t, E[:, 1])
            axX.set_xlim(t.min(), t.max())
            axY.set_xlim(t.min(), t.max())
            axX.set_ylim(np.min(E[:, 0])-1, np.max(E[:, 0])+1)
            axY.set_ylim(np.min(E[:, 1])-1, np.max(E[:, 1])+1)

        return axes_lines + list(bars) + [Ex_line, Ey_line]

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    finally:
        reader.running = False
        reader.join(timeout=1.0)


if __name__ == '__main__':
    main()


