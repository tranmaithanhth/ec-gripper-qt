#!/usr/bin/env python3
"""EC Gripper: one Python file + gripper_settings.json.

Python 3.10+. Install once: python -m pip install "PySide6==6.8.3" "pyserial>=3.5,<4"
Run: python ec_gripper.py
Import legacy settings: python ec_gripper.py --import-settings /path/to/old/project
Sources: tranmaithanhth/ec-gripper-qt; includes the original EC-A4310 driver.
"""
from __future__ import annotations
import sys, json, copy, os
from pathlib import Path
try:
    import serial
    import PySide6
except ImportError as exc:
    raise SystemExit('Missing dependency: '+str(exc)+'\nInstall: python -m pip install "PySide6==6.8.3" "pyserial>=3.5,<4"')
if PySide6.__version__=='6.12.0' and sys.version_info<(3,12):
    raise SystemExit('PySide6 6.12.0 crashes on Python 3.10/3.11. Install PySide6==6.8.3.')
# ec_a4310_waveshare_test.py


import argparse
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial
from serial.tools import list_ports


SERIAL_BAUD = 2_000_000
CAN_BAUD_CODE_1M = 0x01
CAN_STANDARD = 0x01
CAN_MODE_NORMAL = 0x00

ERROR_TEXT = {
    0: "OK",
    1: "Motor over-temperature",
    2: "Motor over-current",
    3: "Motor under-voltage",
    4: "Encoder error",
    6: "Brake voltage too high",
    7: "DRV driver error",
}


@dataclass
class CanFrame:
    can_id: int
    data: bytes


class WaveshareUsbCanA:
    """Waveshare USB-CAN-A using the official variable-length serial protocol."""

    def __init__(self, port: str, timeout: float = 0.05):
        self.port = port
        self.ser = serial.Serial(
            port=port,
            baudrate=SERIAL_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=0.5,
        )

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def configure_1m_standard_normal(self) -> None:
        """Configure adapter: variable-length, 1 Mbps, standard frame, normal mode."""
        # Waveshare 20-byte configuration command:
        # AA 55 12 <baud> <frame> <filter x4> <mask x4> <mode>
        # <auto-retransmit> <reserved x4> <checksum>
        cmd = bytearray([
            0xAA, 0x55,
            0x12,                  # variable-length protocol configuration
            CAN_BAUD_CODE_1M,      # 1 Mbps
            CAN_STANDARD,          # 11-bit standard CAN
            0x00, 0x00, 0x00, 0x00,  # filter ID
            0x00, 0x00, 0x00, 0x00,  # mask ID
            CAN_MODE_NORMAL,
            0x00,                  # auto retransmit enabled
            0x00, 0x00, 0x00, 0x00,
            0x00,                  # checksum, filled below
        ])
        cmd[19] = sum(cmd[2:19]) & 0xFF
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        self.ser.flush()
        time.sleep(0.10)
        self.ser.reset_input_buffer()
        print("[USB-CAN-A] Configured: 1 Mbps, Standard CAN, Normal, variable-length")

    def send_can(self, can_id: int, data: bytes | bytearray | list[int]) -> None:
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("Standard CAN ID must be in range 0x000..0x7FF")
        data = bytes(data)
        if len(data) > 8:
            raise ValueError("Classic CAN payload max = 8 bytes")

        # Official Waveshare variable-length protocol:
        # AA, C0|DLC, ID-low, ID-high, DATA..., 55
        frame = bytearray([0xAA, 0xC0 | len(data), can_id & 0xFF, (can_id >> 8) & 0x07])
        frame.extend(data)
        frame.append(0x55)
        self.ser.write(frame)
        self.ser.flush()

    def recv_can(self, timeout: float = 0.20) -> Optional[CanFrame]:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != 0xAA:
                continue

            type_b = self.ser.read(1)
            if not type_b:
                continue
            t = type_b[0]

            # 0xAA 0x55 is a config/fixed-frame header, not a variable CAN frame.
            if t == 0x55:
                self.ser.read(18)
                continue

            # Variable CAN frame has upper bits 11xxxxxx.
            if (t & 0xC0) != 0xC0:
                continue

            is_extended = bool(t & 0x20)
            is_remote = bool(t & 0x10)
            dlc = t & 0x0F
            if dlc > 8:
                continue

            id_len = 4 if is_extended else 2
            id_bytes = self.ser.read(id_len)
            if len(id_bytes) != id_len:
                continue

            data = self.ser.read(0 if is_remote else dlc)
            if len(data) != (0 if is_remote else dlc):
                continue

            tail = self.ser.read(1)
            if tail != b"\x55":
                continue

            can_id = int.from_bytes(id_bytes, byteorder="little")
            return CanFrame(can_id=can_id, data=data)

        return None

    def wait_for_id(self, can_id: int, timeout: float = 0.5) -> Optional[CanFrame]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.recv_can(timeout=min(0.05, max(0.001, deadline - time.monotonic())))
            if frame is not None and frame.can_id == can_id:
                return frame
        return None


class EC_A4310:
    def __init__(self, can: WaveshareUsbCanA, motor_id: Optional[int] = None):
        self.can = can
        self.motor_id = motor_id

    def discover_id(self, timeout: float = 1.0) -> int:
        """Non-motion query. CAN ID 0x7FF, payload FF FF 00 82."""
        self.can.ser.reset_input_buffer()
        self.can.send_can(0x7FF, [0xFF, 0xFF, 0x00, 0x82])
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            fr = self.can.recv_can(0.10)
            if fr is None or fr.can_id != 0x7FF:
                continue
            d = fr.data
            print(f"[RX] ID=0x{fr.can_id:03X} DATA={d.hex(' ').upper()}")
            # Successful ID query: FF FF 01 <ID_H> <ID_L>
            if len(d) >= 5 and d[0] == 0xFF and d[1] == 0xFF and d[2] == 0x01:
                motor_id = (d[3] << 8) | d[4]
                if 1 <= motor_id <= 0x7FE:
                    self.motor_id = motor_id
                    return motor_id

        raise TimeoutError("No valid motor-ID response on CAN ID 0x7FF")

    def set_qna_mode(self) -> bool:
        self._require_id()
        mid = self.motor_id
        assert mid is not None
        payload = [(mid >> 8) & 0xFF, mid & 0xFF, 0x00, 0x02]
        self.can.ser.reset_input_buffer()
        self.can.send_can(0x7FF, payload)
        fr = self.can.wait_for_id(0x7FF, 0.4)
        # Manual requires >=500 ms before another CAN command after setting mode.
        time.sleep(0.55)
        if fr:
            print(f"[RX] ID=0x7FF DATA={fr.data.hex(' ').upper()}")
            if len(fr.data) >= 4 and fr.data[2] == 0x01 and fr.data[3] == 0x02:
                return True
        return False

    def query_position(self, timeout: float = 0.5) -> float:
        """Query code 1. Returns motor output position as float from the driver."""
        self._require_id()
        mid = self.motor_id
        assert mid is not None

        # Motor mode uint3=0x07 occupies Byte0[7:5], retain=0 -> 0xE0
        self.can.ser.reset_input_buffer()
        self.can.send_can(mid, [0xE0, 0x01])
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            fr = self.can.recv_can(0.10)
            if fr is None or fr.can_id != mid or len(fr.data) < 2:
                continue
            d = fr.data
            msg_type = (d[0] >> 5) & 0x07
            err = d[0] & 0x1F
            if err:
                raise RuntimeError(f"Motor error {err}: {ERROR_TEXT.get(err, 'Unknown')}")
            if msg_type == 0x05 and d[1] == 0x01 and len(d) >= 6:
                # Protocol states data are stored high-byte/high-bit first.
                return struct.unpack(">f", d[2:6])[0]

        raise TimeoutError("No position-query response")

    @staticmethod
    def pack_servo_position(
        position_deg: float,
        speed_rpm: float,
        current_limit_a: float,
        ack: int = 2,
    ) -> bytes:
        """
        Q&A servo position command, 64-bit packed field order:
          mode uint3 = 1
          position float32 (degrees)
          speed uint15 (0.1 rpm/count)
          current uint12 (0.1 A/count)
          ack uint2
        """
        if not 0.0 <= speed_rpm <= 3276.7:
            raise ValueError("speed_rpm out of protocol range")
        if not 0.0 <= current_limit_a <= 409.5:
            raise ValueError("current_limit_a out of protocol range")
        if ack not in (0, 1, 2, 3):
            raise ValueError("ack must be 0..3")

        speed_u15 = int(round(speed_rpm * 10.0))
        current_u12 = int(round(current_limit_a * 10.0))
        pos_bits = int.from_bytes(struct.pack(">f", float(position_deg)), "big")

        raw64 = (
            (0x01 << 61)
            | (pos_bits << 29)
            | ((speed_u15 & 0x7FFF) << 14)
            | ((current_u12 & 0x0FFF) << 2)
            | (ack & 0x03)
        )
        return raw64.to_bytes(8, "big")

    def goto_position(
        self,
        target_deg: float,
        speed_rpm: float = 1.0,
        current_limit_a: float = 0.5,
        command_time_s: float = 1.0,
        rate_hz: float = 20.0,
    ) -> None:
        """Repeatedly send a position setpoint so the 500 ms heartbeat stays alive."""
        self._require_id()
        mid = self.motor_id
        assert mid is not None

        payload = self.pack_servo_position(
            position_deg=target_deg,
            speed_rpm=speed_rpm,
            current_limit_a=current_limit_a,
            ack=2,  # request return message type 2
        )

        print(f"[TX] target={target_deg:.3f} deg, speed={speed_rpm:.2f} rpm, Imax={current_limit_a:.2f} A")
        print(f"[TX] ID=0x{mid:03X} DATA={payload.hex(' ').upper()}")

        period = 1.0 / rate_hz
        end_t = time.monotonic() + command_time_s
        last_feedback_t = 0.0

        try:
            while time.monotonic() < end_t:
                t0 = time.monotonic()
                self.can.send_can(mid, payload)

                fr = self.can.wait_for_id(mid, timeout=min(0.02, period * 0.4))
                if fr is not None:
                    fb = self.parse_feedback_type2(fr)
                    if fb and time.monotonic() - last_feedback_t > 0.15:
                        print(
                            f"  pos={fb['position']:.3f}, "
                            f"current={fb['current_a']:.2f} A, "
                            f"temp={fb['temperature_c']:.1f} C, "
                            f"err={fb['error']}"
                        )
                        last_feedback_t = time.monotonic()

                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)
        except KeyboardInterrupt:
            print("\n[!] Command interrupted.")

        # Do not send more motion commands. The driver heartbeat protection stops motion
        # if another control command is not received within 500 ms.
        print("[STOP] Position command stream ended; motor heartbeat will stop motion within ~500 ms.")

    @staticmethod
    def parse_feedback_type2(frame: CanFrame):
        d = frame.data
        if len(d) < 8:
            return None
        msg_type = (d[0] >> 5) & 0x07
        err = d[0] & 0x1F
        if msg_type != 0x02:
            return None

        position = struct.unpack(">f", d[1:5])[0]
        current_raw = int.from_bytes(d[5:7], "big", signed=True)
        current_a = current_raw / 100.0
        temperature_c = (d[7] - 50) / 2.0
        return {
            "position": position,
            "current_a": current_a,
            "temperature_c": temperature_c,
            "error": f"{err} ({ERROR_TEXT.get(err, 'Unknown')})",
        }

    def _require_id(self) -> None:
        if self.motor_id is None:
            raise RuntimeError("Motor ID is unknown. Run discover_id() first.")








# control.py
from dataclasses import dataclass, asdict
from collections import deque
import math

@dataclass(frozen=True)
class Settings:
    open_deg: float | None = None
    closed_deg: float | None = None
    speed_rpm: float = 1.0
    close_a: float = 0.5
    hold_a: float = 0.3
    contact_a: float = 0.4
    preload_deg: float = 1.0
    tolerance_deg: float = 0.25
    move_timeout_s: float = 20.0
    max_temp_c: float = 70.0

    def validate(self):
        values = asdict(self)
        if any(not math.isfinite(v) for v in values.values() if v is not None):
            raise ValueError('All settings must be finite')
        if self.open_deg is None or self.closed_deg is None:
            raise ValueError('Set open and closed angles before motion')
        span = abs(self.closed_deg-self.open_deg)
        if span < 1 or span > 360:
            raise ValueError('Calibration span must be 1–360 deg; confirm the mechanical travel')
        if not 0.1 <= self.speed_rpm <= 5:
            raise ValueError('Test speed must be 0.1–5 rpm')
        if not 0.1 <= self.hold_a <= self.close_a <= 3:
            raise ValueError('Require 0.1 <= I_hold <= I_close <= 3 A (application test cap)')
        if not 0.1 <= self.contact_a <= self.close_a:
            raise ValueError('Contact threshold must be 0.1 A to I_close')
        if not 0 <= self.preload_deg <= min(5,span):
            raise ValueError('Invalid preload')
        if not 0.05 <= self.tolerance_deg <= min(0.5,span/4):
            raise ValueError('Invalid tolerance')
        if not 2 <= self.move_timeout_s <= 120:
            raise ValueError('Timeout must be 2–120 s')
        if not 20 <= self.max_temp_c <= 70:
            raise ValueError('Temperature limit must be 20–70 C')

    def position(self, raw):
        self.validate()
        if not isinstance(raw,int) or not 0<=raw<=1000:
            raise ValueError('Position must be integer 0–1000')
        return self.open_deg+(self.closed_deg-self.open_deg)*raw/1000

class Sequencer:
    def __init__(self):
        self.state='IDLE'; self.active=False; self.target=None
        self.current_limit=0.; self.settings=None; self.history=deque()
        self.last_pos=None; self.last_time=None; self.error=''; self.completion=None

    def start(self, raw, settings, fb, now):
        settings.validate(); self.check_feedback(fb,settings)
        pos=float(fb['position']); lo=min(settings.open_deg,settings.closed_deg); hi=max(settings.open_deg,settings.closed_deg)
        if not lo-settings.tolerance_deg<=pos<=hi+settings.tolerance_deg:
            raise ValueError('Position is outside calibrated travel')
        self.settings=settings; self.target=settings.position(raw); self.raw=raw
        self.direction=1 if settings.closed_deg>settings.open_deg else -1
        self.closing=(self.target-pos)*self.direction > settings.tolerance_deg
        self.start_pos=pos; self.started=now; self.contact_count=0
        self.settled_since=None; self.history.clear(); self.error=''; self.completion=None
        self.last_pos=pos; self.last_time=now; self.current_limit=settings.close_a
        self.active=True; self.state='MOVING'

    @staticmethod
    def check_feedback(fb, settings):
        for key in ('position','current_a','temperature_c'):
            if not math.isfinite(float(fb[key])): raise RuntimeError('Non-finite motor feedback')
        if int(str(fb['error']).split()[0]) != 0: raise RuntimeError('Motor fault: '+str(fb['error']))
        if fb['temperature_c']>=settings.max_temp_c: raise RuntimeError('Motor temperature limit reached')
        # Allow quantization/transient margin; this is supervisory, not a hardware current limiter.
        if abs(fb['current_a'])>settings.close_a+0.3: raise RuntimeError('Current exceeds application threshold')

    def stop(self, reason='Stopped; command stream ended'):
        self.active=False; self.state='STOPPED'; self.error=reason

    def fault(self, reason):
        self.stop(reason); self.state='FAULT'; self.completion='GRIPPER_ABORTED'

    def update(self,fb,now):
        if not self.active: return
        s=self.settings
        try:
            self.check_feedback(fb,s)
            lo=min(s.open_deg,s.closed_deg); hi=max(s.open_deg,s.closed_deg)
            if not lo-s.tolerance_deg<=fb['position']<=hi+s.tolerance_deg:
                raise RuntimeError('Motor passed calibrated travel limit')
        except Exception as e:
            self.fault(str(e)); return
        pos=fb['position']; elapsed=max(1e-6,now-self.last_time)
        speed=abs(pos-self.last_pos)/elapsed
        self.last_pos=pos; self.last_time=now
        self.history.append((now,pos))
        while self.history and now-self.history[0][0]>.30: self.history.popleft()
        if self.state=='MOVING':
            stalled=len(self.history)>=2 and now-self.history[0][0]>=.21 and abs(pos-self.history[0][1])<=.15
            candidate=(self.closing and stalled and abs(fb['current_a'])>=s.contact_a
                and abs(pos-self.start_pos)>=.5 and now-self.started>=.35
                and abs(self.target-pos)>s.tolerance_deg)
            self.contact_count=self.contact_count+1 if candidate else 0
            if self.contact_count>=3:
                # Never advance beyond the requested target or calibrated endpoints.
                proposed=pos+self.direction*s.preload_deg
                self.target=min(self.target,proposed) if self.direction>0 else max(self.target,proposed)
                self.current_limit=s.hold_a; self.state='SETTLING_HOLD'; self.settled_since=None
                self.hold_started=now
                return
            good=abs(self.target-pos)<=s.tolerance_deg and speed<=1.0
            self.settled_since=(self.settled_since or now) if good else None
            if self.settled_since is not None and now-self.settled_since>=.2:
                self.state='AT_POSITION'; self.completion='GRIPPER_COMPLETE'
            elif now-self.started>s.move_timeout_s:
                self.fault('Motion timeout')
        elif self.state=='SETTLING_HOLD':
            # Evaluate feedback AFTER reduced-current commands have been sent.
            good=speed<=1.0 and abs(fb['current_a'])<=s.hold_a+.3
            self.settled_since=(self.settled_since or now) if good else None
            if self.settled_since is not None and now-self.settled_since>=.3:
                self.state='HOLDING'; self.completion='GRIPPER_COMPLETE'
            elif now-self.hold_started>3:
                self.fault('Hold did not settle')
        # HOLDING / AT_POSITION keep heartbeat until next move, STOP or disconnect.

    def take_completion(self):
        value=self.completion; self.completion=None; return value

def parse_vr5(text):
    if text=='GRIPPER_OPEN': return 0
    prefix='GRIPPER_CLOSE:'
    if text.startswith(prefix):
        payload=text[len(prefix):]
        if payload.isascii() and payload.isdigit() and 0<=int(payload)<=1000: return int(payload)
    raise ValueError('Expected GRIPPER_OPEN or GRIPPER_CLOSE:<0..1000>')


# mechanics.py
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Geometry:
    max_mm: float = 80.
    r_mm: float = 0.
    l_mm: float = 0.
    e_mm: float = 0.
    doffset_mm: float = 0.

    def check_inputs(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError('Thông số phải là số hữu hạn.')
        if self.max_mm <= 0 or self.r_mm <= 0 or self.l_mm <= 0:
            raise ValueError('Nhập độ mở tối đa, r và l lớn hơn 0. e và Doffset có thể bằng 0 hoặc âm.')

    def x(self, degrees):
        t=math.radians(degrees)
        rad=self.l_mm**2-(self.e_mm-self.r_mm*math.sin(t))**2
        if rad <= 1e-12: raise ValueError('Cơ cấu ngoài miền hình học hoặc tại điểm kỳ dị.')
        return self.r_mm*math.cos(t)+math.sqrt(rad)

    def roots(self, width):
        """Analytic inverse, filtered to the positive square-root assembly branch."""
        x=(width-self.doffset_mm)/2
        radius=math.hypot(x,self.e_mm)
        if radius<1e-12: return []
        ratio=(x*x+self.e_mm**2+self.r_mm**2-self.l_mm**2)/(2*self.r_mm*radius)
        if abs(ratio)>1+1e-10: return []
        alpha=math.acos(max(-1.,min(1.,ratio)))
        phi=math.atan2(self.e_mm,x)
        roots=[]
        for t in (phi+alpha,phi-alpha):
            deg=math.degrees(t)%360
            try: residual=abs(self.doffset_mm+2*self.x(deg)-width)
            except ValueError: continue
            if residual<1e-6 and all(abs((deg-old+180)%360-180)>1e-6 for old in roots): roots.append(deg)
        return sorted(roots)

    def solve(self):
        self.check_inputs()
        opens=self.roots(self.max_mm); closes=self.roots(0.)
        if not opens or not closes:
            missing='OPEN' if not opens else 'CLOSED = 0 mm'
            raise ValueError(f'Không có nghiệm {missing} với bộ số này. D = Doffset + 2x; kiểm tra Doffset/kích thước/độ mở. Các ô nhập vẫn được giữ.')
        solutions=[]
        for a in opens:
            for b in closes:
                for winding in (-1,0,1):
                    end=b+360*winding
                    if not 1e-6<abs(end-a)<=360: continue
                    model=SolvedGeometry(self,a,end)
                    try: model.check_path()
                    except ValueError: continue
                    solutions.append(model)
        if not solutions: raise ValueError('Không có hành trình OPEN → CLOSED giảm đơn điệu và hợp lệ. Các ô nhập vẫn được giữ.')
        # Prefer increasing crank angle; expose alternatives instead of hiding ambiguity.
        return sorted(solutions,key=lambda s:(s.crank_closed_deg<s.crank_open_deg,abs(s.crank_closed_deg-s.crank_open_deg),s.crank_open_deg))

@dataclass(frozen=True)
class SolvedGeometry:
    inputs: Geometry
    crank_open_deg: float
    crank_closed_deg: float

    @property
    def max_mm(self): return self.inputs.max_mm

    def at_fraction(self,fraction):
        if not math.isfinite(fraction) or not 0<=fraction<=1:
            raise ValueError('Góc nằm ngoài khoảng hiệu chuẩn OPEN/CLOSED.')
        theta=self.crank_open_deg+fraction*(self.crank_closed_deg-self.crank_open_deg)
        width=self.inputs.doffset_mm+2*self.inputs.x(theta)
        return 0. if abs(width)<1e-8 else width

    def check_path(self):
        lo,hi=sorted(map(math.radians,(self.crank_open_deg,self.crank_closed_deg)))
        extrema=[lo,hi]+[math.pi/2+k*math.pi for k in range(math.ceil((lo-math.pi/2)/math.pi),math.floor((hi-math.pi/2)/math.pi)+1)]
        g=self.inputs
        if any(abs(g.e_mm-g.r_mm*math.sin(t))>=g.l_mm for t in extrema): raise ValueError('Miền căn không hợp lệ.')
        # Include exact stationary points to avoid missing a short reversal near an endpoint.
        knots=[i/720 for i in range(721)]
        for length in (g.r_mm+g.l_mm,g.r_mm-g.l_mm):
            if abs(length)<1e-12 or abs(g.e_mm/length)>1: continue
            a=math.asin(g.e_mm/length)
            for root in (a,math.pi-a):
                for k in range(-3,4):
                    t=root+2*k*math.pi
                    if lo<t<hi:
                        knots.append((math.degrees(t)-self.crank_open_deg)/(self.crank_closed_deg-self.crank_open_deg))
        widths=[self.at_fraction(s) for s in sorted(knots)]
        if any(b>a+1e-7 for a,b in zip(widths,widths[1:])): raise ValueError('Độ mở không đơn điệu.')

    def width(self,position,encoder_open,encoder_closed):
        if not all(math.isfinite(v) for v in (position,encoder_open,encoder_closed)) or abs(encoder_closed-encoder_open)<1e-6:
            raise ValueError('Chưa hiệu chuẩn góc encoder OPEN/CLOSED.')
        return self.at_fraction((position-encoder_open)/(encoder_closed-encoder_open))


def read_config(data):
    """Return inputs and preferred branch angles; migrate previous valid config."""
    if 'doffset_mm' in data:
        inputs=Geometry(**{key:data[key] for key in Geometry.__dataclass_fields__})
        preferred=data.get('selected_angles')
    else:
        if data.get('closed_mm',0)!=0: raise ValueError('Cấu hình cũ có độ mở đóng khác 0 mm; cần chuyển đổi thủ công. File cũ vẫn giữ nguyên.')
        base=Geometry(max_mm=data['max_mm'],r_mm=data['r_mm'],l_mm=data['l_mm'],e_mm=data['e_mm'])
        offset=-2*base.x(data['crank_closed_deg'])
        inputs=Geometry(base.max_mm,base.r_mm,base.l_mm,base.e_mm,offset)
        preferred=[data['crank_open_deg'],data['crank_closed_deg']]
    if not all(math.isfinite(v) for v in inputs.__dict__.values()): raise ValueError('Cấu hình chứa số không hữu hạn.')
    if preferred is not None and (not isinstance(preferred,list) or len(preferred)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in preferred)):
        raise ValueError('Nhánh cơ cấu đã lưu không hợp lệ.')
    return inputs,preferred


# ports.py
import os, sys, platform, subprocess
from pathlib import Path
from dataclasses import dataclass, field
from serial.tools import list_ports

@dataclass
class Port:
    device: str
    description: str = 'Serial device'
    vid: int | None = None
    pid: int | None = None
    hwid: str = ''
    aliases: list = field(default_factory=list)
    @property
    def usb(self):
        return self.vid is not None or self.device.startswith(('/dev/ttyUSB','/dev/ttyACM'))
    @property
    def label(self):
        chip=' [CH340/CH341 candidate]' if (self.vid,self.pid)==(0x1a86,0x7523) else ''
        access=''
        if os.name=='posix' and Path(self.device).exists() and not os.access(self.device,os.R_OK|os.W_OK):
            access=' [no read/write permission]'
        return f'{self.device} | {self.description}{chip}{access}'

def discover(show_all=False, dev_root=None):
    root=Path(dev_root or '/dev'); found={}; errors=[]
    try:
        for info in list_ports.comports():
            device=info.device
            if not show_all and device.startswith('/dev/ttyS'): continue
            found[os.path.realpath(device)]=Port(device,info.description or 'Serial device',info.vid,info.pid,info.hwid)
    except Exception as e: errors.append(f'pyserial enumeration: {e}')
    if sys.platform.startswith('linux') or dev_root is not None:
        # Fallback when pyserial omits nodes, including adapters without USB serial number.
        for pattern in ('ttyUSB*','ttyACM*','serial/by-id/*','serial/by-path/*'):
            for path in sorted(root.glob(pattern)):
                if not path.exists(): continue
                canonical=os.path.realpath(path)
                entry=found.setdefault(canonical,Port(canonical,'USB serial (device-node fallback)'))
                if str(path)!=canonical and str(path) not in entry.aliases: entry.aliases.append(str(path))
    return sorted(found.values(),key=lambda p:(not p.usb,p.device)), errors

def run_readonly(args):
    try:
        r=subprocess.run(args,capture_output=True,text=True,timeout=3,check=False)
        return (r.stdout+r.stderr).strip() or f'exit={r.returncode}; no output'
    except (OSError,subprocess.TimeoutExpired) as e: return str(e)

def diagnose(selected=''):
    ports,errors=discover(show_all=True)
    lines=['EC Gripper USB diagnostics (read-only; no serial port opened)',
           f'OS: {platform.platform()}',f'Python: {sys.executable}',f'Selected: {selected or "none"}',
           'Listed does not guarantee the port is idle or that CAN motor communication works.']
    if errors: lines.extend(errors)
    for p in ports:
        lines.extend([p.label,f'  HWID: {p.hwid}; aliases: {", ".join(p.aliases) or "none"}'])
    usb=[p for p in ports if p.usb]
    if not usb and sys.platform.startswith('linux'):
        lines.append('No ttyUSB/ttyACM USB serial endpoint found. Qt cannot create a missing kernel device.')
    if sys.platform.startswith('linux'):
        try:
            import grp
            groups=', '.join(grp.getgrgid(g).gr_name for g in os.getgroups())
        except (KeyError,OSError): groups='unknown'
        lines.append('Current process groups: '+groups)
        if selected:
            lines.append(f'Exists: {Path(selected).exists()}; read/write access: {os.access(selected,os.R_OK|os.W_OK)}')
            # fuser output is observational. An empty result does not guarantee availability.
            if selected.startswith('/dev/') and Path(selected).exists():
                lines.extend(['Processes visible through fuser:',run_readonly(['fuser','-v',selected])])
        lines.extend(['USB inventory:',run_readonly(['lsusb'])])
        log=run_readonly(['dmesg','--color=never'])
        relevant=[line for line in log.splitlines() if any(s in line.lower() for s in ('ch341','ch340','ttyusb','ttyacm','brltty','permission','permitted','denied'))]
        lines.extend(['Recent relevant kernel messages (may include older unplug events):', '\n'.join(relevant[-35:]) or 'No relevant messages readable.'])
        lines.extend(['If dmesg is restricted, run sudo dmesg -w, replug the adapter, then copy only the new lines.',
                      'If ch341 attaches then brltty disconnects it, resolve that USB conflict; replug and Refresh ports.',
                      'If permission is missing: sudo usermod -aG dialout "$USER", then log out/in and restart the app.',
                      'A missing /dev/serial/by-id folder alone does not mean missing driver; some adapters have no serial number.'])
    return '\n'.join(lines)


# simulation.py
import struct, time

class SimCan:
    def __init__(self):
        self.position=0.; self.target=0.; self.speed=1.; self.limit=.5
        self.current=0.; self.last=time.monotonic(); self.object_present=True
        self.open_deg=0.; self.closed_deg=90.; self.closed=False
    def configure_1m_standard_normal(self): pass
    def close(self): self.closed=True
    def integrate(self):
        now=time.monotonic();dt=min(.1,now-self.last);self.last=now
        delta=self.target-self.position; step=min(abs(delta),self.speed*6*dt)
        proposed=self.position+(step if delta>=0 else -step)
        sign=1 if self.closed_deg>self.open_deg else -1
        barrier=self.open_deg+.55*(self.closed_deg-self.open_deg)
        contact=self.object_present and delta*sign>0 and (proposed-barrier)*sign>=0
        if contact:
            self.position=barrier;self.current=self.limit*sign
        else:
            self.position=proposed;self.current=(min(.1,self.limit)*(1 if delta>0 else -1)) if abs(delta)>.01 else 0.
    def send_can(self,mid,payload):
        self.integrate();raw=int.from_bytes(payload,'big')
        self.target=struct.unpack('>f',((raw>>29)&0xffffffff).to_bytes(4,'big'))[0]
        self.speed=((raw>>14)&0x7fff)/10;self.limit=((raw>>2)&0xfff)/10
    def recv_can(self,timeout):
        self.integrate()
        return type('Frame',(),{'can_id':1})()

class SimMotor:
    pack_servo_position=staticmethod(EC_A4310.pack_servo_position)
    def __init__(self,can,motor_id=1): self.can=can;self.motor_id=1
    def discover_id(self):return 1
    def set_qna_mode(self):return True
    def query_position(self,timeout=.5):return self.can.position
    def parse_feedback_type2(self,frame):
        return dict(position=self.can.position,current_a=self.can.current,temperature_c=25.,error='0 (SIMULATED)')


# backend.py
import threading, queue, socket, time, math

class Backend(threading.Thread):
    def __init__(self, events):
        super().__init__(daemon=False); self.events=events; self.commands=queue.Queue()
        self.stop_requested=threading.Event(); self.quit_requested=threading.Event()
        self.can=None; self.motor=None; self.udp=None; self.fb=None; self.fb_time=0
        self.seq=Sequencer(); self.pending=False; self.armed=False; self.settings=Settings()
        self.last_cycle=time.monotonic(); self.last_send_time=time.monotonic(); self.power_ready_at=0; self.mode="real"
    def request(self,kind,**kwargs): self.commands.put((kind,kwargs))
    def emit(self,kind,**kwargs): self.events.put((kind,kwargs))
    def reply(self,msg):
        if self.udp: self.udp.sendto(msg.encode('ascii'),('127.0.0.1',5051))
        self.emit('log',message='TX '+msg)
    def abort(self,reason):
        self.seq.stop(reason)
        if self.pending: self.reply('GRIPPER_ABORTED')
        self.pending=False; self.emit('log',message=reason+'; no more servo frames (heartbeat stop ~500 ms per existing driver)')
    def disconnect(self):
        self.abort('Disconnect'); self.armed=False
        if self.can: self.can.close()
        self.can=self.motor=self.fb=None
        if self.udp: self.udp.close(); self.udp=None
        self.emit('connection',connected=False)
    def connect(self,port,motor_id,mode="real"):
        if self.can: raise RuntimeError('Disconnect first')
        self.mode=mode
        MotorClass=EC_A4310
        if mode=="simulation":
            self.can=SimCan(); MotorClass=SimMotor
        else:
            self.can=WaveshareUsbCanA(port,timeout=.005)
        try:
            self.can.configure_1m_standard_normal(); self.motor=MotorClass(self.can,motor_id)
            if self.motor.motor_id is None: self.motor.discover_id()
            ok=self.motor.set_qna_mode()
            if not ok: self.emit('log',message='Q&A mode has no ACK; validating position query as in original driver')
            pos=self.motor.query_position()
            if not math.isfinite(pos): raise RuntimeError('Invalid position')
            self.fb={'position':pos,'current_a':None,'temperature_c':None,'error':'0 (OK)'}
            self.fb_time=time.monotonic(); self.power_ready_at=time.monotonic()+(0 if self.mode=="simulation" else 5.2)
            self.emit('connection',connected=True,motor_id=self.motor.motor_id)
            self.emit('log',message='Simulation connected. Demo calibration 0–90 deg.' if self.mode=='simulation' else 'Connected without motion. Wait 5.2 s before movement.')
        except Exception:
            self.can.close(); self.can=self.motor=self.fb=None; raise
    def move(self,raw,remote=False):
        if self.pending: raise RuntimeError('Busy: previous VR5 operation is pending')
        if not self.can or not self.armed: raise RuntimeError('Connect, calibrate and Arm first')
        if time.monotonic()<self.power_ready_at: raise RuntimeError('Motor startup wait not finished')
        # Read fresh position; actual current/temp validated on first type-2 response.
        pos=self.motor.query_position(timeout=.25)
        initial={'position':pos,'current_a':0.,'temperature_c':0.,'error':'0 (OK)'}
        self.seq.start(raw,self.settings,initial,time.monotonic()); self.pending=remote
        self.fb_time=time.monotonic(); self.last_cycle=time.monotonic(); self.last_send_time=time.monotonic()
        self.emit('log',message=f'MOVE {raw} → {self.seq.target:.3f} deg; current threshold {self.seq.current_limit:.1f} A')
    def receive_feedback(self,timeout=.035):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not self.stop_requested.is_set():
            fr=self.can.recv_can(timeout=min(.01,max(.001,deadline-time.monotonic())))
            if fr and fr.can_id==self.motor.motor_id:
                fb=self.motor.parse_feedback_type2(fr)
                if fb: return fb
        return None
    def handle(self,kind,args):
        if kind=='connect': self.connect(**args)
        elif kind=='disconnect': self.disconnect()
        elif kind=='settings':
            if self.seq.active: raise RuntimeError('STOP before changing calibration/settings')
            self.settings=args['settings']; self.armed=False
            if self.mode=='simulation' and self.can:
                self.can.open_deg=self.settings.open_deg; self.can.closed_deg=self.settings.closed_deg
        elif kind=='sim_object':
            if self.mode=='simulation' and self.can: self.can.object_present=args['present']
        elif kind=='arm':
            if not args['enabled']:
                self.armed=False; self.abort('Disarmed')
            else:
                if not self.can: raise RuntimeError('Not connected')
                self.settings.validate(); self.armed=True
        elif kind=='move': self.move(args['raw'])
        elif kind=='udp':
            if args['enabled'] and not self.udp:
                sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                try: sock.bind(('127.0.0.1',5050)); sock.setblocking(False)
                except Exception: sock.close(); raise
                self.udp=sock; self.emit('log',message='UDP listening :5050, feedback :5051')
            elif not args['enabled'] and self.udp:
                self.abort('UDP disabled'); self.udp.close(); self.udp=None
        else: raise ValueError('Unknown local request')
    def run(self):
        try:
            while not self.quit_requested.is_set():
                cycle=time.monotonic()
                if self.stop_requested.is_set():
                    self.armed=False; self.abort('STOP requested'); self.stop_requested.clear()
                    # Flush queued actions so an old MOVE cannot restart after STOP.
                    while True:
                        try: self.commands.get_nowait()
                        except queue.Empty: break
                for _ in range(10):
                    if self.stop_requested.is_set() or self.quit_requested.is_set(): break
                    try: kind,args=self.commands.get_nowait()
                    except queue.Empty: break
                    try: self.handle(kind,args)
                    except Exception as e:
                        self.emit('log',message=str(e))
                        if kind=='connect': self.emit('connection',connected=False)
                if self.udp:
                    for _ in range(10):
                        if self.stop_requested.is_set(): break
                        try: packet,_=self.udp.recvfrom(256)
                        except BlockingIOError: break
                        text=packet.decode('ascii',errors='replace').strip(); self.emit('log',message='RX '+text)
                        try: self.move(parse_vr5(text),remote=True)
                        except Exception as e:
                            self.emit('log',message=str(e)); self.reply('GRIPPER_ABORTED')
                if self.seq.active and not self.stop_requested.is_set():
                    try:
                        if time.monotonic()-self.last_send_time>.25: raise RuntimeError('Servo command interval exceeded 250 ms')
                        payload=self.motor.pack_servo_position(self.seq.target,self.settings.speed_rpm,self.seq.current_limit,ack=2)
                        self.can.send_can(self.motor.motor_id,payload)
                        self.last_send_time=time.monotonic()
                        fb=self.receive_feedback()
                        now=time.monotonic()
                        if fb:
                            self.fb=fb; self.fb_time=now; self.seq.update(fb,now)
                        if now-self.fb_time>.25: raise RuntimeError('Feedback lost >250 ms')
                        outcome=self.seq.take_completion()
                        if outcome:
                            self.emit('log',message=outcome+' / '+self.seq.state)
                            if self.pending: self.reply(outcome); self.pending=False
                            if outcome=='GRIPPER_ABORTED': self.armed=False
                    except Exception as e:
                        self.seq.fault(str(e)); self.armed=False
                        if self.pending: self.reply('GRIPPER_ABORTED'); self.pending=False
                        self.emit('log',message='FAULT: '+str(e))
                elif self.can and not self.stop_requested.is_set() and cycle-self.fb_time>.3:
                    try:
                        pos=self.motor.query_position(timeout=.15)
                        if not math.isfinite(pos): raise RuntimeError('Invalid position feedback')
                        self.fb={'position':pos,'current_a':None,'temperature_c':None,'error':'0 (OK)'}; self.fb_time=time.monotonic()
                    except Exception as e:
                        self.armed=False; self.seq.state='FAULT'; self.emit('log',message=str(e)); self.disconnect()
                self.last_cycle=time.monotonic()
                self.emit('status',state=self.seq.state,armed=self.armed,connected=self.can is not None,
                    active=self.seq.active,pending=self.pending,udp=self.udp is not None,fb=self.fb,mode=self.mode,
                    feedback_age=time.monotonic()-self.fb_time,error=self.seq.error,target=self.seq.target,
                    limit=self.seq.current_limit)
                self.quit_requested.wait(max(0,.05-(time.monotonic()-cycle)))
        except Exception as e:
            self.emit('log',message='Backend stopped: '+str(e))
        finally:
            self.disconnect()
            if self.udp: self.udp.close(); self.udp=None
            self.emit('finished')


# visual.py
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QLinearGradient
from PySide6.QtCore import QRectF, Qt

class JawView(QWidget):
    def __init__(self):
        super().__init__(); self.setMinimumSize(320,300)
        self.position=None; self.target=None; self.open_deg=0.; self.closed_deg=90.
        self.mode='simulation'; self.object_present=True; self.state='DISCONNECTED'; self.stale=True
        self.width_mm=None; self.max_mm=None; self.object_width_mm=None

    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor('#f8fbfe'))
        p.setPen(QPen(QColor('#e4ebf2'),1))
        for x in range(0,self.width(),28): p.drawLine(x,0,x,self.height())
        for y in range(0,self.height(),28): p.drawLine(0,y,self.width(),y)
        p.setFont(QFont('Sans',10)); p.setPen(QColor('#52677c'))
        p.drawText(18,26,'MÔ PHỎNG' if self.mode=='simulation' else 'PHẢN HỒI ENCODER')
        p.drawText(18,47,'Sơ đồ 2D · độ mở theo mô hình cơ khí (mm)' if self.width_mm is not None else 'Sơ đồ 2D · chưa cấu hình mô hình độ mở mm')
        span=self.closed_deg-self.open_deg
        valid=self.position is not None and abs(span)>1e-6
        closure=max(0.,min(1.,(self.position-self.open_deg)/span)) if valid else 0.
        p.save()
        scale=min((self.width()-40)/600,(self.height()-95)/340)
        p.translate(self.width()/2,58+(self.height()-95-340*scale)/2); p.scale(scale,scale)
        def box(x,y,w,h,top,bottom,r=6):
            gradient=QLinearGradient(x,y,x,y+h); gradient.setColorAt(0,QColor(top)); gradient.setColorAt(1,QColor(bottom))
            p.setPen(QPen(QColor('#526a7d'),1)); p.setBrush(gradient); p.drawRoundedRect(QRectF(x,y,w,h),r,r)
        # Cast shadow, motor housing, guide and independent carriages.
        p.setPen(Qt.NoPen); p.setBrush(QColor(0,0,0,55)); p.drawEllipse(QRectF(-230,298,460,25))
        box(-66,0,132,65,'#536b81','#263a4c',12)
        box(-195,45,390,96,'#adc0cf','#536b80',10)
        box(-172,102,344,21,'#172837','#344c60',4)
        for x in (-176,176):
            for y in (62,85):
                p.setBrush(QColor('#182b3b')); p.setPen(QPen(QColor('#c0d0dc'),1)); p.drawEllipse(QRectF(x-4,y-4,8,8))
        p.setPen(QColor('#dce9f2')); p.setFont(QFont('Sans',12,QFont.Bold)); p.drawText(QRectF(-100,60,200,28),Qt.AlignCenter,'EC · A4310')
        gap=260*(1-closure) if self.width_mm is None else 260*max(0.,min(1.,self.width_mm/self.max_mm))
        # Pads meet at zero; at synthetic contact (55%), gap equals object width.
        for side in (-1,1):
            inner=side*gap/2
            x=inner-38 if side<0 else inner
            box(x-8,112,54,39,'#5be0c6','#248477',5)
            box(x,151,38,125,'#b8cad8','#526e84',5)
            pad_x=inner-10 if side<0 else inner
            box(pad_x,202,10,74,'#344d60','#172b3b',2)
            p.setPen(QPen(QColor('#7394a8'),1))
            for y in range(208,273,8): p.drawLine(int(pad_x+2),y,int(pad_x+8),y)
        if self.mode=='simulation' and self.object_present:
            object_width=117 if self.width_mm is None else 260*self.object_width_mm/self.max_mm
            box(-object_width/2,205,object_width,71,'#e9ba70','#ac733b',9)
            p.setPen(QColor('#513e27')); p.setFont(QFont('Sans',10)); p.drawText(QRectF(-58,225,116,25),Qt.AlignCenter,'VẬT MÔ PHỎNG')
        p.setPen(QPen(QColor('#087f72'),1.5))
        p.drawLine(int(-gap/2),294,int(gap/2),294)
        for x in (-gap/2,gap/2): p.drawLine(int(x),286,int(x),302)
        p.restore()
        p.setFont(QFont('Sans',10)); p.setPen(QColor('#087f72') if valid and not self.stale else QColor('#87590d'))
        caption=f'Mức đóng {closure*100:.0f}%  ·  Góc {self.position:.2f}°' if valid else 'Chờ phản hồi và hiệu chuẩn góc mở / đóng'
        if self.width_mm is not None: caption=f'Độ mở {self.width_mm:.2f} mm  ·  '+caption
        if valid and self.stale: caption+='  ·  Phản hồi cũ'
        p.drawText(18,self.height()-17,caption)
        p.end()

SETTINGS_PATH=Path(__file__).resolve().with_name('gripper_settings.json')
DEFAULT_SETTINGS={
    'version':1,
    'hardware':asdict(Settings()),
    'simulation':asdict(Settings(open_deg=0,closed_deg=90,speed_rpm=2)),
    'mechanics':dict(version=2,max_mm=80,r_mm=0,l_mm=0,e_mm=0,doffset_mm=0,selected_angles=None),
    'connection':dict(port='',motor_id=0,show_all_ports=False),
    'ui':dict(target_position=700),
}

def read_settings_file():
    result=copy.deepcopy(DEFAULT_SETTINGS)
    if not SETTINGS_PATH.exists(): return result
    data=json.loads(SETTINGS_PATH.read_text(encoding='utf-8'))
    if not isinstance(data,dict) or data.get('version')!=1:
        raise ValueError('Unsupported gripper_settings.json format; file has not been overwritten.')
    for key,value in data.items():
        if key in result and isinstance(result[key],dict):
            if not isinstance(value,dict): raise ValueError('Invalid settings section: '+key)
            result[key].update(value)
        else: result[key]=value
    return result

def update_settings_file(**sections):
    data=read_settings_file()  # Never overwrite malformed JSON with defaults.
    data.update(sections)
    temporary=SETTINGS_PATH.with_suffix('.json.tmp')
    try:
        with temporary.open('w',encoding='utf-8') as handle:
            handle.write(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
            handle.flush(); os.fsync(handle.fileno())
        temporary.replace(SETTINGS_PATH)
    finally:
        if temporary.exists(): temporary.unlink()

def import_legacy_settings(directory):
    directory=Path(directory); sections={}
    for section,filename in [('hardware','hardware_settings.json'),('mechanics','mechanical_settings.json')]:
        path=directory/filename
        if path.exists():
            data=json.loads(path.read_text(encoding='utf-8'))
            if section=='hardware': Settings(**data).validate()
            else: read_config(data)
            sections[section]=data
    if not sections: raise ValueError('No hardware_settings.json or mechanical_settings.json found in '+str(directory))
    update_settings_file(**sections)
    return ', '.join(sections)

# Qt GUI
import sys, json, queue, time, threading
from pathlib import Path
from dataclasses import asdict
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QColor, QPen
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,
 QFormLayout,QDoubleSpinBox,QSpinBox,QPushButton,QLabel,QComboBox,QCheckBox,QPlainTextEdit,QGroupBox,QDialog,QTabWidget,QScrollArea,QSlider,QSizePolicy)
from collections import deque



class Trace(QWidget):
    def __init__(self):
        super().__init__(); self.points=deque(maxlen=500); self.setMinimumHeight(140)
    def paintEvent(self,event):
        p=QPainter(self); p.fillRect(self.rect(),QColor('#ffffff'))
        p.setPen(QColor('#334155')); p.drawText(15,22,'Actual current (A), −3 … +3; last 25 s nominal')
        p.setPen(QColor('#cbd5e1')); mid=(self.height()+30)/2
        p.drawLine(15,int(mid),self.width()-15,int(mid))
        p.setPen(QPen(QColor('#087f72'),2)); data=list(self.points)
        for i in range(1,len(data)):
            if data[i-1] is None or data[i] is None: continue
            y1=mid-data[i-1]/3*(self.height()-50)/2; y2=mid-data[i]/3*(self.height()-50)/2
            p.drawLine(int(15+(i-1)*(self.width()-30)/500),int(y1),int(15+i*(self.width()-30)/500),int(y2))

class Window(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('EC Gripper Qt • Simulation / Real motor'); self.resize(1200,720)
        self.events=queue.Queue(); self.backend=Backend(self.events); self.backend.start()
        self._ports_initialized=False; self._connecting=False; self._port_signature=None; self._diagnosing=False
        self.fb=None; self.ready=False; self.active=False; self.pending=False
        self.setStyleSheet('QGroupBox {font-weight:bold;} QPushButton {padding:7px;}')
        root=QWidget(); self.setCentralWidget(root); layout=QVBoxLayout(root)
        layout.addWidget(QLabel('EC-A4310-P2-36 + Waveshare USB-CAN-A | servo position + current threshold'))
        mode_row=QHBoxLayout(); layout.addLayout(mode_row)
        self.mode=QComboBox(); self.mode.addItem('Real EC-A4310 — USB-CAN','real'); self.mode.addItem('Simulation — no USB required','simulation'); mode_row.addWidget(self.mode)
        self.allports=QCheckBox('Show ttyS / all ports'); self.allports.toggled.connect(lambda _:self.refresh_ports()); mode_row.addWidget(self.allports)
        self.jaws=JawView(); self.jaws.setMinimumHeight(180); self.jaws.object_present=False; layout.addWidget(self.jaws)
        row=QHBoxLayout(); layout.addLayout(row)
        self.port=QComboBox(); self.port.setEditable(True); row.addWidget(self.port)
        self.refresh=QPushButton('Refresh ports'); self.refresh.clicked.connect(self.refresh_ports); row.addWidget(self.refresh)
        self.mid=QSpinBox(); self.mid.setRange(0,2046); self.mid.setValue(0); self.mid.setSpecialValueText('Auto ID'); row.addWidget(self.mid)
        self.connect_btn=QPushButton('Connect'); self.connect_btn.clicked.connect(self.connect_motor); row.addWidget(self.connect_btn)
        self.disconnect_btn=QPushButton('Disconnect'); self.disconnect_btn.clicked.connect(lambda:self.backend.request('disconnect')); row.addWidget(self.disconnect_btn)
        self.diagnostic_btn=QPushButton('Diagnose USB'); self.diagnostic_btn.clicked.connect(self.diagnose_usb); row.addWidget(self.diagnostic_btn)
        self.port_hint=QLabel(); self.port_hint.setWordWrap(True); layout.addWidget(self.port_hint)
        self.refresh_ports()
        groups=QHBoxLayout(); layout.addLayout(groups)
        calibration=QGroupBox('Calibration — measured motor angles'); groups.addWidget(calibration)
        c=QFormLayout(calibration)
        self.open=self.spin(c,'Open angle (deg)',-100000,100000,0,3)
        self.closed=self.spin(c,'Closed angle (deg)',-100000,100000,0,3)
        self.open_set=QPushButton('Use current angle as OPEN'); self.open_set.clicked.connect(lambda:self.capture(self.open)); c.addRow(self.open_set)
        self.close_set=QPushButton('Use current angle as CLOSED'); self.close_set.clicked.connect(lambda:self.capture(self.closed)); c.addRow(self.close_set)
        params=QGroupBox('Current thresholds — not force in N'); groups.addWidget(params); f=QFormLayout(params)
        self.speed=self.spin(f,'Speed (rpm)',.1,5,1,1)
        self.close_a=self.spin(f,'I_close / I_open max (A)',.1,3,.5,1)
        self.hold_a=self.spin(f,'I_hold threshold (A)',.1,3,.3,1)
        self.contact_a=self.spin(f,'Contact detection (A)',.1,3,.4,1)
        self.preload=self.spin(f,'Preload (deg)',0,5,1,2)
        self.timeout=self.spin(f,'Move timeout (s)',2,120,20,1)
        row=QHBoxLayout(); layout.addLayout(row)
        self.save=QPushButton('Save settings'); self.save.clicked.connect(self.save_settings); row.addWidget(self.save)
        self.motion_buttons=[]; row=QHBoxLayout(); layout.addLayout(row)
        self.raw=QSpinBox(); self.raw.setRange(0,1000); self.raw.setValue(700); row.addWidget(self.raw)
        for label,raw in [('Move target',None),('OPEN',0),('CLOSE',1000)]:
            b=QPushButton(label); b.clicked.connect(lambda checked=False,value=raw:self.backend.request('move',raw=self.raw.value() if value is None else value)); row.addWidget(b); b.setEnabled(False); self.motion_buttons.append(b)
        self.stop_btn=QPushButton('STOP — cease command stream'); self.stop_btn.setStyleSheet('background:#bd3030;color:white;font-weight:bold;'); self.stop_btn.clicked.connect(self.stop); row.addWidget(self.stop_btn)
        layout.addWidget(QLabel('STOP relies on existing motor heartbeat timeout (~500 ms); support payload before stopping.'))
        self.state=QLabel('DISCONNECTED'); layout.addWidget(self.state)
        self.feedback=QLabel('Position — | Current — | Temperature —'); layout.addWidget(self.feedback)
        self.trace=Trace(); layout.addWidget(self.trace)
        self.log=QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(600); layout.addWidget(self.log)
        # Rehouse existing controls without changing their signals or motor commands.
        shell=QWidget(); outer=QVBoxLayout(shell); outer.setContentsMargins(12,10,12,10); outer.setSpacing(6)
        title=QLabel('EC GRIPPER'); title.setObjectName('title'); header=QHBoxLayout(); header.addWidget(title); outer.addLayout(header)
        subtitle=QLabel('EC-A4310 · Điều khiển tay kẹp'); subtitle.setObjectName('muted'); header.addWidget(subtitle); header.addStretch()
        self.tabs=QTabWidget(); outer.addWidget(self.tabs)
        control_page=QWidget(); control_layout=QVBoxLayout(control_page)
        control_layout.setContentsMargins(10,8,10,8); control_layout.setSpacing(6)
        status_row=QHBoxLayout(); control_layout.addLayout(status_row)
        self.overview=QLabel('Chưa kết nối'); self.overview.setWordWrap(True); self.overview.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred); status_row.addWidget(self.overview,1)
        settings_link=QPushButton('Cài đặt kết nối →'); settings_link.clicked.connect(lambda:self.tabs.setCurrentIndex(1)); status_row.addWidget(settings_link)
        self.monitor_button=QPushButton('Theo dõi / Nhật ký ↗'); self.monitor_button.clicked.connect(self.show_monitor); status_row.addWidget(self.monitor_button)
        quick_row=QHBoxLayout(); control_layout.addLayout(quick_row)
        self.connect_real=QPushButton('Connect real gripper')
        self.connect_real.clicked.connect(lambda:self.quick_connect('real'))
        self.connect_simulator=QPushButton('Connect simulator')
        self.connect_simulator.clicked.connect(lambda:self.quick_connect('simulation'))
        self.quick_disconnect=QPushButton('Disconnect')
        self.quick_disconnect.clicked.connect(self.disconnect_btn.click)
        for button in (self.connect_real,self.connect_simulator,self.quick_disconnect):
            quick_row.addWidget(button)
        self.quick_disconnect.setEnabled(False)
        control_layout.addWidget(self.jaws,1)
        self.feedback.setWordWrap(True); control_layout.addWidget(self.feedback)
        self.aperture=QLabel('Độ mở: — mm · Chưa cấu hình cơ khí'); control_layout.addWidget(self.aperture)
        position_box=QWidget(); position_layout=QVBoxLayout(position_box); position_layout.setContentsMargins(0,0,0,0)
        position_row=QHBoxLayout(); position_layout.addLayout(position_row)
        position_row.addWidget(QLabel('Vị trí · 0 mở / 1000 đóng'))
        self.position_slider=QSlider(Qt.Horizontal); self.position_slider.setRange(0,1000); self.position_slider.setValue(self.raw.value())
        self.position_slider.setAccessibleName('Vị trí tay kẹp, mức đóng 0 đến 1000')
        self.position_slider.valueChanged.connect(self.raw.setValue); self.raw.valueChanged.connect(self.position_slider.setValue)
        position_row.addWidget(self.position_slider,1); position_row.addWidget(self.raw)
        self.motion_buttons[0].setText('ĐẾN VỊ TRÍ'); position_row.addWidget(self.motion_buttons[0])
        self.raw.setToolTip('Mức đóng theo hành trình góc; 500 không nhất thiết bằng nửa độ mở mm.')
        control_layout.addWidget(position_box)
        actions=QHBoxLayout(); control_layout.addLayout(actions)
        for button,label,name in [(self.motion_buttons[1],'MỞ · OPEN','openButton'),(self.motion_buttons[2],'KẸP · CLOSE','closeButton'),(self.stop_btn,'DỪNG · STOP','stopButton')]:
            button.setText(label); button.setObjectName(name); button.setMinimumHeight(44); actions.addWidget(button)
        self.stop_btn.setStyleSheet('')
        self.action_message=QLabel('Chọn Connect real gripper hoặc Connect simulator để bắt đầu.')
        self.action_message.setWordWrap(False); self.action_message.setFixedHeight(20); self.action_message.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed); control_layout.addWidget(self.action_message)
        self.log.textChanged.connect(self.update_action_message)
        notice=QLabel('STOP ngừng gửi lệnh; động cơ dựa vào timeout khoảng 500 ms. Đỡ vật trước khi dừng. Kết nối lại hoặc Save settings để tiếp tục.')
        notice.setWordWrap(True); notice.setObjectName('muted'); control_layout.addWidget(notice)
        self.tabs.addTab(control_page,'Điều khiển')
        settings_page=QWidget(); settings_layout=QVBoxLayout(settings_page); settings_layout.setSpacing(14)
        connection=QGroupBox('Kết nối'); connection_layout=QFormLayout(connection)
        connection_layout.addRow('Chế độ',self.mode); connection_layout.addRow('Cổng USB-CAN',self.port)
        connection_layout.addRow('Motor ID',self.mid)
        connection_actions=QHBoxLayout()
        for widget in [self.refresh,self.connect_btn,self.disconnect_btn,self.diagnostic_btn]: connection_actions.addWidget(widget)
        connection_layout.addRow(connection_actions); connection_layout.addRow(self.allports); connection_layout.addRow(self.port_hint)
        settings_layout.addWidget(connection)
        settings_layout.addWidget(calibration); settings_layout.addWidget(params)
        settings_layout.addWidget(self.save)
        settings_hint=QLabel('Bấm DỪNG để sửa thông số. Save settings áp dụng thông số và bật lại điều khiển khi đã kết nối.')
        settings_hint.setWordWrap(True); settings_layout.addWidget(settings_hint)
        self.build_mechanics(settings_layout)
        position_layout.addWidget(self.target_aperture)
        self.monitor=QDialog(self,Qt.Window); self.monitor.setWindowTitle('EC Gripper · Theo dõi và nhật ký'); self.monitor.resize(850,650)
        monitor_layout=QVBoxLayout(self.monitor)
        self.state.setWordWrap(True); monitor_layout.addWidget(self.state)
        self.monitor_feedback=QLabel('Chờ phản hồi'); self.monitor_feedback.setWordWrap(True); monitor_layout.addWidget(self.monitor_feedback)
        self.monitor_aperture=QLabel('Độ mở: — mm'); self.monitor_aperture.setWordWrap(True); monitor_layout.addWidget(self.monitor_aperture)
        monitor_layout.addWidget(self.trace,1); monitor_layout.addWidget(self.log,2)
        monitor_stop=QPushButton('DỪNG · STOP'); monitor_stop.setObjectName('stopButton'); monitor_stop.clicked.connect(self.stop); monitor_layout.addWidget(monitor_stop)
        monitor_hint=QLabel('Đóng cửa sổ này chỉ ẩn phần theo dõi; tay kẹp vẫn hoạt động.'); monitor_hint.setWordWrap(True); monitor_layout.addWidget(monitor_hint)
        launch_monitor=QPushButton('Mở cửa sổ Theo dõi và nhật ký ↗'); launch_monitor.clicked.connect(self.show_monitor); settings_layout.addWidget(launch_monitor)
        settings_layout.addStretch()
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(settings_page)
        self.tabs.addTab(scroll,'Cài đặt')
        # STOP is always reachable, even while reviewing settings.
        self.settings_stop=QPushButton('DỪNG · STOP'); self.settings_stop.setObjectName('stopButton'); self.settings_stop.clicked.connect(self.stop)
        self.tabs.setCornerWidget(self.settings_stop)
        old_root=self.takeCentralWidget(); self.setCentralWidget(shell); old_root.deleteLater()
        self.setStyleSheet("""
            QWidget {background:#f4f7fb; color:#24364b; font-size:13px;}
            QLabel#title {font-size:21px; font-weight:700; color:#16324f;}
            QLabel#muted {color:#52677c; font-size:12px;}
            QTabWidget::pane {border:1px solid #ccd7e3; border-radius:8px;}
            QTabBar::tab {padding:8px 24px; background:#e8eef5; color:#52677c;}
            QTabBar::tab:selected {background:#ffffff; color:#087f72; border-bottom:3px solid #087f72;}
            QGroupBox {border:1px solid #ccd7e3; border-radius:8px; margin-top:16px; padding:16px; font-weight:bold;}
            QGroupBox::title {subcontrol-origin:margin; left:12px; padding:0 5px;}
            QPushButton {background:#ffffff; border:1px solid #b8c8d8; border-radius:7px; padding:6px 12px;}
            QPushButton:hover {background:#e3edf8; border-color:#7295b6;}
            QPushButton:pressed {background:#d1e2f2;}
            QPushButton#openButton {background:#216b96; color:white; font-size:17px; font-weight:bold;}
            QPushButton#closeButton {background:#087f72; color:white; font-size:17px; font-weight:bold;}
            QPushButton#stopButton {background:#b93849; color:white; font-weight:bold;}
            QPushButton#openButton:hover {background:#195678;}
            QPushButton#closeButton:hover {background:#06675d;}
            QPushButton#stopButton:hover {background:#9e2939;}
            QPushButton:disabled, QPushButton#openButton:disabled, QPushButton#closeButton:disabled {background:#e5eaf0; color:#68798b; border-color:#d0dae4;}
            QComboBox,QSpinBox,QDoubleSpinBox,QPlainTextEdit {background:#ffffff; color:#24364b; border:1px solid #b8c8d8; border-radius:4px; padding:6px; selection-background-color:#216b96; selection-color:white;}
            QComboBox QAbstractItemView {background:white; color:#24364b; selection-background-color:#d9eaf7; selection-color:#16324f;}
            QComboBox:disabled,QSpinBox:disabled,QDoubleSpinBox:disabled {background:#e9eef4; color:#68798b;}
            QSlider::groove:horizontal {height:6px; background:#cbd7e3; border-radius:3px;}
            QSlider::sub-page:horizontal {background:#216b96; border-radius:3px;}
            QSlider::handle:horizontal {background:#216b96; border:1px solid #185578; width:16px; margin:-5px 0; border-radius:8px;}
            QCheckBox::indicator {width:14px; height:14px; border:1px solid #8498ad; border-radius:3px; background:white;}
            QCheckBox::indicator:checked {background:#087f72; border:2px solid #055b52;}
            QCheckBox::indicator:disabled {background:#dce4ec; border-color:#a6b6c6;}
            QCheckBox::indicator:checked:disabled {background:#8bafaa;}
            QScrollArea {border:none;}
            QToolTip {background:#ffffff; color:#24364b; border:1px solid #b8c8d8;}
        """)
        self.config_controls=[self.open,self.closed,self.open_set,self.close_set,self.speed,self.close_a,self.hold_a,self.contact_a,self.preload,self.timeout,self.save]
        self.config_controls.extend(self.mechanical_controls)
        # Long feedback and geometry messages must wrap rather than widen the window.
        for label in (self.feedback,self.aperture,self.target_aperture):
            label.setWordWrap(True); label.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
        self.load_mechanics()
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.mode_changed()
        self.load_connection_settings()
        self.timer=QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(50)
        self.port_timer=QTimer(self); self.port_timer.timeout.connect(self.auto_refresh_ports); self.port_timer.start(2000)
        available=self.screen().availableGeometry()
        self.resize(min(1200,available.width()-32),min(720,available.height()-48))
        self.move(available.x()+(available.width()-self.width())//2,available.y()+16)

    def update_action_message(self):
        message=self.log.document().lastBlock().text()
        self.action_message.setText(self.action_message.fontMetrics().elidedText(message,Qt.ElideRight,max(100,self.action_message.width())))
        self.action_message.setToolTip(message)

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'action_message'): self.update_action_message()

    def show_monitor(self):
        self.monitor.show(); self.monitor.raise_(); self.monitor.activateWindow()

    def build_mechanics(self,layout):
        self.geometry=None; self.mech_solutions=[]
        group=QGroupBox('Cơ khí · 5 thông số chính'); form=QFormLayout(group)
        hint=QLabel('Nhập độc lập các thông số dưới đây. Chương trình tự tính góc mở/đóng, không sửa các giá trị bạn nhập.')
        hint.setWordWrap(True); form.addRow(hint)
        self.mech_widgets={}
        fields=[('max_mm','Độ mở tối đa (mm)',0,10000,80),('r_mm','Tay quay r (mm)',0,10000,0),('l_mm','Thanh truyền l (mm)',0,10000,0),('e_mm','Lệch tâm e (mm)',-10000,10000,0),('doffset_mm','Doffset — số bù độ mở (mm)',-100000,100000,0)]
        for key,label,lo,hi,value in fields:
            self.mech_widgets[key]=self.spin(form,label,lo,hi,value,4)
            self.mech_widgets[key].setKeyboardTracking(False)
        note=QLabel('D = Doffset + 2x. Doffset là số bù có dấu, có thể âm; không phải độ lệch tâm e. CLOSED được tính tại D = 0 mm. Góc tự tính chỉ dùng hiển thị; góc encoder vẫn hiệu chuẩn riêng.')
        note.setWordWrap(True); form.addRow(note)
        self.mech_branch=QComboBox(); self.mech_branch.currentIndexChanged.connect(self.select_mechanical_branch)
        form.addRow(self.mech_branch)
        self.mech_status=QLabel(); self.mech_status.setWordWrap(True); form.addRow(self.mech_status)
        self.target_aperture=QLabel(); self.target_aperture.setWordWrap(True); form.addRow(self.target_aperture)
        self.mech_save=QPushButton('Lưu thông số cơ khí'); self.mech_save.clicked.connect(self.save_mechanics); form.addRow(self.mech_save)
        self.mechanical_controls=list(self.mech_widgets.values())+[self.mech_branch,self.mech_save]
        for widget in self.mech_widgets.values(): widget.valueChanged.connect(self.geometry_changed)
        self.raw.valueChanged.connect(self.update_target_aperture)
        layout.addWidget(group); self.geometry_changed()

    def geometry_changed(self,*_):
        preferred=[self.geometry.crank_open_deg,self.geometry.crank_closed_deg] if self.geometry else None
        self.geometry=None; self.mech_solutions=[]
        self.mech_branch.blockSignals(True); self.mech_branch.clear()
        try:
            inputs=Geometry(**{key:w.value() for key,w in self.mech_widgets.items()})
            self.mech_solutions=inputs.solve()
            for i,g in enumerate(self.mech_solutions):
                self.mech_branch.addItem(f'Nhánh {i+1}: OPEN {g.crank_open_deg:.3f}° → CLOSED {g.crank_closed_deg:.3f}°')
            self.mech_branch.setCurrentIndex(self.nearest_branch(preferred))
        except ValueError as exc: self.mech_status.setText('Chưa tính được: '+str(exc))
        self.mech_branch.blockSignals(False); self.mech_branch.setVisible(len(self.mech_solutions)>1)
        self.select_mechanical_branch()

    def nearest_branch(self,angles):
        if not angles or not self.mech_solutions: return 0
        def distance(g):
            # Preserve direction as well as the OPEN phase when migrating/reloading.
            return abs((g.crank_open_deg-angles[0]+180)%360-180)+abs((g.crank_closed_deg-g.crank_open_deg)-(angles[1]-angles[0]))
        return min(range(len(self.mech_solutions)),key=lambda i:distance(self.mech_solutions[i]))

    def select_mechanical_branch(self,*_):
        index=self.mech_branch.currentIndex()
        self.geometry=self.mech_solutions[index] if 0<=index<len(self.mech_solutions) else None
        if self.geometry:
            g=self.geometry
            text=f'Tự tính: góc OPEN {g.crank_open_deg:.3f}° · góc CLOSED {g.crank_closed_deg:.3f}° · hành trình mỗi ngón {g.max_mm/2:.2f} mm'
            if len(self.mech_solutions)>1: text+='\nCó nhiều nhánh hình học; chọn nhánh đúng với chiều lắp thực tế ở trên.'
            self.mech_status.setText(text)
        self.update_target_aperture()

    def update_target_aperture(self,*_):
        text=f'Mục tiêu {self.raw.value()}/1000 → {self.geometry.at_fraction(self.raw.value()/1000):.2f} mm (theo mô hình)' if self.geometry else f'Đã chọn {self.raw.value()}/1000 · Độ mở đích: — mm (chưa có mô hình cơ khí)'
        self.target_aperture.setText(text)

    def save_mechanics(self):
        try:
            data={key:w.value() for key,w in self.mech_widgets.items()}
            data['version']=2
            data['selected_angles']=[self.geometry.crank_open_deg,self.geometry.crank_closed_deg] if self.geometry else None
            update_settings_file(mechanics=data)
            self.log.appendPlainText('Đã lưu thông số cơ khí.' if self.geometry else 'Đã lưu bản nháp cơ khí; chưa có nghiệm để hiển thị mm.')
        except (ValueError,OSError) as exc: self.log.appendPlainText('Cơ khí: '+str(exc))

    def load_mechanics(self):
        try:
            g,preferred=read_config(read_settings_file()['mechanics'])
            if any(not w.minimum()<=getattr(g,key)<=w.maximum() for key,w in self.mech_widgets.items()): raise ValueError('Thông số trong file nằm ngoài khoảng nhập hỗ trợ; file vẫn được giữ.')
            for key,w in self.mech_widgets.items():
                w.blockSignals(True); w.setValue(getattr(g,key)); w.blockSignals(False)
            self.geometry_changed()
            if preferred and self.mech_solutions: self.mech_branch.setCurrentIndex(self.nearest_branch(preferred))
        except (ValueError,TypeError,KeyError,OSError) as exc: self.log.appendPlainText('Không nạp được cơ khí: '+str(exc))

    def update_aperture(self,latest):
        self.jaws.width_mm=None; self.jaws.max_mm=None; self.jaws.object_width_mm=None
        text='Độ mở: — mm · Chưa đủ thông số cơ khí hợp lệ'
        if self.geometry:
            try:
                self.settings()
                if not self.fb: raise ValueError('Chưa có phản hồi encoder')
                width=self.geometry.width(self.fb['position'],self.open.value(),self.closed.value())
                self.jaws.width_mm=width; self.jaws.max_mm=self.geometry.max_mm
                self.jaws.object_width_mm=self.geometry.at_fraction(.55)
                text=f'Độ mở theo mô hình: {width:.2f} mm / {self.geometry.max_mm:.2f} mm'
                if self.jaws.stale: text+=' · Phản hồi cũ'
                if latest['target'] is not None:
                    try: text+=f" · Đích: {self.geometry.width(latest['target'],self.open.value(),self.closed.value()):.2f} mm"
                    except ValueError: text+=' · Đích ngoài khoảng hiệu chuẩn'
            except ValueError as exc: text='Độ mở: — mm · '+str(exc)
        self.aperture.setText(text); self.monitor_aperture.setText(text)
        self.monitor_feedback.setText(self.feedback.text())

    def spin(self,form,name,lo,hi,value,decimals):
        s=QDoubleSpinBox(); s.setDecimals(decimals); s.setRange(lo,hi); s.setValue(value); s.setSingleStep(10**-decimals); form.addRow(name,s); return s
    def refresh_ports(self, automatic=False):
        if automatic and (self.ready or self._connecting or self.port.hasFocus() or self.port.view().isVisible()): return
        old=self.port.currentText().split(' | ',1)[0].strip()
        ports,errors=discover(self.allports.isChecked())
        signature=tuple((p.device,p.label,tuple(p.aliases)) for p in ports)
        if automatic and signature==self._port_signature: return
        self._port_signature=signature
        self.port.blockSignals(True); self.port.clear()
        for p in ports:
            self.port.addItem(p.label,p.device)
            self.port.setItemData(self.port.count()-1,f'{p.hwid}\nAliases: {", ".join(p.aliases)}',Qt.ToolTipRole)
        if old:
            index=next((i for i,p in enumerate(ports) if p.device==old),-1)
            if index>=0: self.port.setCurrentIndex(index)
            elif self._ports_initialized: self.port.setCurrentText(old)
        self.port.blockSignals(False); self._ports_initialized=True
        count=sum(p.usb for p in ports)
        mode_note='Simulation uses no USB. Select Real EC-A4310 to connect hardware.' if self.mode.currentData()=='simulation' else 'Real motor: select the USB serial port, then Connect hardware.'
        self.port_hint.setText(f'{mode_note} Detected: {count} USB serial / {len(ports)} listed. '+('No USB endpoint: use Diagnose USB.' if not count else 'Listed ports may still be busy.'))
        if errors and hasattr(self,'log'): self.log.appendPlainText('; '.join(errors))
    def auto_refresh_ports(self): self.refresh_ports(automatic=True)
    def diagnose_usb(self):
        if self._diagnosing: return
        self._diagnosing=True; self.diagnostic_btn.setEnabled(False)
        selected=self.port.currentText().split(' | ',1)[0].strip()
        def work():
            try: report=diagnose(selected)
            except Exception as e: report='Diagnostic error: '+str(e)
            self.events.put(('diagnostic',{'report':report}))
        threading.Thread(target=work,daemon=True).start()
    def show_diagnostic(self,report):
        self._diagnosing=False; self.diagnostic_btn.setEnabled(True)
        dialog=QDialog(self); dialog.setWindowTitle('USB diagnostics — copy this report'); dialog.resize(850,600)
        box=QVBoxLayout(dialog); edit=QPlainTextEdit(); edit.setReadOnly(True); edit.setPlainText(report); box.addWidget(edit)
        copy=QPushButton('Copy report'); copy.clicked.connect(lambda:QApplication.clipboard().setText(report)); box.addWidget(copy)
        close=QPushButton('Close'); close.clicked.connect(dialog.accept); box.addWidget(close)
        dialog.setAttribute(Qt.WA_DeleteOnClose); dialog.show()
    def connection_settings(self):
        return dict(port=self.port.currentText().split(' | ',1)[0].strip(),motor_id=self.mid.value(),show_all_ports=self.allports.isChecked())

    def load_connection_settings(self):
        try:
            data=read_settings_file(); connection=data['connection']
            self.allports.setChecked(connection.get('show_all_ports',False))
            self.mid.setValue(connection.get('motor_id',0))
            if connection.get('port'): self.port.setEditText(connection['port'])
            self.raw.setValue(data['ui'].get('target_position',700))
        except (OSError,ValueError,TypeError) as exc: self.log.appendPlainText('Không nạp được cài đặt kết nối: '+str(exc))

    def save_connection_settings(self):
        try:
            update_settings_file(connection=self.connection_settings(),ui={'target_position':self.raw.value()})
        except (OSError,ValueError) as exc: self.log.appendPlainText('Không lưu được cài đặt kết nối: '+str(exc))

    def update_connection_buttons(self):
        available=not self.ready and not self._connecting
        for button in (self.connect_btn,self.connect_real,self.connect_simulator):
            button.setEnabled(available)
        self.quick_disconnect.setEnabled(self.ready and not self._connecting)
        self.connect_real.setText('Đang kết nối…' if self._connecting and self.mode.currentData()=='real' else 'Connect real gripper')
        self.connect_simulator.setText('Đang kết nối…' if self._connecting and self.mode.currentData()=='simulation' else 'Connect simulator')

    def quick_connect(self,mode):
        if self.ready or self._connecting: return
        self.mode.setCurrentIndex(self.mode.findData(mode))
        if mode=='real' and not self.port.currentText().strip():
            self.tabs.setCurrentIndex(1); self.port.setFocus()
        self.connect_motor()

    def connect_motor(self):
        if self.ready or self._connecting: return
        if self.mode.currentData()=='real' and not self.port.currentText().strip(): self.log.appendPlainText('Select USB-CAN serial port'); return
        self._connecting=True; self.mode.setEnabled(False); self.connect_btn.setEnabled(False); self.backend.request('connect',port=self.port.currentText().split(' | ',1)[0].strip(),motor_id=self.mid.value() or None,mode=self.mode.currentData())
        self.save_connection_settings()
        self.backend.request('sim_object',present=False)
        self.update_connection_buttons()
    def mode_changed(self):
        sim=self.mode.currentData()=='simulation'
        self.port.setEnabled(True); self.mid.setEnabled(not sim)
        self.connect_btn.setText('Connect simulation' if sim else 'Connect hardware')
        self.refresh_ports()
        self.jaws.mode=self.mode.currentData(); self.jaws.update()
        if sim:
            self.open.setValue(0); self.closed.setValue(90)
            self.speed.setValue(2); self.close_a.setValue(.5); self.hold_a.setValue(.3); self.contact_a.setValue(.4)
            self.load_settings()
        else:
            self.open.setValue(0); self.closed.setValue(0)
            self.load_settings()
            self.log.appendPlainText('Động cơ thật: dùng góc OPEN/CLOSED đã lưu; kết nối thành công sẽ tự bật điều khiển và UDP.')
    def capture(self,widget):
        if self.fb and self.backend.can and time.monotonic()-self.backend.fb_time<.5:
            widget.setValue(self.fb['position'])
        else: self.log.appendPlainText('Connect and obtain fresh position first')
    def settings(self):
        s=Settings(open_deg=self.open.value(),closed_deg=self.closed.value(),speed_rpm=self.speed.value(),close_a=self.close_a.value(),hold_a=self.hold_a.value(),contact_a=self.contact_a.value(),preload_deg=self.preload.value(),move_timeout_s=self.timeout.value())
        s.validate(); return s
    def save_settings(self):
        try:
            s=self.settings()
            section='simulation' if self.mode.currentData()=='simulation' else 'hardware'
            update_settings_file(**{section:asdict(s)},connection=self.connection_settings(),ui={'target_position':self.raw.value()})
            self.log.appendPlainText('Đã lưu thông số vào gripper_settings.json.')
            if self.ready:
                self.backend.request('settings',settings=s)
                self.backend.request('arm',enabled=True)
        except Exception as e: self.log.appendPlainText(str(e))
    def load_settings(self):
        try:
            section='simulation' if self.mode.currentData()=='simulation' else 'hardware'
            s=Settings(**read_settings_file()[section])
            for widget,value in [(self.open,s.open_deg),(self.closed,s.closed_deg),(self.speed,s.speed_rpm),(self.close_a,s.close_a),(self.hold_a,s.hold_a),(self.contact_a,s.contact_a),(self.preload,s.preload_deg),(self.timeout,s.move_timeout_s)]:
                if value is not None: widget.setValue(value)
            self.log.appendPlainText('Đã nạp góc OPEN/CLOSED và thông số động cơ đã lưu.')
        except Exception as e: self.log.appendPlainText('Load settings: '+str(e))
    def enable_connected_controls(self):
        # Run once per successful connection, never from periodic status updates.
        try:
            s=self.settings()
            self.backend.request('settings',settings=s)
            self.backend.request('arm',enabled=True)
        except ValueError as exc:
            self.log.appendPlainText('Đã kết nối. Nhập góc OPEN/CLOSED hợp lệ rồi bấm Save settings: '+str(exc))
        self.backend.request('udp',enabled=True)
    def stop(self): self.backend.stop_requested.set()
    def poll(self):
        latest=None
        for _ in range(100):
            try: kind,data=self.events.get_nowait()
            except queue.Empty: break
            if kind=='log':
                self.log.appendPlainText(data['message'])
            elif kind=='diagnostic': self.show_diagnostic(data['report'])
            elif kind=='status': latest=data
            elif kind=='connection':
                self._connecting=False; self.connect_btn.setEnabled(not data['connected'])
                if data['connected']: self.enable_connected_controls()
            elif kind=='finished': self.log.appendPlainText('Backend stopped')
        if latest:
            self.mode.setEnabled(not latest['connected'] and not self._connecting)
            self.active=latest['active']; self.pending=latest['pending']; self.fb=latest['fb']; self.ready=latest['connected']
            target_text='—' if latest['target'] is None else f"{latest['target']:.3f} deg"
            self.overview.setText(f"{'MÔ PHỎNG' if self.mode.currentData()=='simulation' else 'ĐỘNG CƠ THẬT'} · {'Đã kết nối' if self.ready else 'Chưa kết nối'} · {latest['state']}" + (f" · {latest['error']}" if latest['error'] else ''))
            control_text='Sẵn sàng điều khiển' if latest['armed'] else 'Điều khiển đang dừng'
            udp_text='UDP VR5 :5050 → :5051' if latest['udp'] else 'UDP chưa bật'
            self.overview.setText(self.overview.text()+f' · {control_text} · {udp_text}')
            self.state.setText(f"{latest['state']} | {'CONNECTED' if self.ready else 'DISCONNECTED'} | {control_text} | {udp_text} | Target {target_text} | I_limit {latest['limit']:.1f} A | {latest['error']}")
            if self.fb:
                cur=self.fb.get('current_a'); temp=self.fb.get('temperature_c')
                self.feedback.setText(f"Position {self.fb['position']:.3f} deg | Current {cur if cur is not None else '—'} A | Temperature {temp if temp is not None else '—'} C | Feedback age {latest['feedback_age']:.2f} s")
                self.trace.points.append(cur); self.trace.update()
            self.jaws.stale=not self.ready or latest['feedback_age']>.5
            if not self.fb: self.feedback.setText('Position — | Current — | Temperature —')
            self.jaws.position=self.fb['position'] if self.fb else None
            self.update_aperture(latest)
            self.jaws.target=latest['target']; self.jaws.open_deg=self.open.value(); self.jaws.closed_deg=self.closed.value()
            self.jaws.mode=self.mode.currentData(); self.jaws.object_present=False; self.jaws.state=latest['state']; self.jaws.update()
            for widget in self.config_controls: widget.setEnabled(not self.active and not latest['armed'])
            for b in self.motion_buttons: b.setEnabled(latest['armed'] and self.ready and not self.pending)
        self.update_connection_buttons()
    def closeEvent(self,event):
        self.backend.stop_requested.set(); self.backend.quit_requested.set(); self.backend.join(timeout=3)
        if self.backend.is_alive():
            self.log.appendPlainText('Waiting for serial operation to end…'); event.ignore(); return
        self.save_connection_settings()
        self.timer.stop(); self.port_timer.stop(); self.monitor.close(); event.accept()


def run_application():
    parser=argparse.ArgumentParser(description='Standalone EC Gripper Qt; settings beside this script.')
    parser.add_argument('--import-settings',metavar='DIRECTORY',help='Import legacy JSON files, then exit.')
    parser.add_argument('--check',action='store_true',help='Check dependencies and JSON without opening hardware.')
    args=parser.parse_args()
    if args.import_settings:
        print('Imported '+import_legacy_settings(args.import_settings)+' into '+str(SETTINGS_PATH)); return 0
    if args.check:
        read_settings_file(); print('OK: PySide6 '+PySide6.__version__+', pyserial '+serial.__version__+'; settings: '+str(SETTINGS_PATH)); return 0
    app=QApplication(sys.argv); app.setStyle('Fusion'); win=Window(); win.show()
    return app.exec()

if __name__=='__main__':
    sys.exit(run_application())
