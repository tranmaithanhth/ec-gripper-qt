#!/usr/bin/env python3
"""
Simple first-test controller for:
  - Waveshare USB-CAN-A
  - ENCOS EC-A4310-P2-36

The script does NOT move the motor automatically.
It first configures USB-CAN-A to 1 Mbps / Standard CAN / Normal mode,
queries the motor ID, switches it to Q&A mode, reads position, then shows
an interactive menu.

Install:
    pip install pyserial

Run:
    python ec_a4310_waveshare_test.py --port COM5
"""

from __future__ import annotations

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


def choose_port(cli_port: Optional[str]) -> str:
    if cli_port:
        return cli_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial COM port found")

    print("Available serial ports:")
    for i, p in enumerate(ports, start=1):
        print(f"  {i}. {p.device}  {p.description}")

    idx = int(input("Select port number: ").strip())
    return ports[idx - 1].device


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Example: COM5 or /dev/ttyUSB0")
    parser.add_argument("--id", type=lambda s: int(s, 0), help="Known motor ID, e.g. 0x01")
    args = parser.parse_args()

    port = choose_port(args.port)
    print(f"Opening {port} @ {SERIAL_BAUD} baud...")

    can = WaveshareUsbCanA(port)
    try:
        can.configure_1m_standard_normal()
        motor = EC_A4310(can, motor_id=args.id)

        if motor.motor_id is None:
            print("\n[1] Querying motor CAN ID (non-motion command)...")
            mid = motor.discover_id()
            print(f"Motor ID = 0x{mid:03X}")
        else:
            print(f"Using motor ID = 0x{motor.motor_id:03X}")

        print("\n[2] Switching motor to Q&A mode...")
        ok = motor.set_qna_mode()
        print("Q&A mode:", "OK" if ok else "No ACK (continue cautiously; it may already be in Q&A mode)")

        print("\n[3] Reading current position...")
        pos = motor.query_position()
        print(f"Current position value = {pos:.6f}")

        print("\nFirst motion test")
        print("  Recommendation: motor unloaded or gripper mechanically clear.")
        print("  Start with delta = +2 or -2 degrees, speed = 1 rpm, current limit = 0.5 A.")
        print("  Enter 0 to exit without moving.")
        delta = float(input("Delta angle [deg]: ").strip())
        if abs(delta) < 1e-12:
            print("No motion command sent.")
            return
        if abs(delta) > 5.0:
            raise ValueError("For the first test this script limits |delta| to 5 degrees")

        confirm = input(
            f"Move from {pos:.3f} to {pos + delta:.3f} with Imax=0.5 A? Type YES: "
        ).strip()
        if confirm != "YES":
            print("Cancelled. No motion command sent.")
            return

        # The product manual says CAN control should be used after the driver has started
        # successfully for at least ~5 s. This short wait is harmless if it is already ready.
        print("Safety wait: keep motor powered for at least 5 s before CAN motion control...")
        time.sleep(5.2)

        motor.goto_position(
            target_deg=pos + delta,
            speed_rpm=1.0,
            current_limit_a=0.5,
            command_time_s=1.2,
            rate_hz=20.0,
        )

        time.sleep(0.6)
        try:
            pos2 = motor.query_position()
            print(f"Position after test = {pos2:.6f}")
        except Exception as exc:
            print(f"Could not read final position: {exc}")

    finally:
        can.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
