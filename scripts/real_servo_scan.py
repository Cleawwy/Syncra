#!/usr/bin/env python3
"""Read-only FEETECH STS/SCS bus scan for the Syncra robot."""

from __future__ import annotations

import argparse
import time
from typing import Dict, Iterable, List, Optional

import serial


ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63


def checksum(parts: Iterable[int]) -> int:
    return (~sum(parts)) & 0xFF


def packet(servo_id: int, instruction: int, params: List[int]) -> bytes:
    length = len(params) + 2
    body = [servo_id, length, instruction, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def read_response(port: serial.Serial) -> Optional[Dict[str, object]]:
    deadline = time.time() + (port.timeout or 0.1)
    window = bytearray()

    while time.time() < deadline:
        byte = port.read(1)
        if not byte:
            continue
        window += byte
        if len(window) > 2:
            window = window[-2:]
        if window == b"\xff\xff":
            header = port.read(2)
            if len(header) != 2:
                return None
            servo_id = header[0]
            length = header[1]
            tail = port.read(length)
            if len(tail) != length:
                return None
            error = tail[0]
            params = list(tail[1:-1])
            received_checksum = tail[-1]
            expected = checksum([servo_id, length, error, *params])
            if received_checksum != expected:
                return None
            return {"id": servo_id, "error": error, "params": params}

    return None


def ping(port: serial.Serial, servo_id: int) -> bool:
    port.reset_input_buffer()
    port.write(packet(servo_id, 0x01, []))
    port.flush()
    response = read_response(port)
    return bool(response and response["id"] == servo_id and response["error"] == 0)


def read_mem(port: serial.Serial, servo_id: int, address: int, size: int) -> Optional[List[int]]:
    port.reset_input_buffer()
    port.write(packet(servo_id, 0x02, [address, size]))
    port.flush()
    response = read_response(port)
    if not response or response["id"] != servo_id or response["error"] != 0:
        return None
    return list(response["params"])


def u16_le(values: Optional[List[int]]) -> Optional[int]:
    if not values or len(values) < 2:
        return None
    return int(values[0]) | (int(values[1]) << 8)


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan and read FEETECH bus servo telemetry.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", default="1-20", help="ID list/range, e.g. 7,8,9 or 1-20.")
    args = parser.parse_args()

    ids: List[int] = []
    for part in args.ids.split(","):
        part = part.strip()
        if "-" in part:
            start, end = [int(x) for x in part.split("-", 1)]
            ids.extend(range(start, end + 1))
        elif part:
            ids.append(int(part))

    found: List[int] = []
    with serial.Serial(args.port, args.baud, timeout=0.08) as port:
        print(f"Scanning {args.port} at {args.baud}...")
        for servo_id in ids:
            if ping(port, servo_id):
                found.append(servo_id)
                print(f"ID {servo_id}: found")

        if not found:
            print("Found IDs: none")
            return 1

        print()
        for servo_id in found:
            pos = u16_le(read_mem(port, servo_id, ADDR_PRESENT_POSITION, 2))
            speed_load = read_mem(port, servo_id, ADDR_PRESENT_SPEED, 4) or []
            voltage = (read_mem(port, servo_id, ADDR_PRESENT_VOLTAGE, 1) or [None])[0]
            temp = (read_mem(port, servo_id, ADDR_PRESENT_TEMPERATURE, 1) or [None])[0]
            voltage_v = None if voltage is None else voltage / 10.0
            print(f"Servo ID {servo_id}")
            print(f"  position_counts: {pos}")
            print(f"  speed_load_raw: {speed_load}")
            print(f"  voltage_v: {voltage_v}")
            print(f"  temperature_c: {temp}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
