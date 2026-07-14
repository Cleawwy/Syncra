#!/usr/bin/env python3
"""Read FEETECH servo telemetry and publish it as Syncra robot state.

This script is read-only. It sends no movement commands to the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Dict, Iterable, List, Optional

import paho.mqtt.client as mqtt
import serial


TOPIC = "syncra/mobilebot/state"
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


def delta_counts(current: int, previous: int, modulo: int = 4096) -> int:
    diff = current - previous
    half = modulo // 2
    if diff > half:
        diff -= modulo
    elif diff < -half:
        diff += modulo
    return diff


def make_client(client_id: str) -> mqtt.Client:
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    return mqtt.Client(client_id=client_id)


def battery_pct_from_voltage(voltage_v: Optional[float]) -> float:
    if voltage_v is None:
        return 0.0
    pct = (voltage_v - 9.0) / (12.6 - 9.0) * 100.0
    return max(0.0, min(100.0, pct))


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish read-only real servo telemetry into Syncra MQTT.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/ttyUSB0.")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", default="7,8,9")
    parser.add_argument("--broker", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--topic", default=TOPIC)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--wheel-scale", type=float, default=0.001)
    args = parser.parse_args()

    ids = [int(part.strip()) for part in args.ids.split(",") if part.strip()]
    if len(ids) != 3:
        raise SystemExit("--ids must contain exactly three servo IDs, e.g. 7,8,9")

    client = make_client("syncra-real-servo-readonly")
    client.connect(args.broker, args.mqtt_port, keepalive=60)
    client.loop_start()

    previous_positions: Dict[int, int] = {}
    pose = {"x_m": 0.0, "z_m": 0.0, "heading_rad": 0.0}
    sequence = 0
    last_t = time.time()

    print("Syncra real servo MQTT bridge")
    print("Safety: read-only telemetry. No servo movement commands are sent.")
    print(f"Serial: {args.port} baud={args.baud} ids={ids}")
    print(f"MQTT: {args.broker}:{args.mqtt_port}/{args.topic}")

    with serial.Serial(args.port, args.baud, timeout=0.08) as port:
        while True:
            now = time.time()
            dt = max(0.001, now - last_t)
            last_t = now
            sequence += 1

            positions: Dict[int, Optional[int]] = {}
            voltages: List[float] = []
            temperatures: Dict[str, Optional[int]] = {}

            for servo_id in ids:
                position = u16_le(read_mem(port, servo_id, ADDR_PRESENT_POSITION, 2))
                voltage_raw = (read_mem(port, servo_id, ADDR_PRESENT_VOLTAGE, 1) or [None])[0]
                temperature = (read_mem(port, servo_id, ADDR_PRESENT_TEMPERATURE, 1) or [None])[0]
                positions[servo_id] = position
                if voltage_raw is not None:
                    voltages.append(float(voltage_raw) / 10.0)
                temperatures[f"id{servo_id}_c"] = temperature

            wheel_rates: List[float] = []
            for servo_id in ids:
                position = positions.get(servo_id)
                if position is None:
                    wheel_rates.append(0.0)
                    continue
                previous = previous_positions.get(servo_id)
                previous_positions[servo_id] = position
                if previous is None:
                    wheel_rates.append(0.0)
                else:
                    wheel_rates.append(delta_counts(position, previous) / dt * args.wheel_scale)

            # This is a visualization proxy, not final physical odometry.
            forward = sum(wheel_rates) / 3.0
            omega = (wheel_rates[0] - wheel_rates[1]) * 0.25
            pose["heading_rad"] += omega * dt
            pose["x_m"] += math.sin(pose["heading_rad"]) * forward * dt
            pose["z_m"] += math.cos(pose["heading_rad"]) * forward * dt

            voltage_v = sum(voltages) / len(voltages) if voltages else None
            state = {
                "robot_id": "syncra-real-mobilebot-01",
                "sequence": sequence,
                "timestamp": now,
                "source": "real-servo-readonly",
                "pose": {
                    "x_m": round(pose["x_m"], 4),
                    "z_m": round(pose["z_m"], 4),
                    "heading_rad": round(pose["heading_rad"], 5),
                },
                "velocity": {
                    "vx_mps": 0.0,
                    "vz_mps": round(forward, 4),
                    "omega_radps": round(omega, 5),
                },
                "wheels": {
                    "w0_mps": round(wheel_rates[0], 4),
                    "w1_mps": round(wheel_rates[1], 4),
                    "w2_mps": round(wheel_rates[2], 4),
                },
                "battery_pct": round(battery_pct_from_voltage(voltage_v), 2),
                "servo": {
                    "ids": ids,
                    "position_counts": {str(key): value for key, value in positions.items()},
                    "voltage_v": voltage_v,
                    "temperature_c": temperatures,
                },
            }

            payload = json.dumps(state, separators=(",", ":"))
            client.publish(args.topic, payload, qos=0, retain=False).wait_for_publish(timeout=2.0)
            print(
                f"seq={sequence} pos={state['servo']['position_counts']} "
                f"wheels={state['wheels']} voltage={voltage_v}"
            )
            time.sleep(max(0.0, (1.0 / args.hz) - (time.time() - now)))


if __name__ == "__main__":
    raise SystemExit(main())
