#!/usr/bin/env python3
"""Generate safe simulated mobile robot state and publish it over MQTT.

This is a one-way sample data source. It does not connect to, control, or
command a real robot.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any, Dict, Tuple


TOPIC = "syncra/mobilebot/state"
WHEEL_ANGLES_DEG = [90.0, 210.0, 330.0]
WHEEL_OFFSET_R_M = 0.18


def make_mqtt_client(client_id: str):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print(
            "Missing dependency: paho-mqtt. Install with: python -m pip install -r requirements.txt",
            file=sys.stderr,
        )
        raise

    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    return mqtt.Client(client_id=client_id)


def body_to_wheels(vx_mps: float, vz_mps: float, omega_radps: float) -> Tuple[float, float, float]:
    wheels = []
    for deg in WHEEL_ANGLES_DEG:
        angle = math.radians(deg)
        wheel_speed = (
            -vx_mps * math.sin(angle)
            + vz_mps * math.cos(angle)
            + omega_radps * WHEEL_OFFSET_R_M
        )
        wheels.append(wheel_speed)
    return tuple(wheels)


def normal_motion(t_s: float) -> Tuple[float, float, float]:
    vx_local = 0.08 * math.sin(t_s * 0.7)
    vz_local = 0.28 + 0.08 * math.sin(t_s * 0.35)
    omega = 0.35 * math.sin(t_s * 0.45)
    return vx_local, vz_local, omega


def abnormal_motion(t_s: float) -> Tuple[float, float, float]:
    # Deliberately outside the state-service demo bounds.
    vx_local = 2.8 + 0.2 * math.sin(t_s)
    vz_local = 2.4
    omega = 5.5
    return vx_local, vz_local, omega


def make_state(
    *,
    robot_id: str,
    sequence: int,
    source: str,
    mode: str,
    t_s: float,
    dt_s: float,
    pose: Dict[str, float],
) -> Dict[str, Any]:
    if mode == "abnormal":
        vx_local, vz_local, omega = abnormal_motion(t_s)
    else:
        vx_local, vz_local, omega = normal_motion(t_s)

    heading = pose["heading_rad"]
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    vx_world = cos_h * vx_local - sin_h * vz_local
    vz_world = sin_h * vx_local + cos_h * vz_local

    pose["x_m"] += vx_world * dt_s
    pose["z_m"] += vz_world * dt_s
    pose["heading_rad"] += omega * dt_s

    if mode == "abnormal":
        pose["x_m"] = 30.0 + 0.2 * math.sin(t_s)
        battery_pct = 140.0
    else:
        battery_pct = max(20.0, 96.0 - (t_s * 0.02))

    w0, w1, w2 = body_to_wheels(vx_local, vz_local, omega)

    return {
        "robot_id": robot_id,
        "sequence": sequence,
        "timestamp": time.time(),
        "source": source,
        "pose": {
            "x_m": round(pose["x_m"], 4),
            "z_m": round(pose["z_m"], 4),
            "heading_rad": round(pose["heading_rad"], 5),
        },
        "velocity": {
            "vx_mps": round(vx_world, 4),
            "vz_mps": round(vz_world, 4),
            "omega_radps": round(omega, 5),
        },
        "wheels": {
            "w0_mps": round(w0, 4),
            "w1_mps": round(w1, 4),
            "w2_mps": round(w2, 4),
        },
        "battery_pct": round(battery_pct, 2),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish safe simulated Syncra robot state over MQTT.")
    parser.add_argument("--broker", default="127.0.0.1", help="MQTT broker host.")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port.")
    parser.add_argument("--topic", default=TOPIC, help="MQTT topic for state messages.")
    parser.add_argument("--robot-id", default="syncra-mobilebot-01")
    parser.add_argument("--source", default="sample-generator")
    parser.add_argument("--mode", choices=["normal", "abnormal"], default="normal")
    parser.add_argument("--hz", type=float, default=2.0, help="Publish frequency.")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means forever.")
    parser.add_argument("--print-json", action="store_true", help="Print full JSON payloads.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        print("--hz must be greater than 0", file=sys.stderr)
        return 2

    print()
    print("=" * 72)
    print("Syncra Sample MQTT Publisher")
    print("=" * 72)
    print("Purpose: generate simulated mobile robot state for the digital twin.")
    print("Safety: sample data only; no real robot connection and no command output.")
    print()

    try:
        client = make_mqtt_client("syncra-sample-publisher")
    except ImportError:
        return 2

    for attempt in range(1, 16):
        try:
            client.connect(args.broker, args.port, keepalive=60)
            break
        except OSError as exc:
            if attempt == 15:
                print(f"[publisher] MQTT broker is not reachable at {args.broker}:{args.port}: {exc}", file=sys.stderr)
                print("[publisher] Start Mosquitto first, for example: mosquitto -v", file=sys.stderr)
                return 1
            print(f"[publisher] waiting for MQTT broker {args.broker}:{args.port} attempt={attempt}/15")
            time.sleep(1.0)

    client.loop_start()

    interval_s = 1.0 / args.hz
    start = time.time()
    last = start
    sequence = 0
    pose = {"x_m": 0.0, "z_m": 0.0, "heading_rad": 0.0}

    print(
        f"[publisher] one-way simulated state -> mqtt://{args.broker}:{args.port}/{args.topic} "
        f"mode={args.mode} hz={args.hz}"
    )
    print("[publisher] safety: this script publishes state only; it sends no robot commands")

    try:
        while True:
            now = time.time()
            elapsed = now - start
            if args.duration > 0 and elapsed >= args.duration:
                break

            dt_s = max(0.001, now - last)
            last = now
            sequence += 1

            state = make_state(
                robot_id=args.robot_id,
                sequence=sequence,
                source=args.source,
                mode=args.mode,
                t_s=elapsed,
                dt_s=dt_s,
                pose=pose,
            )
            payload = json.dumps(state, separators=(",", ":"))
            result = client.publish(args.topic, payload, qos=0, retain=False)
            try:
                result.wait_for_publish(timeout=2.0)
            except TypeError:
                result.wait_for_publish()

            if args.print_json:
                print(payload)
            else:
                pose_out = state["pose"]
                vel_out = state["velocity"]
                print(
                    f"[publisher] seq={sequence:04d} x={pose_out['x_m']:.2f}m "
                    f"z={pose_out['z_m']:.2f}m heading={pose_out['heading_rad']:.2f}rad "
                    f"speed={math.hypot(vel_out['vx_mps'], vel_out['vz_mps']):.2f}m/s"
                )

            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("\n[publisher] stopped by user")
    finally:
        client.loop_stop()
        client.disconnect()

    print("[publisher] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
