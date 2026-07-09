#!/usr/bin/env python3
"""Print Syncra HTTP health/latest/history evidence."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict
from urllib.error import URLError
from urllib.request import urlopen


def read_json(url: str) -> Dict[str, Any]:
    with urlopen(url, timeout=3.0) as response:
        return json.loads(response.read().decode("utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check Syncra twin state HTTP endpoints.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--history-limit", type=int, default=5)
    parser.add_argument("--expect-status", help="Optional expected status, for example OK or STALE.")
    parser.add_argument("--json", action="store_true", help="Print raw JSON.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = args.base_url.rstrip("/")

    print()
    print("=" * 72)
    print("Syncra HTTP State Check")
    print("=" * 72)
    print("Purpose: prove latest state, history memory, and processed status are readable.")
    print()

    try:
        health = read_json(f"{base}/health")
        latest = read_json(f"{base}/state/latest")
        history = read_json(f"{base}/state/history?limit={args.history_limit}")
    except URLError as exc:
        print(f"[check] HTTP service is not reachable: {exc}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps({"health": health, "latest": latest, "history": history}, indent=2, sort_keys=True))
    else:
        print(f"[health] status={health.get('status')} mqtt_connected={health.get('mqtt_connected')}")
        print(f"[health] warnings={health.get('warnings', [])}")

        state = latest.get("latest")
        if state:
            pose = state["pose"]
            velocity = state["velocity"]
            print(
                f"[latest] robot={state['robot_id']} seq={state['sequence']} "
                f"status={latest.get('status')} age={latest.get('age_s')}s"
            )
            print(
                f"[latest] pose x={pose['x_m']:.3f}m z={pose['z_m']:.3f}m "
                f"heading={pose['heading_rad']:.3f}rad"
            )
            print(
                f"[latest] velocity vx={velocity['vx_mps']:.3f}m/s "
                f"vz={velocity['vz_mps']:.3f}m/s omega={velocity['omega_radps']:.3f}rad/s"
            )
            print(f"[latest] warnings={latest.get('warnings', [])}")
        else:
            print(f"[latest] no state yet status={latest.get('status')} warnings={latest.get('warnings', [])}")

        sequences = [
            item.get("state", {}).get("sequence")
            for item in history.get("history", [])
        ]
        print(f"[history] count={history.get('count')} sequences={sequences}")
        print()
        print("[proof] MQTT data reached the state service if latest/history show sequence numbers.")
        print("[proof] Processing works if status changes between OK, STALE, and ABNORMAL.")

    if args.expect_status and latest.get("status") != args.expect_status:
        print(
            f"[check] expected status {args.expect_status}, got {latest.get('status')}",
            file=sys.stderr,
        )
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
