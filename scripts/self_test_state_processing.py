#!/usr/bin/env python3
"""Self-test Syncra state storage and processing without MQTT."""

from __future__ import annotations

import time

from twin_state_service import StateStore


def make_state(sequence: int, *, abnormal: bool = False):
    return {
        "robot_id": "syncra-mobilebot-01",
        "sequence": sequence,
        "timestamp": time.time(),
        "source": "self-test",
        "pose": {
            "x_m": 30.0 if abnormal else 1.0,
            "z_m": 0.5,
            "heading_rad": 0.2,
        },
        "velocity": {
            "vx_mps": 2.5 if abnormal else 0.2,
            "vz_mps": 0.1,
            "omega_radps": 5.0 if abnormal else 0.1,
        },
        "wheels": {
            "w0_mps": 4.0 if abnormal else 0.1,
            "w1_mps": 0.1,
            "w2_mps": 0.1,
        },
        "battery_pct": 150.0 if abnormal else 90.0,
    }


def assert_equal(actual, expected, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def main() -> int:
    store = StateStore(history_size=5, stale_after_s=3.0, evidence_logger=None)

    missing = store.latest_snapshot()
    assert_equal(missing["status"], "MISSING", "initial status")

    received_at = time.time()
    processed = store.ingest(make_state(1), received_at=received_at)
    assert_equal(processed["status"], "OK", "normal ingest status")
    assert_equal(store.latest_snapshot(now=received_at + 1.0)["status"], "OK", "fresh latest status")
    assert_equal(store.latest_snapshot(now=received_at + 5.0)["status"], "STALE", "stale latest status")

    abnormal = store.ingest(make_state(2, abnormal=True), received_at=time.time())
    assert_equal(abnormal["status"], "ABNORMAL", "abnormal ingest status")
    assert_equal(store.history_snapshot(limit=10)["count"], 2, "history count")

    print("[self-test] state storage, history, stale detection, and abnormal detection passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
