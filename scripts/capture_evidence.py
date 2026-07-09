#!/usr/bin/env python3
"""Capture a timestamped HTTP evidence snapshot for the Syncra demo."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1]


def read_json(url: str):
    with urlopen(url, timeout=3.0) as response:
        return json.loads(response.read().decode("utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Save Syncra HTTP evidence to evidence/*.json.")
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--label", default="normal", help="Short label for the evidence file.")
    parser.add_argument("--history-limit", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = args.base_url.rstrip("/")

    evidence = {
        "captured_at": time.time(),
        "label": args.label,
        "safety_note": "sample/simulated data only; no real robot command path",
        "health": read_json(f"{base}/health"),
        "latest": read_json(f"{base}/state/latest"),
        "history": read_json(f"{base}/state/history?limit={args.history_limit}"),
    }

    evidence_dir = ROOT / "evidence"
    evidence_dir.mkdir(exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    safe_label = "".join(ch for ch in args.label if ch.isalnum() or ch in ("-", "_")) or "snapshot"
    output_path = evidence_dir / f"syncra_evidence_{safe_label}_{timestamp}.json"
    output_path.write_text(json.dumps(evidence, indent=2, sort_keys=True), encoding="utf-8")

    latest = evidence["latest"]
    history = evidence["history"]
    print(f"[evidence] saved {os.fspath(output_path)}")
    print(f"[evidence] latest_status={latest.get('status')} history_count={history.get('count')}")
    print(f"[evidence] warnings={latest.get('warnings', [])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
