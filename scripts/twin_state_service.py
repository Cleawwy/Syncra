#!/usr/bin/env python3
"""MQTT subscriber, in-memory twin state store, processing layer, and HTTP API."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Deque, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse


DEFAULT_TOPIC = "syncra/mobilebot/state"
DEFAULT_HTTP_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 8000
STATE_CSV_FIELDS = [
    "run_id",
    "time_iso",
    "received_at",
    "robot_timestamp",
    "robot_id",
    "source",
    "sequence",
    "status",
    "warnings",
    "age_s",
    "x_m",
    "z_m",
    "heading_rad",
    "vx_mps",
    "vz_mps",
    "omega_radps",
    "speed_mps",
    "battery_pct",
    "w0_mps",
    "w1_mps",
    "w2_mps",
]


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _utc_iso(ts: Optional[float] = None) -> str:
    ts = time.time() if ts is None else ts
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


def _safe_run_id(value: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)
    return safe.strip("_") or "syncra_run"


class EvidenceLogger:
    """Run-specific evidence writer for state history and system events."""

    def __init__(self, evidence_dir: str = "evidence", run_id: Optional[str] = None) -> None:
        self.enabled = bool(evidence_dir)
        self._lock = threading.Lock()
        self.run_id = _safe_run_id(run_id or datetime.now().strftime("syncra_%Y%m%d_%H%M%S"))
        self.base_dir = evidence_dir
        self.run_dir = os.path.join(evidence_dir, "runs", self.run_id)
        self.events_path = os.path.join(self.run_dir, "events.jsonl")
        self.states_jsonl_path = os.path.join(self.run_dir, "states.jsonl")
        self.states_csv_path = os.path.join(self.run_dir, "states.csv")
        self.summary_path = os.path.join(self.run_dir, "summary.json")
        self.latest_run_path = os.path.join(evidence_dir, "latest_run.txt")
        self._csv_initialized = False

        if self.enabled:
            os.makedirs(self.run_dir, exist_ok=True)
            with open(self.latest_run_path, "w", encoding="utf-8") as handle:
                handle.write(self.run_id + "\n")
            self._ensure_csv_header()

    def info(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "run_id": self.run_id,
            "run_dir": self.run_dir,
            "events_path": self.events_path,
            "states_jsonl_path": self.states_jsonl_path,
            "states_csv_path": self.states_csv_path,
            "summary_path": self.summary_path,
        }

    def log_event(self, event_type: str, detail: str = "", **fields: Any) -> None:
        if not self.enabled:
            return
        now = time.time()
        record = {
            "run_id": self.run_id,
            "time": now,
            "time_iso": _utc_iso(now),
            "event_type": event_type,
            "detail": detail,
        }
        record.update(fields)
        self._append_jsonl(self.events_path, record)

    def log_state(self, state: Dict[str, Any], processed: Dict[str, Any], received_at: float) -> None:
        if not self.enabled:
            return
        flat = self._flatten_state(state, processed, received_at)
        json_record = {
            **flat,
            "state": state,
            "processed": processed,
        }
        self._append_jsonl(self.states_jsonl_path, json_record)
        self._append_csv(flat)

    def write_summary(self, snapshot: Optional[Dict[str, Any]] = None) -> None:
        if not self.enabled:
            return
        summary = {
            "run_id": self.run_id,
            "written_at": time.time(),
            "written_at_iso": _utc_iso(),
            "snapshot": snapshot,
            "files": self.info(),
        }
        with self._lock:
            with open(self.summary_path, "w", encoding="utf-8") as handle:
                json.dump(summary, handle, indent=2, sort_keys=True)

    def _flatten_state(self, state: Dict[str, Any], processed: Dict[str, Any], received_at: float) -> Dict[str, Any]:
        pose = state.get("pose", {})
        velocity = state.get("velocity", {})
        wheels = state.get("wheels", {})
        warnings = processed.get("warnings") or []
        return {
            "run_id": self.run_id,
            "time_iso": _utc_iso(received_at),
            "received_at": received_at,
            "robot_timestamp": state.get("timestamp"),
            "robot_id": state.get("robot_id"),
            "source": state.get("source"),
            "sequence": state.get("sequence"),
            "status": processed.get("status"),
            "warnings": "; ".join(warnings),
            "age_s": processed.get("age_s"),
            "x_m": pose.get("x_m"),
            "z_m": pose.get("z_m"),
            "heading_rad": pose.get("heading_rad"),
            "vx_mps": velocity.get("vx_mps"),
            "vz_mps": velocity.get("vz_mps"),
            "omega_radps": velocity.get("omega_radps"),
            "speed_mps": processed.get("speed_mps"),
            "battery_pct": state.get("battery_pct"),
            "w0_mps": wheels.get("w0_mps"),
            "w1_mps": wheels.get("w1_mps"),
            "w2_mps": wheels.get("w2_mps"),
        }

    def _append_jsonl(self, path: str, record: Dict[str, Any]) -> None:
        line = json.dumps(record, sort_keys=True)
        with self._lock:
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(line + "\n")

    def _ensure_csv_header(self) -> None:
        if self._csv_initialized:
            return
        with self._lock:
            if self._csv_initialized:
                return
            if not os.path.exists(self.states_csv_path) or os.path.getsize(self.states_csv_path) == 0:
                with open(self.states_csv_path, "w", encoding="utf-8", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=STATE_CSV_FIELDS)
                    writer.writeheader()
            self._csv_initialized = True

    def _append_csv(self, row: Dict[str, Any]) -> None:
        self._ensure_csv_header()
        with self._lock:
            with open(self.states_csv_path, "a", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=STATE_CSV_FIELDS)
                writer.writerow({field: row.get(field) for field in STATE_CSV_FIELDS})


class StateStore:
    """Thread-safe latest/history state store with simple digital twin processing."""

    def __init__(
        self,
        *,
        history_size: int = 100,
        stale_after_s: float = 3.0,
        evidence_logger: Optional[EvidenceLogger] = None,
    ) -> None:
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._latest_received_at: Optional[float] = None
        self._history: Deque[Dict[str, Any]] = deque(maxlen=history_size)
        self._last_error: Optional[str] = None
        self._last_error_at: Optional[float] = None
        self._mqtt_connected = False
        self._last_logged_status: Optional[str] = None
        self.stale_after_s = stale_after_s
        self.evidence_logger = evidence_logger

    def set_mqtt_connected(self, connected: bool) -> None:
        with self._lock:
            self._mqtt_connected = connected

    def ingest(self, payload: Dict[str, Any], *, received_at: Optional[float] = None) -> Dict[str, Any]:
        received_at = received_at if received_at is not None else time.time()
        try:
            state = self._normalize_state(payload)
            processed = self._process_state(state, received_at, received_at)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._last_error_at = received_at
            if self.evidence_logger:
                self.evidence_logger.log_event("processing_error", str(exc), received_at=received_at)
            return {
                "status": "ERROR",
                "warnings": [str(exc)],
                "received_at": received_at,
            }

        record = {
            "received_at": received_at,
            "state": state,
            "processed": processed,
        }

        with self._lock:
            self._latest = copy.deepcopy(state)
            self._latest_received_at = received_at
            self._history.append(copy.deepcopy(record))
            self._last_error = None
            self._last_error_at = None
            self._last_logged_status = processed["status"]

        if self.evidence_logger:
            self.evidence_logger.log_state(state, processed, received_at)
            self.evidence_logger.log_event(
                "state_updated",
                f"sequence={state['sequence']} status={processed['status']}",
                sequence=state["sequence"],
                status=processed["status"],
                source=state["source"],
            )
            if processed["status"] == "ABNORMAL":
                self.evidence_logger.log_event(
                    "abnormal_data_detected",
                    "; ".join(processed.get("warnings", [])),
                    sequence=state["sequence"],
                    status=processed["status"],
                    warnings=processed.get("warnings", []),
                )

        return processed

    def latest_snapshot(self, *, now: Optional[float] = None) -> Dict[str, Any]:
        now = now if now is not None else time.time()
        with self._lock:
            latest = copy.deepcopy(self._latest)
            latest_received_at = self._latest_received_at
            history_count = len(self._history)
            last_error = self._last_error
            last_error_at = self._last_error_at
            mqtt_connected = self._mqtt_connected

        if latest is None:
            status = "ERROR" if last_error else "MISSING"
            warnings = [last_error] if last_error else ["no state has been received yet"]
            return {
                "ok": False,
                "service": "syncra-twin-state-service",
                "status": status,
                "warnings": warnings,
                "latest": None,
                "processed": {
                    "status": status,
                    "warnings": warnings,
                    "received_at": latest_received_at,
                    "last_error_at": last_error_at,
                },
                "age_s": None,
                "history_count": history_count,
                "mqtt_connected": mqtt_connected,
                "evidence": self.evidence_logger.info() if self.evidence_logger else None,
            }

        processed = self._process_state(latest, latest_received_at or now, now)
        self._record_status_transition(latest, processed)
        return {
            "ok": processed["status"] == "OK",
            "service": "syncra-twin-state-service",
            "status": processed["status"],
            "warnings": processed["warnings"],
            "latest": latest,
            "processed": processed,
            "age_s": processed["age_s"],
            "history_count": history_count,
            "mqtt_connected": mqtt_connected,
            "evidence": self.evidence_logger.info() if self.evidence_logger else None,
        }

    def history_snapshot(self, *, limit: int = 20) -> Dict[str, Any]:
        with self._lock:
            items = [] if limit <= 0 else list(self._history)[-limit:]
        return {
            "count": len(items),
            "history": copy.deepcopy(items),
        }

    def health_snapshot(self) -> Dict[str, Any]:
        latest = self.latest_snapshot()
        return {
            "service": "syncra-twin-state-service",
            "ok": True,
            "mqtt_connected": latest["mqtt_connected"],
            "status": latest["status"],
            "warnings": latest["warnings"],
            "history_count": latest["history_count"],
            "age_s": latest["age_s"],
            "evidence": latest.get("evidence"),
        }

    def _normalize_state(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError("payload must be a JSON object")

        pose = payload.get("pose")
        velocity = payload.get("velocity")
        wheels = payload.get("wheels")
        if not isinstance(pose, dict):
            raise ValueError("pose must be an object")
        if not isinstance(velocity, dict):
            raise ValueError("velocity must be an object")
        if not isinstance(wheels, dict):
            raise ValueError("wheels must be an object")

        state = {
            "robot_id": _string(payload.get("robot_id"), "robot_id"),
            "sequence": int(_number(payload.get("sequence"), "sequence")),
            "timestamp": _number(payload.get("timestamp"), "timestamp"),
            "source": _string(payload.get("source"), "source"),
            "pose": {
                "x_m": _number(pose.get("x_m"), "pose.x_m"),
                "z_m": _number(pose.get("z_m"), "pose.z_m"),
                "heading_rad": _number(pose.get("heading_rad"), "pose.heading_rad"),
            },
            "velocity": {
                "vx_mps": _number(velocity.get("vx_mps"), "velocity.vx_mps"),
                "vz_mps": _number(velocity.get("vz_mps"), "velocity.vz_mps"),
                "omega_radps": _number(velocity.get("omega_radps"), "velocity.omega_radps"),
            },
            "wheels": {
                "w0_mps": _number(wheels.get("w0_mps"), "wheels.w0_mps"),
                "w1_mps": _number(wheels.get("w1_mps"), "wheels.w1_mps"),
                "w2_mps": _number(wheels.get("w2_mps"), "wheels.w2_mps"),
            },
        }

        if "battery_pct" in payload and payload.get("battery_pct") is not None:
            state["battery_pct"] = _number(payload.get("battery_pct"), "battery_pct")

        return state

    def _process_state(self, state: Dict[str, Any], received_at: float, now: float) -> Dict[str, Any]:
        warnings: List[str] = []
        status = "OK"
        age_s = max(0.0, now - received_at)

        if age_s > self.stale_after_s:
            status = "STALE"
            warnings.append(f"state age {age_s:.2f}s exceeds stale threshold {self.stale_after_s:.2f}s")

        pose = state["pose"]
        velocity = state["velocity"]
        wheels = state["wheels"]
        speed = math.hypot(velocity["vx_mps"], velocity["vz_mps"])

        abnormal_checks: List[Tuple[bool, str]] = [
            (abs(pose["x_m"]) > 25.0 or abs(pose["z_m"]) > 25.0, "pose is outside 25 m lab demo boundary"),
            (speed > 2.0, f"speed {speed:.2f} m/s exceeds 2.00 m/s demo limit"),
            (abs(velocity["omega_radps"]) > 4.0, "angular speed exceeds 4.00 rad/s demo limit"),
            (
                any(abs(wheels[key]) > 3.0 for key in ("w0_mps", "w1_mps", "w2_mps")),
                "wheel speed exceeds 3.00 m/s demo limit",
            ),
        ]

        if "battery_pct" in state:
            battery = state["battery_pct"]
            abnormal_checks.append((battery < 0.0 or battery > 100.0, "battery percentage is outside 0..100"))

        abnormal = False
        for failed, warning in abnormal_checks:
            if failed:
                abnormal = True
                warnings.append(warning)

        if abnormal and status != "STALE":
            status = "ABNORMAL"

        return {
            "status": status,
            "warnings": warnings,
            "age_s": round(age_s, 3),
            "received_at": received_at,
            "speed_mps": round(speed, 4),
            "stale_after_s": self.stale_after_s,
        }

    def _record_status_transition(self, state: Dict[str, Any], processed: Dict[str, Any]) -> None:
        status = processed["status"]
        with self._lock:
            if status == self._last_logged_status:
                return
            self._last_logged_status = status

        if not self.evidence_logger:
            return
        if status == "STALE":
            self.evidence_logger.log_event(
                "stale_data_detected",
                "; ".join(processed.get("warnings", [])),
                sequence=state.get("sequence"),
                status=status,
                warnings=processed.get("warnings", []),
            )
        elif status == "ABNORMAL":
            self.evidence_logger.log_event(
                "abnormal_data_detected",
                "; ".join(processed.get("warnings", [])),
                sequence=state.get("sequence"),
                status=status,
                warnings=processed.get("warnings", []),
            )


def make_handler(store: StateStore):
    class SyncraHandler(BaseHTTPRequestHandler):
        server_version = "SyncraTwinState/0.1"

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)

            if parsed.path in ("", "/"):
                self._send_json(
                    {
                        "service": "syncra-twin-state-service",
                        "routes": ["/health", "/state/latest", "/state/history?limit=20"],
                    }
                )
                return

            if parsed.path == "/health":
                self._send_json(store.health_snapshot())
                return

            if parsed.path == "/state/latest":
                snapshot = store.latest_snapshot()
                latest = snapshot.get("latest") or {}
                if store.evidence_logger:
                    store.evidence_logger.log_event(
                        "latest_state_served",
                        "Latest state served over HTTP for monitoring/visualization",
                        requester=self.client_address[0],
                        status=snapshot.get("status"),
                        sequence=latest.get("sequence"),
                    )
                self._send_json(snapshot)
                return

            if parsed.path == "/state/history":
                try:
                    limit = int(query.get("limit", ["20"])[0])
                except ValueError:
                    limit = 20
                snapshot = store.history_snapshot(limit=limit)
                if store.evidence_logger:
                    store.evidence_logger.log_event(
                        "history_served",
                        "History served over HTTP for evidence inspection",
                        requester=self.client_address[0],
                        count=snapshot.get("count"),
                    )
                self._send_json(snapshot)
                return

            self._send_json({"error": "not found", "path": parsed.path}, status=404)

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[http] {self.address_string()} - {fmt % args}")

        def _send_json(self, body: Dict[str, Any], *, status: int = 200) -> None:
            encoded = json.dumps(body, indent=2, sort_keys=True).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

    return SyncraHandler


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Syncra MQTT-to-HTTP twin state service.")
    parser.add_argument("--broker", default="127.0.0.1", help="MQTT broker host.")
    parser.add_argument("--mqtt-port", type=int, default=1883, help="MQTT broker port.")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="MQTT state topic.")
    parser.add_argument("--http-host", default=DEFAULT_HTTP_HOST, help="HTTP bind host.")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="HTTP bind port.")
    parser.add_argument("--history-size", type=int, default=100)
    parser.add_argument("--stale-after", type=float, default=3.0, help="Seconds before latest state is stale.")
    parser.add_argument("--evidence-dir", default="evidence", help="Base evidence directory.")
    parser.add_argument("--run-id", help="Optional run id. Defaults to syncra_YYYYMMDD_HHMMSS.")
    parser.add_argument("--disable-evidence", action="store_true", help="Disable persistent evidence logging.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    evidence_logger = None if args.disable_evidence else EvidenceLogger(args.evidence_dir, args.run_id)
    store = StateStore(
        history_size=args.history_size,
        stale_after_s=args.stale_after,
        evidence_logger=evidence_logger,
    )

    print()
    print("=" * 72)
    print("Syncra Twin State Service")
    print("=" * 72)
    print("Purpose: MQTT receiver + latest/history memory + processing + HTTP API.")
    print("Safety: one-way state replication only; no real robot command endpoint.")
    print(f"MQTT input: mqtt://{args.broker}:{args.mqtt_port}/{args.topic}")
    print(f"HTTP output: http://{args.http_host}:{args.http_port}")
    if evidence_logger:
        print(f"Evidence run id: {evidence_logger.run_id}")
        print(f"Evidence folder: {evidence_logger.run_dir}")
    print()

    if evidence_logger:
        evidence_logger.log_event(
            "service_started",
            "Syncra state service started",
            broker=args.broker,
            mqtt_port=args.mqtt_port,
            topic=args.topic,
            http_host=args.http_host,
            http_port=args.http_port,
        )

    try:
        client = make_mqtt_client("syncra-twin-state-service")
    except ImportError:
        return 2

    def on_connect(client, _userdata, _flags, rc):
        connected = rc == 0
        store.set_mqtt_connected(connected)
        if connected:
            print(f"[mqtt] connected to {args.broker}:{args.mqtt_port}; subscribing to {args.topic}")
            if evidence_logger:
                evidence_logger.log_event("mqtt_connected", "MQTT broker connected", broker=args.broker, port=args.mqtt_port)
            client.subscribe(args.topic)
        else:
            print(f"[mqtt] connection failed rc={rc}")
            if evidence_logger:
                evidence_logger.log_event("mqtt_connection_failed", f"rc={rc}", broker=args.broker, port=args.mqtt_port)

    def on_disconnect(_client, _userdata, rc):
        store.set_mqtt_connected(False)
        print(f"[mqtt] disconnected rc={rc}")
        if evidence_logger:
            evidence_logger.log_event("mqtt_disconnected", f"rc={rc}", rc=rc)

    def on_message(_client, _userdata, message):
        received_at = time.time()
        if evidence_logger:
            evidence_logger.log_event("message_received", "MQTT message received", topic=message.topic, bytes=len(message.payload))

        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except Exception as exc:
            processed = store.ingest({"bad_payload": str(exc)}, received_at=received_at)
        else:
            processed = store.ingest(payload, received_at=received_at)

        snapshot = store.latest_snapshot()
        latest = snapshot.get("latest") or {}
        sequence = latest.get("sequence", "none")
        print(
            f"[state] topic={message.topic} seq={sequence} status={processed['status']} "
            f"warnings={len(processed.get('warnings', []))}"
        )

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    for attempt in range(1, 16):
        try:
            client.connect(args.broker, args.mqtt_port, keepalive=60)
            break
        except OSError as exc:
            if attempt == 15:
                print(f"[mqtt] could not connect to {args.broker}:{args.mqtt_port}: {exc}", file=sys.stderr)
                if evidence_logger:
                    evidence_logger.log_event("mqtt_connect_failed", str(exc), broker=args.broker, port=args.mqtt_port)
                return 1
            print(f"[mqtt] waiting for broker {args.broker}:{args.mqtt_port} attempt={attempt}/15")
            time.sleep(1.0)

    client.loop_start()

    httpd = ThreadingHTTPServer((args.http_host, args.http_port), make_handler(store))
    print(f"[http] serving http://{args.http_host}:{args.http_port}")
    print("[http] routes: /health, /state/latest, /state/history?limit=20")
    print("[run-order] next: start the sample publisher or use docker compose")
    if evidence_logger:
        print(f"[evidence] states JSONL: {evidence_logger.states_jsonl_path}")
        print(f"[evidence] states CSV:   {evidence_logger.states_csv_path}")
        print(f"[evidence] events JSONL: {evidence_logger.events_path}")
        evidence_logger.log_event("http_server_started", "HTTP server started", host=args.http_host, port=args.http_port)

    def handle_stop_signal(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_stop_signal)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[service] stopped by user")
    finally:
        snapshot = store.latest_snapshot()
        if evidence_logger:
            evidence_logger.log_event("service_stopped", "Syncra state service stopped", status=snapshot.get("status"))
            evidence_logger.write_summary(snapshot)
        httpd.server_close()
        client.loop_stop()
        client.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
