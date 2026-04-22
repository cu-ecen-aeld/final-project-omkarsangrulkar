import argparse
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template_string, request
from google import genai
from google.genai import types

from can_bridge import CANBridge


PROMPT = (
    "Return one word only: forward, left, right, or stop. "
    "Prefer forward if the path is open. "
    "Use stop if blocked or unclear."
)


HTML = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Rover Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial, sans-serif;
      background: #0f1115;
      color: #e8eaed;
      margin: 0;
      padding: 20px;
    }
    h1 {
      margin-top: 0;
      font-size: 24px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1.5fr 1fr;
      gap: 20px;
    }
    .card {
      background: #171a21;
      border: 1px solid #2b313d;
      border-radius: 12px;
      padding: 16px;
    }
    .frame {
      width: 100%;
      border-radius: 10px;
      border: 1px solid #333b4a;
      background: #000;
    }
    .label {
      color: #9aa0a6;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 4px;
    }
    .value {
      font-size: 18px;
      margin-bottom: 14px;
      word-break: break-word;
    }
    .mono {
      font-family: monospace;
      font-size: 14px;
      white-space: pre-wrap;
      word-break: break-word;
    }
    .row {
      margin-bottom: 12px;
      padding-bottom: 12px;
      border-bottom: 1px solid #262b36;
    }
    .row:last-child {
      border-bottom: none;
      margin-bottom: 0;
      padding-bottom: 0;
    }
    .small {
      font-size: 13px;
      color: #b0b7c3;
    }
    button {
      background: #2d6cdf;
      border: none;
      color: white;
      border-radius: 8px;
      padding: 10px 14px;
      cursor: pointer;
      margin-right: 8px;
      margin-top: 8px;
      min-width: 90px;
    }
    button.stop {
      background: #d9534f;
    }
    button.mode {
      background: #333b4a;
    }
    button.active {
      background: #198754;
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(3, 90px);
      grid-template-rows: repeat(3, 60px);
      gap: 10px;
      justify-content: start;
      margin-top: 10px;
      margin-bottom: 12px;
    }
    .empty {
      visibility: hidden;
    }
    input[type="range"] {
      width: 100%;
    }
    input[type="text"] {
      width: 100%;
      background: #11161f;
      color: white;
      border: 1px solid #333b4a;
      border-radius: 8px;
      padding: 10px;
      box-sizing: border-box;
    }
    .inline {
      display: flex;
      gap: 10px;
      align-items: center;
    }
  </style>
</head>
<body>
  <h1>Rover Dashboard</h1>
  <div class="small">Live camera + auto/manual + CAN control</div>
  <br>

  <div class="grid">
    <div class="card">
      <div class="label">Live Camera</div>
      <img class="frame" src="/stream.mjpg" alt="camera stream">
    </div>

    <div class="card">
      <div class="row">
        <div class="label">Mode</div>
        <div id="mode" class="value">AUTO</div>
        <button id="autoBtn" class="mode" onclick="setMode('AUTO')">AUTO</button>
        <button id="manualBtn" class="mode" onclick="setMode('MANUAL')">MANUAL</button>
      </div>

      <div class="row">
        <div class="label">Action</div>
        <div id="action" class="value">-</div>
      </div>

      <div class="row">
        <div class="label">Gemini Raw</div>
        <div id="gemini_raw" class="value mono">-</div>
      </div>

      <div class="row">
        <div class="label">Distance</div>
        <div id="distance" class="value">unknown</div>
      </div>

      <div class="row">
        <div class="label">Manual Speed</div>
        <div class="inline">
          <input id="speed" type="range" min="10" max="127" value="58" oninput="speedVal.textContent=this.value">
          <span id="speedVal">58</span>
        </div>
      </div>

      <div class="row">
        <div class="label">Manual Controls</div>
        <div class="controls">
          <button class="empty">.</button>
          <button onmousedown="move('forward')" onmouseup="move('stop')" ontouchstart="move('forward')" ontouchend="move('stop')">▲</button>
          <button class="empty">.</button>

          <button onmousedown="move('left')" onmouseup="move('stop')" ontouchstart="move('left')" ontouchend="move('stop')">◀</button>
          <button class="stop" onclick="move('stop')">STOP</button>
          <button onmousedown="move('right')" onmouseup="move('stop')" ontouchstart="move('right')" ontouchend="move('stop')">▶</button>

          <button class="empty">.</button>
          <button onmousedown="move('backward')" onmouseup="move('stop')" ontouchstart="move('backward')" ontouchend="move('stop')">▼</button>
          <button class="stop" onclick="estop()">E-STOP</button>
        </div>
      </div>

      <div class="row">
        <div class="label">Manual Trigger Input</div>
        <input id="manualInput" type="text" placeholder="Type w / a / s / d / x / e">
        <button onclick="sendManualInput()">Send</button>
      </div>

      <div class="row">
        <div class="label">Last Error</div>
        <div id="last_error" class="value mono">none</div>
      </div>

      <div class="row">
        <div class="label">Last CAN Message</div>
        <div id="last_can" class="value mono">-</div>
      </div>

      <div class="row">
        <div class="label">ESP Heartbeat</div>
        <div id="esp_hb" class="value mono">-</div>
      </div>

      <div class="row">
        <div class="label">ESP Status</div>
        <div id="esp_status" class="value mono">-</div>
      </div>

      <div class="row">
        <div class="label">ESP Telemetry</div>
        <div id="esp_tlm" class="value mono">-</div>
      </div>

      <div class="row">
        <div class="label">Last Gemini Time</div>
        <div id="last_gemini_time" class="value">-</div>
      </div>

      <div class="row">
        <div class="label">Last Update</div>
        <div id="last_update" class="value">-</div>
      </div>
    </div>
  </div>

  <script>
    function currentSpeed() {
      return parseInt(document.getElementById('speed').value);
    }

    async function refreshStatus() {
      try {
        const r = await fetch('/api/status');
        const d = await r.json();

        document.getElementById('mode').textContent = d.mode || 'AUTO';
        document.getElementById('action').textContent = d.action || '-';
        document.getElementById('gemini_raw').textContent = d.gemini_raw || '-';
        document.getElementById('distance').textContent =
          d.distance_cm === null ? 'unknown' : `${d.distance_cm.toFixed(1)} cm`;
        document.getElementById('last_error').textContent = d.last_error || 'none';
        document.getElementById('last_can').textContent = d.last_can || '-';
        document.getElementById('esp_hb').textContent = JSON.stringify(d.esp_hb, null, 2);
        document.getElementById('esp_status').textContent = JSON.stringify(d.esp_status, null, 2);
        document.getElementById('esp_tlm').textContent = JSON.stringify(d.esp_tlm, null, 2);
        document.getElementById('last_gemini_time').textContent = d.last_gemini_time_text || '-';
        document.getElementById('last_update').textContent = d.last_update_text || '-';

        document.getElementById('autoBtn').classList.toggle('active', d.mode === 'AUTO');
        document.getElementById('manualBtn').classList.toggle('active', d.mode === 'MANUAL');
      } catch (e) {
        document.getElementById('last_error').textContent = 'UI refresh failed: ' + e;
      }
    }

    async function setMode(mode) {
      await fetch('/api/mode', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({mode})
      });
    }

    async function move(direction) {
      await fetch('/api/move', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({direction, speed: currentSpeed()})
      });
    }

    async function estop() {
      await fetch('/api/estop', {method: 'POST'});
    }

    async function sendManualInput() {
      const input = document.getElementById('manualInput');
      const text = input.value.trim();
      if (!text) return;

      await fetch('/api/manual_input', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({text, speed: currentSpeed()})
      });

      input.value = '';
    }

    setInterval(refreshStatus, 500);
    refreshStatus();
  </script>
</body>
</html>
"""


@dataclass
class SharedState:
    mode: str = "AUTO"
    action: str = "stop"
    gemini_raw: str = ""
    distance_cm: Optional[float] = None
    last_error: str = ""
    frame_jpeg: Optional[bytes] = None
    gemini_frame_jpeg: Optional[bytes] = None
    last_update_ts: float = 0.0
    last_gemini_time_ts: float = 0.0
    last_can: str = ""
    esp_hb: Optional[dict] = field(default_factory=dict)
    esp_status: Optional[dict] = field(default_factory=dict)
    esp_tlm: Optional[dict] = field(default_factory=dict)
    manual_speed: int = 58


state = SharedState()
state_lock = threading.Lock()


def setup_gemini() -> genai.Client:
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise RuntimeError("GEMINI_API_KEY is not set")
    return genai.Client(api_key=api_key)


def capture_frame_rpi(width: int = 640, height: int = 480, timeout_ms: int = 1):
    cmd = [
        "rpicam-jpeg",
        "-n",
        "--width", str(width),
        "--height", str(height),
        "--timeout", str(timeout_ms),
        "-o", "-",
    ]

    result = subprocess.run(cmd, capture_output=True, text=False)

    if result.returncode != 0:
        err = result.stderr.decode("utf-8", errors="ignore")
        raise RuntimeError(f"rpicam-jpeg failed rc={result.returncode}: {err.strip()}")

    jpg = np.frombuffer(result.stdout, dtype=np.uint8)
    frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)

    if frame is None:
        raise RuntimeError("Could not decode camera JPEG frame")

    return frame


def encode_jpeg(frame, quality: int = 80) -> bytes:
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return encoded.tobytes()


def ask_gemini(client: genai.Client, small_frame) -> tuple[str, str]:
    jpeg_bytes = encode_jpeg(small_frame, quality=70)

    response = client.models.generate_content(
        model="gemini-2.5-flash-lite",
        contents=[
            types.Part.from_bytes(
                data=jpeg_bytes,
                mime_type="image/jpeg",
            ),
            PROMPT,
        ],
    )

    raw = (response.text or "").strip()
    text = raw.lower().strip()

    if "forward" in text:
        return "forward", raw
    if "left" in text:
        return "left", raw
    if "right" in text:
        return "right", raw
    if "stop" in text:
        return "stop", raw

    return "stop", raw


def resize_for_gemini(frame, width: int = 320, height: int = 240):
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def get_distance_from_latest_can(bridge: CANBridge):
    return None


def can_msg_to_text(msg) -> str:
    if msg is None:
        return "-"
    data = " ".join(f"{b:02X}" for b in msg.data)
    return f"ID=0x{msg.arbitration_id:03X} DATA={data}"


def do_manual_action(bridge: CANBridge, direction: str, speed: int) -> str:
    speed = max(0, min(127, int(speed)))

    if direction == "forward":
        bridge.send_drive_lr(speed, speed)
        return "forward"
    if direction == "backward":
        bridge.send_drive_lr(-speed, -speed)
        return "backward"
    if direction == "left":
        bridge.send_drive_lr(-speed, speed)
        return "left"
    if direction == "right":
        bridge.send_drive_lr(speed, -speed)
        return "right"
    if direction == "stop":
        bridge.stop()
        return "stop"

    return "stop"


def apply_manual_text(bridge: CANBridge, text: str, speed: int) -> str:
    t = text.strip().lower()

    if t == "w":
        return do_manual_action(bridge, "forward", speed)
    if t == "s":
        return do_manual_action(bridge, "backward", speed)
    if t == "a":
        return do_manual_action(bridge, "left", speed)
    if t == "d":
        return do_manual_action(bridge, "right", speed)
    if t == "x":
        return do_manual_action(bridge, "stop", speed)
    if t == "e":
        bridge.estop()
        return "estop"

    return "unknown"


def camera_loop(
    bridge: CANBridge,
    width: int,
    height: int,
    camera_interval: float,
    stop_event: threading.Event,
):
    while not stop_event.is_set():
        try:
            frame = capture_frame_rpi(width=width, height=height, timeout_ms=1)
            frame_jpeg = encode_jpeg(frame, quality=80)

            small = resize_for_gemini(frame, width=320, height=240)
            small_jpeg = encode_jpeg(small, quality=70)

            with state_lock:
                state.frame_jpeg = frame_jpeg
                state.gemini_frame_jpeg = small_jpeg
                state.distance_cm = get_distance_from_latest_can(bridge)
                state.esp_hb = bridge.latest.get("esp_hb")
                state.esp_status = bridge.latest.get("esp_status")
                state.esp_tlm = bridge.latest.get("esp_tlm")
                state.last_can = can_msg_to_text(bridge.latest.get("last_msg"))
                state.last_update_ts = time.time()
                state.last_error = ""

            cv2.imwrite("/tmp/rover_debug.jpg", frame)

        except Exception as exc:
            with state_lock:
                state.last_error = f"camera: {exc}"
                state.last_update_ts = time.time()

        time.sleep(camera_interval)


def gemini_loop(
    bridge: CANBridge,
    client: genai.Client,
    speed: int,
    turn: int,
    stop_threshold: float,
    gemini_interval: float,
    stop_event: threading.Event,
):
    while not stop_event.is_set():
        try:
            with state_lock:
                current_mode = state.mode
                gemini_jpeg = state.gemini_frame_jpeg
                dist = state.distance_cm

            if gemini_jpeg is None:
                time.sleep(gemini_interval)
                continue

            if current_mode != "AUTO":
                time.sleep(gemini_interval)
                continue

            if dist is not None and dist < stop_threshold:
                bridge.stop()
                with state_lock:
                    state.action = "stop"
                    state.gemini_raw = "hard stop from ultrasonic"
                    state.last_gemini_time_ts = time.time()
                time.sleep(gemini_interval)
                continue

            arr = np.frombuffer(gemini_jpeg, dtype=np.uint8)
            small_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if small_frame is None:
                raise RuntimeError("Could not decode gemini frame")

            action, raw = ask_gemini(client, small_frame)

            if action == "forward":
                bridge.send_drive_lr(speed, speed)
            elif action == "left":
                bridge.send_drive_lr(-turn, turn)
            elif action == "right":
                bridge.send_drive_lr(turn, -turn)
            else:
                bridge.stop()

            with state_lock:
                state.action = action
                state.gemini_raw = raw
                state.last_gemini_time_ts = time.time()
                state.esp_hb = bridge.latest.get("esp_hb")
                state.esp_status = bridge.latest.get("esp_status")
                state.esp_tlm = bridge.latest.get("esp_tlm")
                state.last_can = can_msg_to_text(bridge.latest.get("last_msg"))
                state.last_error = ""

        except Exception as exc:
            try:
                bridge.stop()
            except Exception:
                pass

            with state_lock:
                state.action = "stop"
                state.last_error = f"gemini: {exc}"
                state.last_gemini_time_ts = time.time()

        time.sleep(gemini_interval)


app = Flask(__name__)
bridge_global: Optional[CANBridge] = None


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/stream.mjpg")
def stream_mjpg():
    def generate():
        last = None
        while True:
            with state_lock:
                frame = state.frame_jpeg

            if frame is None:
                time.sleep(0.05)
                continue

            if frame is last:
                time.sleep(0.03)
                continue

            last = frame
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            time.sleep(0.03)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify(
            {
                "mode": state.mode,
                "action": state.action,
                "gemini_raw": state.gemini_raw,
                "distance_cm": state.distance_cm,
                "last_error": state.last_error,
                "last_can": state.last_can,
                "esp_hb": state.esp_hb,
                "esp_status": state.esp_status,
                "esp_tlm": state.esp_tlm,
                "last_update": state.last_update_ts,
                "last_gemini_time": state.last_gemini_time_ts,
                "last_update_text": (
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.last_update_ts))
                    if state.last_update_ts
                    else "-"
                ),
                "last_gemini_time_text": (
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.last_gemini_time_ts))
                    if state.last_gemini_time_ts
                    else "-"
                ),
            }
        )


@app.route("/api/mode", methods=["POST"])
def api_mode():
    payload = request.get_json(force=True)
    mode = str(payload.get("mode", "AUTO")).upper()

    if mode not in {"AUTO", "MANUAL"}:
        return jsonify({"ok": False, "error": "invalid mode"}), 400

    if bridge_global is None:
        return jsonify({"ok": False, "error": "bridge not ready"}), 500

    with state_lock:
        state.mode = mode
        state.last_update_ts = time.time()
        state.gemini_raw = f"mode changed to {mode}"

    if mode == "MANUAL":
        bridge_global.stop()
        with state_lock:
            state.action = "stop"

    return jsonify({"ok": True, "mode": mode})


@app.route("/api/move", methods=["POST"])
def api_move():
    if bridge_global is None:
        return jsonify({"ok": False, "error": "bridge not ready"}), 500

    payload = request.get_json(force=True)
    direction = str(payload.get("direction", "stop")).lower()
    speed = int(payload.get("speed", 58))

    with state_lock:
        if state.mode != "MANUAL":
            return jsonify({"ok": False, "error": "switch to MANUAL mode first"}), 400

    action = do_manual_action(bridge_global, direction, speed)

    with state_lock:
        state.action = action
        state.manual_speed = speed
        state.last_update_ts = time.time()
        state.gemini_raw = f"manual move: {direction} @ {speed}"

    return jsonify({"ok": True, "action": action})


@app.route("/api/manual_input", methods=["POST"])
def api_manual_input():
    if bridge_global is None:
        return jsonify({"ok": False, "error": "bridge not ready"}), 500

    payload = request.get_json(force=True)
    text = str(payload.get("text", "")).strip()
    speed = int(payload.get("speed", 58))

    with state_lock:
        if state.mode != "MANUAL":
            return jsonify({"ok": False, "error": "switch to MANUAL mode first"}), 400

    action = apply_manual_text(bridge_global, text, speed)

    with state_lock:
        state.action = action
        state.manual_speed = speed
        state.last_update_ts = time.time()
        state.gemini_raw = f"manual input: {text} -> {action}"

    return jsonify({"ok": True, "action": action})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    if bridge_global is not None:
        bridge_global.stop()
    with state_lock:
        state.action = "stop"
        state.gemini_raw = "manual stop from UI"
        state.last_update_ts = time.time()
    return jsonify({"ok": True})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    if bridge_global is not None:
        bridge_global.estop()
    with state_lock:
        state.action = "estop"
        state.gemini_raw = "manual estop from UI"
        state.last_update_ts = time.time()
    return jsonify({"ok": True})


def main() -> int:
    global bridge_global

    parser = argparse.ArgumentParser(description="Low-lag autonomy web dashboard over CAN")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--speed", type=int, default=58)
    parser.add_argument("--turn", type=int, default=45)
    parser.add_argument("--stop-threshold", type=float, default=20.0)
    parser.add_argument("--camera-interval", type=float, default=0.08)
    parser.add_argument("--gemini-interval", type=float, default=1.2)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8082)
    args = parser.parse_args()

    try:
        client = setup_gemini()
    except Exception as exc:
        print(f"Gemini setup failed: {exc}")
        return 1

    bridge = CANBridge(channel=args.channel)
    bridge_global = bridge

    try:
        bridge.connect()
    except Exception as exc:
        print(f"Failed to connect CAN: {exc}")
        return 1

    print(f"[WEB] Dashboard: http://localhost:{args.port}")

    stop_event = threading.Event()

    cam_thread = threading.Thread(
        target=camera_loop,
        args=(
            bridge,
            args.width,
            args.height,
            args.camera_interval,
            stop_event,
        ),
        daemon=True,
    )

    ai_thread = threading.Thread(
        target=gemini_loop,
        args=(
            bridge,
            client,
            args.speed,
            args.turn,
            args.stop_threshold,
            args.gemini_interval,
            stop_event,
        ),
        daemon=True,
    )

    cam_thread.start()
    ai_thread.start()

    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    finally:
        stop_event.set()
        try:
            bridge.stop()
        except Exception:
            pass
        bridge.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
