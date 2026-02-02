#!/usr/bin/env python3
import json
import os
import shlex
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from booster_interface.msg import BoosterApiReqMsg, Subtitle
    from booster_interface.srv import RpcService
except ImportError:  # Allow running off-robot without ROS2
    rclpy = None
    Node = None
    BoosterApiReqMsg = None
    Subtitle = None
    RpcService = None

LLM_CLIENT_PATH = (
    Path(__file__).resolve().parents[1]
    / "booster_robotics_sdk_ros2"
    / "booster_ros2_example"
    / "rtc_client"
    / "src"
)
if LLM_CLIENT_PATH.exists():
    sys.path.append(str(LLM_CLIENT_PATH))
try:
    from llm_web_client import load_config, request_llm
except ImportError:
    load_config = None
    request_llm = None


API_IDS = {
    "change_mode": 2000,
    "move": 2001,
    "rotate_head": 2004,
    "wave_hand": 2005,
    "rotate_head_dir": 2006,
    "lie_down": 2007,
    "get_up": 2008,
    "handshake": 2015,
    "dance": 2016,
    "push_up": 2019,
}

AI_API_IDS = {
    "start": 2000,
    "stop": 2001,
    "speak": 2002,
}


def make_msg(api_id, body=None):
    msg = BoosterApiReqMsg()
    msg.api_id = api_id
    msg.body = json.dumps(body) if body is not None else ""
    return msg


class GestureRunner:
    def __init__(self, cmd, cooldown_s=3.0):
        self.cmd = cmd
        self.cooldown_s = cooldown_s
        self.last_trigger = 0.0
        self.process = None
        self.enabled = True
        self.lock = threading.Lock()

    def trigger(self):
        with self.lock:
            if not self.enabled:
                return False
            now = time.time()
            if now - self.last_trigger < self.cooldown_s:
                return False
            if self.process and self.process.poll() is None:
                return False
            self.last_trigger = now
            if self.cmd:
                self.process = subprocess.Popen(self.cmd)
                return True
        return False

    def enable(self, on=True):
        with self.lock:
            self.enabled = on


class BoosterWebBridge:
    def __init__(self, gesture_runner, subtitle_topic):
        if Node is None:
            raise RuntimeError("ROS2 not available; cannot start local bridge.")
        self.node = Node("booster_web_bridge")
        self.rpc_client = self.node.create_client(RpcService, "booster_rpc_service")
        self.rtc_client = self.node.create_client(RpcService, "booster_rtc_service")
        self.gesture_runner = gesture_runner
        self.subtitle_sub = self.node.create_subscription(
            Subtitle, subtitle_topic, self._on_subtitle, 10
        )

    def _on_subtitle(self, msg: Subtitle):
        if msg.text and msg.definite and msg.paragraph:
            self.gesture_runner.trigger()

    def call_rpc(self, api_id, body=None, use_rtc=False, timeout=2.0):
        client = self.rtc_client if use_rtc else self.rpc_client
        if not client.wait_for_service(timeout_sec=timeout):
            return {"ok": False, "error": "service_unavailable"}
        req = RpcService.Request()
        req.msg = make_msg(api_id, body)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if future.result() is None:
            return {"ok": False, "error": "no_response"}
        return {"ok": True, "status": future.result().msg.status, "body": future.result().msg.body}


class ApiHandler(SimpleHTTPRequestHandler):
    def do_POST(self):
        if not self.path.startswith("/api/"):
            self.send_error(404)
            return

        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            payload = {}

        response = self.server.state.handle_api(self.path, payload)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(response).encode("utf-8"))

    def do_GET(self):
        if self.path.startswith("/api/"):
            response = self.server.state.handle_api(self.path, {})
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(response).encode("utf-8"))
            return
        return super().do_GET()


class AppState:
    def __init__(self, bridge, gesture_runner, remote_base=""):
        self.bridge = bridge
        self.gesture_runner = gesture_runner
        self.llm_config_path = os.environ.get(
            "LLM_CONFIG_PATH", "~/.config/booster/ai_chat_web.json"
        )
        self.llm_config = load_config(self.llm_config_path) if load_config else None
        self.remote_base = remote_base.rstrip("/")

    def _forward_api(self, path, payload):
        if not self.remote_base:
            return {"ok": False, "error": "remote_base_not_configured"}
        url = f"{self.remote_base}{path}"
        data = json.dumps(payload or {}).encode("utf-8")
        req = urllib.request.Request(
            url, data=data, headers={"Content-Type": "application/json"}, method="POST"
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            return {"ok": False, "error": f"http_error_{exc.code}"}
        except urllib.error.URLError as exc:
            return {"ok": False, "error": f"url_error_{exc.reason}"}
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"ok": False, "error": "invalid_remote_response", "raw": raw}

    def handle_api(self, path, payload):
        if self.bridge is None and path != "/api/llm/query":
            return self._forward_api(path, payload)
        if path == "/api/ping":
            if self.bridge is None:
                return self._forward_api(path, payload)
            return {"ok": True}
        if path == "/api/move":
            body = {
                "vx": payload.get("vx", 0.0),
                "vy": payload.get("vy", 0.0),
                "vyaw": payload.get("vyaw", 0.0),
            }
            return self.bridge.call_rpc(API_IDS["move"], body)
        if path == "/api/stop":
            return self.bridge.call_rpc(API_IDS["move"], {"vx": 0.0, "vy": 0.0, "vyaw": 0.0})
        if path == "/api/rotate_head":
            body = {
                "pitch": payload.get("pitch", 0.0),
                "yaw": payload.get("yaw", 0.0),
            }
            return self.bridge.call_rpc(API_IDS["rotate_head"], body)
        if path == "/api/rotate_head_dir":
            body = {
                "pitch_direction": payload.get("pitch", 0),
                "yaw_direction": payload.get("yaw", 0),
            }
            return self.bridge.call_rpc(API_IDS["rotate_head_dir"], body)
        if path == "/api/wave_hand":
            body = {
                "hand_index": payload.get("hand_index", 1),
                "hand_action": payload.get("hand_action", 0),
            }
            return self.bridge.call_rpc(API_IDS["wave_hand"], body)
        if path == "/api/handshake":
            body = {"hand_action": payload.get("hand_action", 0)}
            return self.bridge.call_rpc(API_IDS["handshake"], body)
        if path == "/api/lie_down":
            return self.bridge.call_rpc(API_IDS["lie_down"], None)
        if path == "/api/get_up":
            return self.bridge.call_rpc(API_IDS["get_up"], None)
        if path == "/api/push_up":
            return self.bridge.call_rpc(API_IDS["push_up"], None)
        if path == "/api/dance":
            body = {"dance_id": payload.get("dance_id", 0)}
            return self.bridge.call_rpc(API_IDS["dance"], body)
        if path == "/api/change_mode":
            body = {"mode": payload.get("mode", 1)}
            return self.bridge.call_rpc(API_IDS["change_mode"], body)
        if path == "/api/ai/start":
            body = {
                "interrupt_mode": False,
                "asr_config": {
                    "interrupt_speech_duration": 200,
                    "interrupt_keywords": [],
                },
                "llm_config": {
                    "system_prompt": payload.get("system_prompt", ""),
                    "welcome_msg": payload.get("welcome_message", ""),
                    "prompt_name": "",
                },
                "tts_config": {
                    "voice_type": payload.get(
                        "voice_type",
                        "zh_female_shuangkuaisisi_emo_v2_mars_bigtts",
                    ),
                    "ignore_bracket_text": [3],
                },
                "enable_face_tracking": False,
            }
            return self.bridge.call_rpc(AI_API_IDS["start"], body, use_rtc=True)
        if path == "/api/ai/stop":
            return self.bridge.call_rpc(AI_API_IDS["stop"], None, use_rtc=True)
        if path == "/api/ai/speak":
            body = {"msg": payload.get("msg", "")}
            return self.bridge.call_rpc(AI_API_IDS["speak"], body, use_rtc=True)
        if path == "/api/llm/query":
            if request_llm is None:
                return {"ok": False, "error": "llm_client_unavailable"}
            text = payload.get("text") or payload.get("query") or ""
            if not text:
                return {"ok": False, "error": "empty_query"}
            config = self.llm_config or {}
            response, err = request_llm(text, config)
            if err:
                return {"ok": False, "error": err}
            return {"ok": True, "response": response}
        if path == "/api/gesture_trigger":
            return {"ok": self.gesture_runner.trigger()}
        if path == "/api/gesture/enable":
            self.gesture_runner.enable(True)
            return {"ok": True}
        if path == "/api/gesture/disable":
            self.gesture_runner.enable(False)
            return {"ok": True}
        return {"ok": False, "error": "unknown_endpoint"}


class WebServer(ThreadingHTTPServer):
    def __init__(self, address, handler, state):
        super().__init__(address, handler)
        self.state = state


def main():
    root = Path(__file__).resolve().parent
    os.chdir(root)

    gesture_cmd = os.environ.get(
        "K1_GESTURE_CMD",
        "python /opt/booster/booster_gym/deploy/deploy.py --config=k1_gesture_001.yaml --auto-start --duration=6",
    )
    cmd = shlex.split(gesture_cmd) if gesture_cmd else []
    gesture_runner = GestureRunner(cmd=cmd, cooldown_s=3.0)

    subtitle_topic = os.environ.get("SUBTITLE_TOPIC", "subtitle")

    bridge = None
    if rclpy is not None:
        rclpy.init()
        bridge = BoosterWebBridge(gesture_runner, subtitle_topic)

    remote_base = os.environ.get("ROBOT_API_BASE", "").strip()
    state = AppState(bridge, gesture_runner, remote_base=remote_base)
    port = int(os.environ.get("WEB_PILOT_PORT", "8000"))
    server = WebServer(("0.0.0.0", port), ApiHandler, state)

    print(f"Booster Web Pilot running on http://0.0.0.0:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if bridge is not None:
            bridge.node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
