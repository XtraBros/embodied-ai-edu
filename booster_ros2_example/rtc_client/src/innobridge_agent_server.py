import argparse
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs

import rclpy
from rclpy.node import Node

from booster_interface.msg import RemoteControllerState
from booster_interface.srv import RpcService

from llm_web_client import load_config, request_llm
from llm_web_test import handle_action, parse_response_action, speak


class _Context:
    def __init__(self, node, client, config, joystick_publisher, enable_tts):
        self.node = node
        self.client = client
        self.config = config
        self.joystick_publisher = joystick_publisher
        self.enable_tts = enable_tts


def _extract_response_action(response, response_field):
    speech_text = ""
    action_cmd = ""
    payload = None
    if isinstance(response, str):
        cleaned = response.strip()
        if cleaned.startswith("```"):
            cleaned = cleaned.strip("`").strip()
            if cleaned.lower().startswith("json"):
                cleaned = cleaned[4:].strip()
        try:
            payload = json.loads(cleaned)
        except json.JSONDecodeError:
            if "{" in cleaned and "}" in cleaned:
                snippet = cleaned[cleaned.find("{") : cleaned.rfind("}") + 1]
                try:
                    payload = json.loads(snippet)
                except json.JSONDecodeError:
                    payload = None
        if payload is None and response_field == "response":
            speech_text, action_cmd = parse_response_action(cleaned)
            if not speech_text:
                speech_text = cleaned
    elif isinstance(response, dict):
        payload = response
    if isinstance(payload, dict):
        speech_text = payload.get(response_field, "") or ""
        action_cmd = (
            payload.get("action_name")
            or payload.get("action")
            or ""
        )
        if isinstance(action_cmd, str):
            action_cmd = action_cmd.strip().strip("`").strip()
        if not speech_text:
            speech_text = response if isinstance(response, str) else ""
    return speech_text, action_cmd, payload


def _parse_request_body(handler):
    length = int(handler.headers.get("Content-Length", "0") or 0)
    raw = handler.rfile.read(length) if length else b""
    content_type = (handler.headers.get("Content-Type") or "").lower()
    text = ""
    if "application/json" in content_type:
        try:
            payload = json.loads(raw.decode("utf-8", errors="replace") or "{}")
        except json.JSONDecodeError:
            return "", {"error": "Invalid JSON payload."}
        if isinstance(payload, dict):
            text = (
                payload.get("text")
                or payload.get("prompt")
                or payload.get("query")
                or ""
            )
        else:
            text = ""
    elif "application/x-www-form-urlencoded" in content_type:
        parsed = parse_qs(raw.decode("utf-8", errors="replace"))
        text = (parsed.get("text") or parsed.get("prompt") or [""])[0]
    else:
        text = raw.decode("utf-8", errors="replace")
    return text.strip(), None


class InnoBridgeHandler(BaseHTTPRequestHandler):
    def _send_json(self, status_code, payload):
        body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        ctx = self.server.context
        text, err = _parse_request_body(self)
        if err:
            self._send_json(400, err)
            return
        if not text:
            self._send_json(400, {"error": "Missing text input."})
            return
        response, err = request_llm(text, ctx.config)
        if err:
            self._send_json(500, {"error": err})
            return
        response_field = ctx.config.get("response_field", "response")
        ignore_action = bool(ctx.config.get("ignore_action", False))
        speech_text, action_cmd, payload = _extract_response_action(
            response, response_field
        )
        if not ignore_action and action_cmd:
            handle_action(ctx.node, action_cmd, ctx.config, ctx.joystick_publisher)
        if ctx.enable_tts and speech_text:
            speak(ctx.node, ctx.client, speech_text)
        ctx.node.get_logger().info(f"InnoBridge prompt: {text}")
        ctx.node.get_logger().info(f"InnoBridge response: {speech_text}")
        self._send_json(
            200,
            {
                "response": speech_text,
                "action_name": action_cmd,
                "raw": payload if payload is not None else response,
            },
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address.")
    parser.add_argument("--port", type=int, default=9000, help="HTTP port.")
    parser.add_argument(
        "--config",
        default="~/.config/booster/ai_chat_web.json",
        help="Path to JSON config containing LLM endpoint settings.",
    )
    parser.add_argument(
        "--no-tts",
        action="store_true",
        help="Disable TTS output (print response only).",
    )
    args, ros_args = parser.parse_known_args()

    config = load_config(args.config)
    enable_tts = bool(config.get("enable_tts", True)) and not args.no_tts

    rclpy.init(args=ros_args)
    node = Node("innobridge_agent")
    client = node.create_client(RpcService, "booster_rtc_service")
    joystick_topic = config.get("joystick_topic", "/remote_controller_state")
    joystick_publisher = node.create_publisher(
        RemoteControllerState, joystick_topic, 10
    )

    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            return 1
        node.get_logger().info("booster_rtc_service not available, waiting...")

    server = HTTPServer((args.host, args.port), InnoBridgeHandler)
    server.context = _Context(node, client, config, joystick_publisher, enable_tts)

    node.get_logger().info(
        f"InnoBridge Agent HTTP server listening on http://{args.host}:{args.port}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
