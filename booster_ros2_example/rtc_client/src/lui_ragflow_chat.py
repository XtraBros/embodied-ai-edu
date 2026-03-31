import argparse
import difflib
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

POC_SRC = Path(__file__).resolve().parents[3] / "AI_chat_POC" / "src"
if str(POC_SRC) not in sys.path:
    sys.path.insert(0, str(POC_SRC))

from booster_interface.msg import BoosterApiReqMsg
from booster_interface.srv import RpcService

try:
    from booster_interface.msg import Subtitle
except ImportError:
    Subtitle = None

try:
    from booster_interface.msg import AsrChunk
except ImportError:
    AsrChunk = None

try:
    from booster_interface.msg import RawBytesMsg
except ImportError:
    RawBytesMsg = None

try:
    from std_msgs.msg import String
except ImportError:
    String = None

try:
    from booster_interface.msg import RemoteControllerState
except ImportError:
    RemoteControllerState = None

try:
    from llm_web_client import load_config, request_llm_with_raw
    _LLM_WITH_RAW = True
except ImportError:
    from llm_web_client import load_config, request_llm
    request_llm_with_raw = None
    _LLM_WITH_RAW = False
from main import (
    extract_payload_from_text,
    handle_action,
    handle_song,
    load_action_and_song_lists,
    MUSIC_STOP_WORDS,
    parse_response_action,
)


DEFAULT_LUI_START_SCRIPT = "/opt/booster/BoosterLui/bin/start_client.sh"
DEFAULT_LUI_SERVICE = "booster_lui_service"
DEFAULT_RTC_SERVICE = "booster_rtc_service"
DEFAULT_LUI_ASR_TOPIC = "/lui_asr_chunk"
DEFAULT_TTS_START_API_ID = 1050
DEFAULT_TTS_STOP_API_ID = 1051
DEFAULT_TTS_SEND_API_ID = 1052
DEFAULT_ASR_START_API_ID = 1000
DEFAULT_ASR_STOP_API_ID = 1001
DEFAULT_SPEAK_API_ID = 2002
DEFAULT_AI_CHAT_START_API_ID = 2000
DEFAULT_AI_CHAT_STOP_API_ID = 2001


def make_msg(api_id, body):
    msg = BoosterApiReqMsg()
    msg.api_id = int(api_id)
    msg.body = body
    return msg


def call_api(node, client, api_id, body, timeout_sec=5.0):
    node.get_logger().info(f"RPC api_id={api_id} body={body}")
    start_time = time.monotonic()
    req = RpcService.Request()
    req.msg = make_msg(api_id, body)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f"RpcService call timed out (api_id={api_id})")
        return False
    elapsed = time.monotonic() - start_time
    result = future.result()
    if not result or not getattr(result, "msg", None):
        node.get_logger().error(f"RpcService response missing (api_id={api_id})")
        return False
    response = result.msg
    detail = getattr(response, "body", "")
    node.get_logger().info(
        f"RPC api_id={api_id} status={response.status} elapsed={elapsed:.2f}s body={detail}"
    )
    return response.status == 0


def sanitize_tts_text(text):
    if not isinstance(text, str) or not text:
        return ""
    out = []
    in_asterisk = False
    for ch in text:
        if ch == "*":
            in_asterisk = not in_asterisk
            continue
        if not in_asterisk:
            out.append(ch)
    cleaned = "".join(out).strip()
    return cleaned


def _read_process_bytes(proc, chunk_size):
    while True:
        data = proc.stdout.read(chunk_size)
        if not data:
            break
        yield data


def run_asr_vosk(config, node):
    try:
        from vosk import Model, KaldiRecognizer
    except ImportError:
        node.get_logger().error("Vosk is not installed. Install 'vosk' to use ASR.")
        return ""

    model_path = (config.get("asr_model_path") or "").strip()
    model_path = os.path.expanduser(model_path)
    if not model_path:
        node.get_logger().error("ASR model path not configured (asr_model_path).")
        return ""

    sample_rate = int(config.get("asr_sample_rate", 16000))
    chunk_ms = float(config.get("asr_chunk_ms", 200))
    max_sec = float(config.get("asr_max_sec", 12.0))
    chunk_size = int(sample_rate * 2 * (chunk_ms / 1000.0))

    node.get_logger().info("ASR (vosk): starting microphone capture...")
    log_level = str(config.get("vosk_log_level", "error")).lower()
    if log_level == "off":
        os.environ["VOSK_LOG_LEVEL"] = "-1"
    elif log_level == "error":
        os.environ["VOSK_LOG_LEVEL"] = "0"
    elif log_level == "warn":
        os.environ["VOSK_LOG_LEVEL"] = "1"
    elif log_level == "info":
        os.environ["VOSK_LOG_LEVEL"] = "2"
    model = Model(model_path)
    recognizer = KaldiRecognizer(model, sample_rate)

    command = ["arecord", "-q", "-f", "S16_LE", "-r", str(sample_rate), "-c", "1"]
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    except FileNotFoundError:
        node.get_logger().error("ASR (vosk): 'arecord' not found.")
        return ""
    start_time = time.time()
    final_text = ""
    try:
        for data in _read_process_bytes(proc, chunk_size):
            now = time.time()
            if max_sec > 0 and (now - start_time) > max_sec:
                node.get_logger().info("ASR (vosk): max duration reached, stopping.")
                break
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result() or "{}")
                final_text = (result.get("text") or "").strip()
                if final_text:
                    break
                start_time = now
            else:
                partial = json.loads(recognizer.PartialResult() or "{}").get("partial", "")
                if partial:
                    node.get_logger().info(f"ASR (vosk partial): {partial}")
                    start_time = now
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    if final_text:
        node.get_logger().info(f"ASR (vosk final): {final_text}")
    else:
        node.get_logger().info("ASR (vosk): no speech detected.")
    return final_text


def estimate_tts_delay(text, config):
    words = len((text or "").split())
    wpm = float(config.get("ai_chat_stop_delay_wpm", 160.0))
    multiplier = float(config.get("ai_chat_stop_delay_multiplier", 2.0))
    floor_sec = float(config.get("ai_chat_stop_delay_floor_sec", 1.0))
    padding_sec = float(config.get("ai_chat_stop_delay_padding_sec", 0.5))
    max_sec = float(config.get("ai_chat_stop_delay_max_sec", 30.0))
    if wpm <= 0:
        wpm = 160.0
    duration = words / (wpm / 60.0) if words else 0.0
    delay = max(floor_sec, duration * multiplier) + padding_sec
    if max_sec > 0:
        delay = min(delay, max_sec)
    return max(delay, 0.0)


def start_lui_daemon(node, config):
    script = (
        os.environ.get("LUI_START_SCRIPT")
        or config.get("lui_start_script")
        or DEFAULT_LUI_START_SCRIPT
    )
    script = os.path.expanduser(script)
    if not script:
        node.get_logger().info("LUI start script not configured; skipping daemon start.")
        return None
    if not os.path.exists(script):
        node.get_logger().warn(f"LUI start script not found at {script}; skipping daemon start.")
        return None
    load_script = config.get("lui_load_device_script", "/opt/booster/RTCCli/bin/load_device.sh")
    if load_script and os.path.exists(load_script):
        node.get_logger().info(f"Running LUI device setup: {load_script}")
        try:
            subprocess.run(["/bin/bash", load_script], check=False)
        except Exception as exc:
            node.get_logger().warn(f"Failed to run LUI device setup: {exc}")
    if is_lui_running(node):
        node.get_logger().info("LUI daemon already running; skipping start.")
        return None
    node.get_logger().info(f"Starting LUI daemon via {script}")
    try:
        return subprocess.Popen([script], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    except PermissionError:
        try:
            if os.access(script, os.R_OK):
                node.get_logger().warn("LUI start script not executable; retrying via bash.")
                return subprocess.Popen(["/bin/bash", script], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            node.get_logger().warn("LUI start script is not readable; cannot start.")
        except Exception as exc:
            node.get_logger().warn(f"Failed to start LUI daemon via bash: {exc}")
        return None
    except OSError as exc:
        node.get_logger().warn(f"Failed to start LUI daemon: {exc}")
        return None


def is_lui_running(node):
    pgrep = shutil.which("pgrep")
    if not pgrep:
        return False
    patterns = [
        "BoosterLui",
        "start_client.sh",
        "booster_lui",
    ]
    for pattern in patterns:
        try:
            result = subprocess.run(
                [pgrep, "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if result.returncode == 0:
                return True
        except Exception as exc:
            node.get_logger().warn(f"Failed to check LUI daemon status: {exc}")
            break
    return False


def decode_raw_bytes(msg):
    raw = bytes(msg.msg)
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("utf-8", errors="replace")


def extract_asr_text(msg):
    for field in ("text", "data", "msg", "content"):
        if hasattr(msg, field):
            value = getattr(msg, field)
            if isinstance(value, (bytes, bytearray)):
                try:
                    return value.decode("utf-8")
                except UnicodeDecodeError:
                    return value.decode("utf-8", errors="replace")
            if isinstance(value, str):
                return value
    return ""


class LuiRagflowChat:
    def __init__(
        self,
        node,
        config,
        wake_word,
        silence_sec,
        print_asr=True,
        enable_stdin=True,
        joystick_publisher=None,
    ):
        self.node = node
        self.config = config
        self.wake_word = wake_word.strip()
        self.silence_sec = float(silence_sec)
        self.print_asr = bool(print_asr)
        self.enable_stdin = bool(enable_stdin)
        self.joystick_publisher = joystick_publisher
        self.wake_word_aliases = config.get("wake_word_aliases") or []
        self.wake_word_fuzzy = bool(config.get("wake_word_fuzzy", True))
        self.wake_word_fuzzy_threshold = float(
            config.get("wake_word_fuzzy_threshold", 0.8)
        )

        self.lui_service = (
            config.get("lui_service_name")
            or config.get("tts_service_name")
            or DEFAULT_LUI_SERVICE
        )
        self.rtc_service = config.get("rtc_service_name") or DEFAULT_RTC_SERVICE
        self.lui_asr_topic = config.get("lui_asr_topic") or DEFAULT_LUI_ASR_TOPIC
        self.use_ai_speak = bool(config.get("tts_use_ai_speak", False))

        self.client = node.create_client(RpcService, self.lui_service)
        self._wait_for_service(self.client, self.lui_service)
        self.rtc_client = None
        if self.use_ai_speak:
            self.rtc_client = node.create_client(RpcService, self.rtc_service)
            self._wait_for_service(self.rtc_client, self.rtc_service)

        self._wake_pattern = re.compile(re.escape(self.wake_word), re.IGNORECASE)
        self._waiting_wake = True
        self._current_text = ""
        self._last_update = None
        self._last_partial = ""
        self._utterance_start = None
        self._asr_enabled = True
        self._last_shortcut_ts = 0.0
        self._last_asr_msg_ts = time.monotonic()
        self._last_asr_warn_ts = 0.0
        self._asr_backend = str(self.config.get("asr_backend", "lui")).lower()
        self._vosk_enabled = threading.Event()
        self._vosk_stop = threading.Event()
        self._vosk_thread = None
        self._wake_grace_sec = float(self.config.get("wake_word_grace_sec", 0.0))
        self._wake_grace_until = 0.0

        self._asr_subscription = None
        if self._asr_backend == "lui":
            self._asr_subscription = self._create_asr_subscription()
        elif self._asr_backend == "vosk":
            self._start_vosk_listener()
        self._timer = node.create_timer(0.2, self._on_timer)
        self._request_lock = threading.Lock()
        self._stdin_stop = threading.Event()
        self._stdin_thread = None
        if self.enable_stdin:
            self._start_stdin_listener()
        self._shortcut_subscription = self._create_shortcut_subscription()

    def _start_vosk_listener(self):
        self._vosk_enabled.set()

        def _loop():
            while not self._vosk_stop.is_set():
                if not self._vosk_enabled.is_set():
                    time.sleep(0.1)
                    continue
                text = run_asr_vosk(self.config, self.node)
                if not text:
                    continue
                if not self.wake_word:
                    self._process_text(text)
                    continue
                found, after = self._split_on_wake_word(text)
                if not found:
                    continue
                if after:
                    self._process_text(after)

        self._vosk_thread = threading.Thread(target=_loop, daemon=True)
        self._vosk_thread.start()

    def _wait_for_service(self, client, name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f"{name} not available, waiting...")

    def _create_asr_subscription(self):
        msg_type = (self.config.get("lui_asr_msg_type") or "").strip().lower()
        if msg_type in ("asr_chunk", "asrchunk") and AsrChunk:
            return self.node.create_subscription(AsrChunk, self.lui_asr_topic, self._on_asr_chunk, 10)
        if msg_type == "subtitle" and Subtitle:
            return self.node.create_subscription(Subtitle, self.lui_asr_topic, self._on_subtitle, 10)
        if msg_type == "string" and String:
            return self.node.create_subscription(String, self.lui_asr_topic, self._on_string, 10)
        if msg_type == "raw_bytes" and RawBytesMsg:
            return self.node.create_subscription(RawBytesMsg, self.lui_asr_topic, self._on_raw_bytes, 10)

        if AsrChunk:
            self.node.get_logger().info("Using booster_interface/AsrChunk for LUI ASR.")
            return self.node.create_subscription(AsrChunk, self.lui_asr_topic, self._on_asr_chunk, 10)
        if Subtitle:
            self.node.get_logger().info("Using booster_interface/Subtitle for LUI ASR.")
            return self.node.create_subscription(Subtitle, self.lui_asr_topic, self._on_subtitle, 10)
        if String:
            self.node.get_logger().info("Using std_msgs/String for LUI ASR.")
            return self.node.create_subscription(String, self.lui_asr_topic, self._on_string, 10)
        if RawBytesMsg:
            self.node.get_logger().info("Using booster_interface/RawBytesMsg for LUI ASR.")
            return self.node.create_subscription(RawBytesMsg, self.lui_asr_topic, self._on_raw_bytes, 10)

        raise RuntimeError("No supported message type available for LUI ASR topic.")

    def _create_shortcut_subscription(self):
        if not RemoteControllerState:
            return None
        topic = self.config.get("joystick_topic", "/remote_controller_state")
        return self.node.create_subscription(
            RemoteControllerState, topic, self._on_shortcut_event, 10
        )

    def _on_shortcut_event(self, msg):
        buttons = set(b.upper() for b in (self.config.get("shortcut_trigger_buttons") or []))
        if not buttons:
            return
        press_event = int(self.config.get("shortcut_trigger_event", self.config.get("joystick_event_press", 1539)))
        debounce_sec = float(self.config.get("shortcut_trigger_debounce_sec", 1.0))
        now = time.monotonic()
        if (now - self._last_shortcut_ts) < debounce_sec:
            return
        if int(getattr(msg, "event", 0)) != press_event:
            return
        has_lt = getattr(msg, "lt", False)
        has_rt = getattr(msg, "rt", False)
        if buttons == {"LT", "RT"} and not (has_lt and has_rt):
            return
        if buttons == {"LT"} and not has_lt:
            return
        if buttons == {"RT"} and not has_rt:
            return
        self._last_shortcut_ts = now
        if self._asr_enabled:
            self.node.get_logger().info("Shortcut trigger: disabling ASR.")
            self.stop_asr()
            self._asr_enabled = False
            return
        self.node.get_logger().info("Shortcut trigger: enabling ASR.")
        self.start_asr()
        self._asr_enabled = True

    def _on_subtitle(self, msg):
        text = msg.text.strip() if msg.text else ""
        if not text:
            return
        self._last_asr_msg_ts = time.monotonic()
        is_final = bool(getattr(msg, "definite", False))
        self._handle_asr_text(text, is_final=is_final)

    def _on_asr_chunk(self, msg):
        text = extract_asr_text(msg).strip()
        if not text:
            self.node.get_logger().info(f"ASR chunk: {msg}")
            return
        self._last_asr_msg_ts = time.monotonic()
        self._handle_asr_text(text, is_final=False)

    def _on_string(self, msg):
        text = msg.data.strip() if msg.data else ""
        if not text:
            return
        self._last_asr_msg_ts = time.monotonic()
        self._handle_asr_text(text, is_final=False)

    def _on_raw_bytes(self, msg):
        text = decode_raw_bytes(msg).strip()
        if not text:
            return
        self._last_asr_msg_ts = time.monotonic()
        self._handle_asr_text(text, is_final=False)

    def _split_on_wake_word(self, text):
        if not self.wake_word:
            return False, text
        original_tokens = text.split()
        normalized_tokens = [
            re.sub(r"[^a-z0-9]", "", token.lower()) for token in original_tokens
        ]

        def _pattern_tokens(pattern):
            return [
                re.sub(r"[^a-z0-9]", "", token.lower())
                for token in pattern.split()
                if token.strip()
            ]

        patterns = [self.wake_word]
        for alias in self.wake_word_aliases:
            if isinstance(alias, str) and alias.strip():
                patterns.append(alias.strip())

        for pattern in patterns:
            pattern_tokens = _pattern_tokens(pattern)
            if not pattern_tokens:
                continue
            if normalized_tokens[: len(pattern_tokens)] == pattern_tokens:
                after = " ".join(original_tokens[len(pattern_tokens) :]).strip()
                return True, after
            if self.wake_word_fuzzy and normalized_tokens:
                sample = " ".join(normalized_tokens[: len(pattern_tokens)])
                target = " ".join(pattern_tokens)
                if sample and target:
                    ratio = difflib.SequenceMatcher(None, sample, target).ratio()
                    if ratio >= self.wake_word_fuzzy_threshold:
                        after = " ".join(original_tokens[len(pattern_tokens) :]).strip()
                        return True, after

        match = self._wake_pattern.search(text)
        if not match:
            return False, text
        return True, text[match.end():].strip()

    def _handle_asr_text(self, text, is_final=False):
        if text == self._last_partial and not is_final:
            return
        self._last_partial = text
        if self.print_asr:
            self.node.get_logger().info(f"ASR: {text}")

        if self._waiting_wake:
            found, after = self._split_on_wake_word(text)
            if not found:
                return
            self._waiting_wake = False
            self._current_text = after
            self._last_update = time.monotonic()
            self._utterance_start = self._last_update
            if self._wake_grace_sec > 0:
                self._wake_grace_until = self._last_update + self._wake_grace_sec
            self.node.get_logger().info("Wake word detected; listening...")
            if not after:
                return

        self._current_text = text
        self._last_update = time.monotonic()
        if is_final:
            self._finalize_input()

    def _on_timer(self):
        warn_sec = float(self.config.get("asr_no_msg_warn_sec", 10.0))
        if self._asr_backend == "lui" and warn_sec > 0:
            now = time.monotonic()
            if (now - self._last_asr_msg_ts) > warn_sec and (now - self._last_asr_warn_ts) > warn_sec:
                self._last_asr_warn_ts = now
                self.node.get_logger().warn(
                    f"No ASR messages received for {warn_sec:.0f}s on {self.lui_asr_topic}."
                )
        if self._waiting_wake:
            return
        if self._last_update is None:
            return
        now = time.monotonic()
        if self._wake_grace_until and now < self._wake_grace_until:
            return
        if (now - self._last_update) < self.silence_sec:
            return
        self._finalize_input()

    def _finalize_input(self):
        text = self._current_text.strip()
        self._waiting_wake = True
        self._current_text = ""
        self._last_update = None
        utterance_start = self._utterance_start
        self._utterance_start = None
        self._process_text(text)
        if utterance_start is not None:
            utterance_sec = max(time.monotonic() - utterance_start, 0.0)
            self.node.get_logger().info(
                f"Timing: asr_capture_sec={utterance_sec:.2f}"
            )

    def _process_text(self, text):
        text = (text or "").strip()
        if not text:
            return
        if not self._request_lock.acquire(blocking=False):
            self.node.get_logger().warn("LLM request already in progress; skipping input.")
            return
        t_input = time.monotonic()
        try:
            self.node.get_logger().info(f"User input: {text}")
            t_llm_start = time.monotonic()
            if _LLM_WITH_RAW and request_llm_with_raw:
                reply, err, raw = request_llm_with_raw(text, self.config)
                if err:
                    self.node.get_logger().warn(f"LLM request failed: {err}")
                    return
                if raw:
                    self.node.get_logger().info(f"Backend raw: {raw}")
            else:
                reply, err = request_llm(text, self.config)
                if err:
                    self.node.get_logger().warn(f"LLM request failed: {err}")
                    return
                self.node.get_logger().info(f"Backend reply: {reply}")
            llm_sec = max(time.monotonic() - t_llm_start, 0.0)
            self.node.get_logger().info(f"Timing: llm_sec={llm_sec:.2f}")
            payload = extract_payload_from_text(reply)
            speech_text = reply
            action_cmd = ""
            song_value = None

            if isinstance(payload, dict):
                speech_text = payload.get("response") or reply
                action_cmd = payload.get("action", "") or ""
                song_value = payload.get("song")
                if isinstance(action_cmd, str):
                    action_cmd = action_cmd.strip().strip("`").strip()
            else:
                speech_text, action_cmd = parse_response_action(reply)

            speech_text = sanitize_tts_text(speech_text)
            def _run_actions():
                handle_song(self.node, song_value, self.config)
                if not song_value and isinstance(action_cmd, str) and action_cmd.strip().lower() in MUSIC_STOP_WORDS:
                    handle_song(self.node, "stop", self.config)
                ignore_action = bool(self.config.get("ignore_action", False))
                if not ignore_action:
                    handle_action(self.node, action_cmd, self.config, self.joystick_publisher)

            run_parallel = bool(self.config.get("action_parallel_tts", True))
            if run_parallel:
                threading.Thread(target=_run_actions, daemon=True).start()
            else:
                _run_actions()

            if speech_text:
                self.node.get_logger().info(f"Assistant: {speech_text}")
                t_tts_start = time.monotonic()
                self.send_tts_text(speech_text)
                tts_sec = max(time.monotonic() - t_tts_start, 0.0)
                total_sec = max(time.monotonic() - t_input, 0.0)
                self.node.get_logger().info(
                    f"Timing: tts_api_sec={tts_sec:.2f} total_to_tts_sec={total_sec:.2f}"
                )
        finally:
            self._request_lock.release()

    def _start_stdin_listener(self):
        def _reader():
            while not self._stdin_stop.is_set():
                line = sys.stdin.readline()
                if not line:
                    break
                text = line.strip()
                if not text:
                    continue
                self._process_text(text)
        self._stdin_thread = threading.Thread(target=_reader, daemon=True)
        self._stdin_thread.start()

    def stop_stdin_listener(self):
        if not self.enable_stdin:
            return
        self._stdin_stop.set()
        if self._vosk_thread:
            self._vosk_stop.set()

    def start_asr(self):
        if self._asr_backend == "vosk":
            self._vosk_enabled.set()
            return True
        api_id = int(self.config.get("lui_asr_start_api_id") or DEFAULT_ASR_START_API_ID)
        return call_api(self.node, self.client, api_id, "")

    def stop_asr(self):
        if self._asr_backend == "vosk":
            self._vosk_enabled.clear()
            return True
        api_id = int(self.config.get("lui_asr_stop_api_id") or DEFAULT_ASR_STOP_API_ID)
        return call_api(self.node, self.client, api_id, "")

    def start_tts(self):
        if self.use_ai_speak:
            self.node.get_logger().info("Using kSpeak; deferring AI chat start to utterance.")
            return True
        api_id = int(self.config.get("tts_start_api_id") or DEFAULT_TTS_START_API_ID)
        voice_type = ""
        tts_config = self.config.get("tts_config") or {}
        if isinstance(tts_config, dict):
            voice_type = tts_config.get("voice_type") or ""
        body = {"voice_type": voice_type} if voice_type else {}
        self.node.get_logger().info(f"Starting TTS (voice_type={voice_type or 'default'})")
        return call_api(self.node, self.client, api_id, json.dumps(body))

    def stop_tts(self):
        if self.use_ai_speak:
            self.node.get_logger().info("Using kSpeak; no persistent AI chat to stop.")
            return True
        api_id = int(self.config.get("tts_stop_api_id") or DEFAULT_TTS_STOP_API_ID)
        return call_api(self.node, self.client, api_id, "")

    def send_tts_text(self, text):
        preview = text[:80] + ("..." if len(text) > 80 else "")
        if self.use_ai_speak:
            api_id = DEFAULT_SPEAK_API_ID
            self.node.get_logger().info(
                f"Sending Speak text ({len(text)} chars): {preview}"
            )
            t_ai_start = time.monotonic()
            if not self._start_ai_chat():
                return False
            t_ai_ready = time.monotonic()
            ok = call_api(self.node, self.rtc_client, api_id, json.dumps({"msg": text}))
            t_speak_done = time.monotonic()
            self.node.get_logger().info(
                "Timing: ai_chat_start_sec={:.2f} speak_rpc_sec={:.2f}".format(
                    max(t_ai_ready - t_ai_start, 0.0),
                    max(t_speak_done - t_ai_ready, 0.0),
                )
            )
            delay_mode = str(self.config.get("ai_chat_stop_delay_mode", "auto")).lower()
            if delay_mode == "fixed":
                stop_delay = float(self.config.get("ai_chat_stop_delay_sec", 2.0))
            else:
                stop_delay = estimate_tts_delay(text, self.config)
            if stop_delay > 0:
                self.node.get_logger().info(
                    f"Delaying AI chat stop for {stop_delay:.2f}s to allow playback."
                )
                time.sleep(stop_delay)
            if not bool(self.config.get("ai_chat_keep_alive", False)):
                t_stop_start = time.monotonic()
                self._stop_ai_chat()
                t_stop_done = time.monotonic()
                self.node.get_logger().info(
                    "Timing: ai_chat_stop_sec={:.2f}".format(
                        max(t_stop_done - t_stop_start, 0.0)
                    )
                )
            return ok
        api_id = int(self.config.get("tts_send_api_id") or DEFAULT_TTS_SEND_API_ID)
        self.node.get_logger().info(f"Sending TTS text ({len(text)} chars): {preview}")
        body = {"text": text}
        return call_api(self.node, self.client, api_id, json.dumps(body))

    def _start_ai_chat(self):
        if not self.rtc_client:
            self.node.get_logger().error("RTC client not available for AI chat.")
            return False
        tts_config = self.config.get("tts_config") or {}
        if not isinstance(tts_config, dict):
            tts_config = {}
        voice_type = tts_config.get("voice_type") or ""
        ignore_brackets = tts_config.get("ignore_bracket_text") or []
        if not isinstance(ignore_brackets, list):
            ignore_brackets = []
        payload = {
            "interrupt_mode": False,
            "enable_face_tracking": False,
            "tts_config": {
                "voice_type": voice_type,
                "ignore_bracket_text": ignore_brackets,
            },
            "llm_config": {
                "system_prompt": self.config.get("ai_chat_system_prompt", ""),
                "welcome_msg": self.config.get("ai_chat_welcome_msg", ""),
                "prompt_name": self.config.get("ai_chat_prompt_name", ""),
            },
            "asr_config": {
                "interrupt_speech_duration": int(
                    self.config.get("ai_chat_interrupt_ms", 0)
                ),
                "interrupt_keywords": self.config.get(
                    "ai_chat_interrupt_keywords", []
                ),
            },
        }
        return call_api(
            self.node,
            self.rtc_client,
            DEFAULT_AI_CHAT_START_API_ID,
            json.dumps(payload),
        )

    def _stop_ai_chat(self):
        if not self.rtc_client:
            self.node.get_logger().error("RTC client not available for AI chat.")
            return False
        return call_api(self.node, self.rtc_client, DEFAULT_AI_CHAT_STOP_API_ID, "")


def parse_args():
    parser = argparse.ArgumentParser(description="LUI ASR -> RAGFlow -> LUI TTS")
    default_config = Path(__file__).resolve().parents[3] / "AI_chat_POC" / "config" / "config.json"
    parser.add_argument("--config", default=str(default_config), help="Path to LLM config JSON.")
    parser.add_argument("--wake-word", default="Hey Robo", help="Wake word for ASR.")
    parser.add_argument("--silence-sec", type=float, default=1.0, help="Silence window to finalize input.")
    parser.add_argument("--no-lui-daemon", action="store_true", help="Do not start the LUI daemon.")
    parser.add_argument("--no-print-asr", action="store_true", help="Disable printing ASR text.")
    parser.add_argument("--no-stdin", action="store_true", help="Disable terminal text input.")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("lui_ragflow_chat")
    config = load_config(args.config, node=node)
    load_action_and_song_lists(args.config, config)

    joystick_publisher = None
    if RemoteControllerState:
        joystick_topic = config.get("joystick_topic", "/remote_controller_state")
        joystick_publisher = node.create_publisher(RemoteControllerState, joystick_topic, 10)

    lui_proc = None
    if not args.no_lui_daemon:
        lui_proc = start_lui_daemon(node, config)

    server = LuiRagflowChat(
        node,
        config,
        args.wake_word,
        args.silence_sec,
        print_asr=not args.no_print_asr,
        enable_stdin=not args.no_stdin,
        joystick_publisher=joystick_publisher,
    )
    server.start_tts()
    server.start_asr()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop_asr()
        server.stop_tts()
        server.stop_stdin_listener()
        if lui_proc and lui_proc.poll() is None:
            lui_proc.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
