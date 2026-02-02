import argparse
import json
import os
import re
import subprocess
import threading
import time
import sys
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from booster_interface.msg import BoosterApiReqMsg, RemoteControllerState
try:
    from booster_interface.msg import Subtitle
except ImportError:
    Subtitle = None
from booster_interface.srv import RpcService

from llm_web_client import load_config, request_llm, request_llm_stream
from stt_openai import run_stt_openai
from tts_openai import speak_openai_tts


WELCOME_MSG = "Welcome! It's great to see you"


def make_msg(api_id, body):
    msg = BoosterApiReqMsg()
    msg.api_id = int(api_id)
    msg.body = body
    return msg


def call_api(node, client, api_id, body, timeout_sec=5.0):
    node.get_logger().info(f"RPC api_id={api_id} body={body}")
    req = RpcService.Request()
    req.msg = make_msg(api_id, body)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f"RpcService call timed out (api_id={api_id})")
        return False
    response = future.result().msg
    if response.status != 0:
        detail = getattr(response, "body", "")
        if detail:
            node.get_logger().warn(f"RpcService returned status={response.status} body={detail}")
        else:
            node.get_logger().warn(f"RpcService returned status={response.status}")
    return response.status == 0


def start_ai_chat(node, client):
    param = {
        "interrupt_mode": False,
        "asr_config": {
            "interrupt_speech_duration": 200,
            "interrupt_keywords": [],
        },
        "llm_config": {
            "welcome_msg": WELCOME_MSG,
            "prompt_name": "",
        },
        "tts_config": {
            "voice_type": "zh_female_shuangkuaisisi_emo_v2_mars_bigtts",
            "ignore_bracket_text": [3],
        },
        "enable_face_tracking": False,
    }
    body = json.dumps(param)
    return call_api(node, client, 2000, body)


def stop_ai_chat(node, client):
    return call_api(node, client, 2001, "")


def start_face_tracking(node, client):
    return call_api(node, client, 2003, "")


def stop_face_tracking(node, client):
    return call_api(node, client, 2004, "")


def speak(node, client, text):
    body = json.dumps({"msg": text})
    return call_api(node, client, 2002, body)


def speak_with_backend(node, client, text, config):
    if not text:
        return False
    text = sanitize_tts_text(text)
    if not text:
        return False
    backend = (config.get("tts_backend") or config.get("tts_provider") or "robot").lower()
    if backend in ("openai", "openai_tts", "openai-tts"):
        ok = speak_openai_tts(text, config, node=node)
        if ok:
            return True
        node.get_logger().warn("OpenAI TTS failed; falling back to robot TTS.")
    return speak(node, client, text)


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


SDL_HAT_CENTERED = 0x00
SDL_HAT_UP = 0x01
SDL_HAT_RIGHT = 0x02
SDL_HAT_DOWN = 0x04
SDL_HAT_LEFT = 0x08


HAT_POSITIONS = {
    "HAT_UP": SDL_HAT_UP,
    "HAT_RIGHT": SDL_HAT_RIGHT,
    "HAT_DOWN": SDL_HAT_DOWN,
    "HAT_LEFT": SDL_HAT_LEFT,
    "HAT_RIGHTUP": SDL_HAT_RIGHT | SDL_HAT_UP,
    "HAT_RIGHTDOWN": SDL_HAT_RIGHT | SDL_HAT_DOWN,
    "HAT_LEFTUP": SDL_HAT_LEFT | SDL_HAT_UP,
    "HAT_LEFTDOWN": SDL_HAT_LEFT | SDL_HAT_DOWN,
}


DEFAULT_BUILTIN_COMMANDS = {
    "wave_hand": {"buttons": ["B"]},
    "shake_hand": {"buttons": ["A"]},
    "head_up": {"hat": "HAT_UP"},
    "head_right": {"hat": "HAT_RIGHT"},
    "head_down": {"hat": "HAT_DOWN"},
    "head_left": {"hat": "HAT_LEFT"},
    "head_right_up": {"hat": "HAT_RIGHTUP"},
    "head_right_down": {"hat": "HAT_RIGHTDOWN"},
    "head_left_up": {"hat": "HAT_LEFTUP"},
    "head_left_down": {"hat": "HAT_LEFTDOWN"},
    "open_hand": {"buttons": ["RB", "X"]},
    "close_hand": {"buttons": ["RB", "Y"]},
    "boxing": {"buttons": ["RB", "A"]},
    "push_up": {"buttons": ["LT", "HAT_LEFT"]},
    "lie_down": {"buttons": ["LT", "HAT_DOWN"]},
    "fancy_kick": {"buttons": ["LT", "HAT_UP"]},
    "fall_down_recovery": {"buttons": ["RB", "HAT_UP"]},
    "new_year_dance": {"buttons": ["LB", "A"]},
    "dance_rock_and_roll": {"buttons": ["LB", "HAT_LEFT"]},
    "dance_towards_future": {"buttons": ["LB", "HAT_RIGHT"]},
    "gesture_maneki_neko": {"buttons": ["LB", "HAT_DOWN"]},
    "gesture_pogba": {"buttons": ["LB", "B"]},
    "gesture_ultraman": {"buttons": ["LB", "X"]},
    "gesture_chinese_greet": {"buttons": ["LB", "Y"]},
    "gesture_cheer": {"buttons": ["LB", "HAT_UP"]},
    "rma_goalie_crouch": {"buttons": ["LT", "Y"]},
    "rma_goalie_getup": {"buttons": ["LT", "B"]},
    "stand_kick_ball": {"buttons": ["LB", "X"]},
}

DEFAULT_ACTION_ALIASES = {
    "WaveHand": "wave_hand",
    "ShakeHand": "shake_hand",
    "MoveHeadUp": "head_up",
    "MoveHeadRight": "head_right",
    "MoveHeadDown": "head_down",
    "MoveHeadLeft": "head_left",
    "MoveHeadRightUp": "head_right_up",
    "MoveHeadRightDown": "head_right_down",
    "MoveHeadLeftUp": "head_left_up",
    "MoveHeadLeftDown": "head_left_down",
    "FallDownRecovery": "fall_down_recovery",
    "PushUp": "push_up",
    "OpenHand": "open_hand",
    "CloseHand": "close_hand",
    "NewYearDance": "new_year_dance",
    "Boxing": "boxing",
    "DanceRockAndRoll": "dance_rock_and_roll",
    "DanceTowardsFuture": "dance_towards_future",
    "GestureManekiNeko": "gesture_maneki_neko",
    "GesturePogba": "gesture_pogba",
    "GestureUltraman": "gesture_ultraman",
    "GestureChineseGreet": "gesture_chinese_greet",
    "GestureCheer": "gesture_cheer",
    "LieDown": "lie_down",
    "FancyKick": "fancy_kick",
    "RMAGoalieCrouch": "rma_goalie_crouch",
    "RMAGoalieGetup": "rma_goalie_getup",
    "StandKickBall": "stand_kick_ball",
    "Dance": None,
}

DEFAULT_ORCH_SHORTCUTS = {
    "ChickenDance": {"buttons": ["LT", "X"]},
    "Chicken dance": {"buttons": ["LT", "X"]},
    "Robot Dance": {"buttons": ["RT", "X"]},
    "RobotDance": {"buttons": ["RT", "X"]},

}

BUILTIN_COMMANDS = dict(DEFAULT_BUILTIN_COMMANDS)
ACTION_ALIASES = dict(DEFAULT_ACTION_ALIASES)
ORCH_SHORTCUTS = dict(DEFAULT_ORCH_SHORTCUTS)
SONG_LIBRARY = {}
SONG_LIBRARY_NORMALIZED = {}

MUSIC_STOP_WORDS = {"stop", "stop_song", "stop_music", "stop_playing", "off", "none", "null"}


def set_hat_fields(msg, hat_pos):
    msg.hat_pos = int(hat_pos)
    msg.hat_c = hat_pos == SDL_HAT_CENTERED
    msg.hat_u = bool(hat_pos & SDL_HAT_UP)
    msg.hat_d = bool(hat_pos & SDL_HAT_DOWN)
    msg.hat_l = bool(hat_pos & SDL_HAT_LEFT)
    msg.hat_r = bool(hat_pos & SDL_HAT_RIGHT)
    msg.hat_lu = hat_pos == (SDL_HAT_LEFT | SDL_HAT_UP)
    msg.hat_ld = hat_pos == (SDL_HAT_LEFT | SDL_HAT_DOWN)
    msg.hat_ru = hat_pos == (SDL_HAT_RIGHT | SDL_HAT_UP)
    msg.hat_rd = hat_pos == (SDL_HAT_RIGHT | SDL_HAT_DOWN)


def build_remote_state(buttons=None, hat_key=None, event=0):
    msg = RemoteControllerState()
    msg.event = int(event)
    msg.lx = 0.0
    msg.ly = 0.0
    msg.rx = 0.0
    msg.ry = 0.0
    msg.a = False
    msg.b = False
    msg.x = False
    msg.y = False
    msg.lb = False
    msg.rb = False
    msg.lt = False
    msg.rt = False
    msg.ls = False
    msg.rs = False
    msg.back = False
    msg.start = False

    button_map = {
        "A": "a",
        "B": "b",
        "X": "x",
        "Y": "y",
        "LB": "lb",
        "RB": "rb",
        "LT": "lt",
        "RT": "rt",
        "LS": "ls",
        "RS": "rs",
        "BACK": "back",
        "START": "start",
    }
    if buttons:
        for button in buttons:
            field = button_map.get(button)
            if field:
                setattr(msg, field, True)

    hat_pos = HAT_POSITIONS.get(hat_key, SDL_HAT_CENTERED)
    set_hat_fields(msg, hat_pos)
    return msg


def build_axis_state(lx=0.0, ly=0.0, vyaw=0.0, event=0):
    msg = RemoteControllerState()
    msg.event = int(event)
    msg.lx = float(lx)
    msg.ly = float(ly)
    msg.rx = 0.0
    msg.ry = float(vyaw)
    msg.a = False
    msg.b = False
    msg.x = False
    msg.y = False
    msg.lb = False
    msg.rb = False
    msg.lt = False
    msg.rt = False
    msg.ls = False
    msg.rs = False
    msg.back = False
    msg.start = False
    set_hat_fields(msg, SDL_HAT_CENTERED)
    return msg


def _parse_walk_command(action_cmd):
    if not action_cmd:
        return None
    text = action_cmd.strip().lower()
    if not text.startswith("walk"):
        return None
    steps = None
    match = re.search(r"\b(\d+)\b", text)
    if match:
        steps = int(match.group(1))
    direction = "forward"
    if "backward" in text or "back" in text:
        direction = "backward"
    elif "left" in text:
        direction = "left"
    elif "right" in text:
        direction = "right"
    return direction, steps


def _walk_axes(direction, magnitude=0.6):
    if direction == "backward":
        return 0.0, magnitude, 0.0
    if direction == "left":
        return -magnitude, 0.0, 0.0
    if direction == "right":
        return magnitude, 0.0, 0.0
    return 0.0, -magnitude, 0.0


def schedule_walk(
    node,
    publisher,
    direction,
    steps,
    step_sec,
    magnitude,
    hz=10.0,
    event_press=0,
    event_release=0,
):
    if publisher is None:
        node.get_logger().warn("No joystick publisher available for walk command.")
        return

    def _run():
        duration = max(step_sec * steps, 0.0)
        lx, ly, vyaw = _walk_axes(direction, magnitude=magnitude)
        interval = 1.0 / max(hz, 1.0)
        end_time = time.monotonic() + duration
        node.get_logger().info(
            f"Walking {direction} for {steps} steps ({duration:.2f}s)."
        )
        while time.monotonic() < end_time:
            publisher.publish(build_axis_state(lx=lx, ly=ly, vyaw=vyaw, event=event_press))
            time.sleep(interval)
        publisher.publish(build_axis_state(event=event_release))
        node.get_logger().info("Walk command complete.")

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()


def send_combo(
    node,
    publisher,
    buttons=None,
    hat_key=None,
    event_press=1539,
    event_release=1540,
    hold_sec=0.2,
):
    if publisher is None:
        return
    press_msg = build_remote_state(
        buttons=buttons, hat_key=hat_key, event=event_press
    )
    publisher.publish(press_msg)
    if hold_sec > 0:
        time.sleep(hold_sec)
    release_msg = build_remote_state(event=event_release)
    publisher.publish(release_msg)


def schedule_combo_stop(
    node,
    publisher,
    buttons=None,
    hat_key=None,
    event_press=1539,
    event_release=1540,
    hold_sec=0.2,
    delay_sec=5.0,
):
    def _stop():
        time.sleep(max(0.0, delay_sec))
        send_combo(
            node,
            publisher,
            buttons=buttons,
            hat_key=hat_key,
            event_press=event_press,
            event_release=event_release,
            hold_sec=hold_sec,
        )
        node.get_logger().info("Auto-stop combo sent.")

    thread = threading.Thread(target=_stop, daemon=True)
    thread.start()


def normalize_action(action_cmd):
    if not action_cmd:
        return None
    raw = action_cmd.strip()
    if raw in ACTION_ALIASES:
        return ACTION_ALIASES[raw]
    lowered = raw.lower()
    if lowered in ACTION_ALIASES:
        return ACTION_ALIASES[lowered]
    if lowered in BUILTIN_COMMANDS:
        return lowered
    return raw


def _load_json_optional(path):
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        return None
    except json.JSONDecodeError:
        return None


def load_action_and_song_lists(config_path, config):
    base_dir = Path(config_path).resolve().parent
    song_path = config.get("song_library_path") or str(base_dir / "song_library.json")
    action_path = config.get("action_commands_path") or str(base_dir / "action_commands.json")

    song_data = _load_json_optional(song_path)
    if isinstance(song_data, dict):
        SONG_LIBRARY.update(song_data)
        SONG_LIBRARY_NORMALIZED.clear()
        SONG_LIBRARY_NORMALIZED.update({title.lower(): path for title, path in SONG_LIBRARY.items()})

    action_data = _load_json_optional(action_path)
    if isinstance(action_data, dict):
        builtin = action_data.get("builtin_commands")
        aliases = action_data.get("action_aliases")
        orch = action_data.get("orch_shortcuts")
        if isinstance(builtin, dict):
            BUILTIN_COMMANDS.clear()
            BUILTIN_COMMANDS.update(builtin)
        if isinstance(aliases, dict):
            ACTION_ALIASES.clear()
            ACTION_ALIASES.update(aliases)
        if isinstance(orch, dict):
            ORCH_SHORTCUTS.clear()
            ORCH_SHORTCUTS.update(orch)


def _music_player_path(config):
    script_path = config.get("music_player_script", "").strip()
    if script_path:
        return script_path
    return str(Path(__file__).resolve().parent / "music_player.py")


def handle_song(node, song_value, config):
    if song_value is None:
        return
    if isinstance(song_value, str):
        value = song_value.strip()
    else:
        value = str(song_value).strip()
    if not value:
        return
    lower = value.lower()
    player_script = _music_player_path(config)
    if lower in MUSIC_STOP_WORDS:
        node.get_logger().info("Stopping music playback.")
        subprocess.run([sys.executable, player_script, "stop"], check=False)
        return
    path = SONG_LIBRARY.get(value) or SONG_LIBRARY_NORMALIZED.get(lower)
    if not path:
        node.get_logger().warn(f"Unknown song requested: {value}")
        return
    node.get_logger().info(f"Playing song: {value}")
    subprocess.run([sys.executable, player_script, "play", path], check=False)


def extract_payload_from_text(text):
    if not isinstance(text, str):
        return None
    cleaned = text.strip()
    if not cleaned:
        return None
    if cleaned.startswith("```"):
        cleaned = cleaned.strip("`").strip()
        if cleaned.lower().startswith("json"):
            cleaned = cleaned[4:].strip()
    try:
        payload = json.loads(cleaned)
    except json.JSONDecodeError:
        payload = None
        if "{" in cleaned and "}" in cleaned:
            snippet = cleaned[cleaned.find("{") : cleaned.rfind("}") + 1]
            try:
                payload = json.loads(snippet)
            except json.JSONDecodeError:
                payload = None
    return payload if isinstance(payload, dict) else None


def handle_action(node, action_cmd, config, joystick_publisher=None):
    if not action_cmd:
        return
    walk_cmd = _parse_walk_command(action_cmd)
    if walk_cmd:
        direction, steps = walk_cmd
        steps = 1
        step_sec = float(config.get("walk_step_sec", 0.6))
        magnitude = float(config.get("walk_magnitude", 0.6))
        event_press = int(config.get("joystick_event_axis", 1536))
        event_release = int(config.get("joystick_event_axis_release", 1536))
        schedule_walk(
            node,
            joystick_publisher,
            direction=direction,
            steps=steps,
            step_sec=step_sec,
            magnitude=magnitude,
            event_press=event_press,
            event_release=event_release,
        )
        return
    if action_cmd in ORCH_SHORTCUTS:
        shortcut = ORCH_SHORTCUTS[action_cmd]
        node.get_logger().info(f"Trigger orchestration shortcut: {action_cmd}")
        send_combo(
            node,
            joystick_publisher,
            buttons=shortcut.get("buttons"),
            hat_key=shortcut.get("hat"),
            event_press=config.get("joystick_event_press", 1539),
            event_release=config.get("joystick_event_release", 1540),
            hold_sec=config.get("joystick_hold_sec", 0.2),
        )
        schedule_combo_stop(
            node,
            joystick_publisher,
            buttons=shortcut.get("buttons"),
            hat_key=shortcut.get("hat"),
            event_press=config.get("joystick_event_press", 1539),
            event_release=config.get("joystick_event_release", 1540),
            hold_sec=config.get("joystick_hold_sec", 0.2),
            delay_sec=config.get("auto_stop_sec", 5.0),
        )
        return
    action_key = normalize_action(action_cmd)
    enable_builtin = bool(config.get("enable_builtin_commands", True))
    if action_key is None:
        node.get_logger().warn(f"Unsupported built-in command: {action_cmd}")
        return
    if enable_builtin and action_key in BUILTIN_COMMANDS:
        combo = BUILTIN_COMMANDS[action_key]
        node.get_logger().info(f"Trigger built-in command: {action_cmd}")
        send_combo(
            node,
            joystick_publisher,
            buttons=combo.get("buttons"),
            hat_key=combo.get("hat"),
            event_press=config.get("joystick_event_press", 1539),
            event_release=config.get("joystick_event_release", 1540),
            hold_sec=config.get("joystick_hold_sec", 0.2),
        )
        schedule_combo_stop(
            node,
            joystick_publisher,
            buttons=combo.get("buttons"),
            hat_key=combo.get("hat"),
            event_press=config.get("joystick_event_press", 1539),
            event_release=config.get("joystick_event_release", 1540),
            hold_sec=config.get("joystick_hold_sec", 0.2),
            delay_sec=config.get("auto_stop_sec", 5.0),
        )
        return

    interface = config.get("loco_interface", "").strip()
    runner = config.get("loco_action_runner", "").strip()
    if not interface or not runner:
        print(f"Action placeholder: {action_cmd}")
        return
    print(f"Trigger action: {action_cmd}")
    command = [runner, interface, action_cmd]
    if runner.endswith(".py") and not os.access(runner, os.X_OK):
        command = [sys.executable] + command
    try:
        result = subprocess.run(command, check=False)
        print(f"Action runner exit code: {result.returncode}")
        print("Auto-stop not supported for runner actions.")
    except FileNotFoundError:
        print(f"Action runner not found: {runner}")


def parse_response_action(raw_text):
    if not isinstance(raw_text, str):
        return "", ""
    text = raw_text.strip()
    if not text:
        return "", ""
    response_value = ""
    action_value = ""
    if '"response"' in text and '"action"' in text:
        resp_idx = text.find('"response"')
        act_idx = text.find('"action"')
        if resp_idx != -1 and act_idx != -1 and act_idx > resp_idx:
            between = text[resp_idx:act_idx]
            if ":" in between:
                response_value = between.split(":", 1)[1].strip()
            after = text[act_idx:]
            if ":" in after:
                action_value = after.split(":", 1)[1].strip().rstrip("}").strip()
    response_value = response_value.strip().strip(",")
    action_value = action_value.strip().strip(",")
    response_value = response_value.strip().strip('"')
    action_value = action_value.strip().strip('"')
    if response_value.lower() == "null":
        response_value = ""
    if action_value.lower() == "null":
        action_value = ""
    return response_value, action_value


class ResponseJsonStreamExtractor:
    def __init__(self, field_name="response"):
        self.field_name = field_name
        self._search = ""
        self._state = "search"
        self._escape = False
        self._unicode_remaining = 0
        self._unicode_buffer = ""
        self._done = False

    def feed(self, chunk):
        if self._done or not chunk:
            return ""
        out = []
        for ch in chunk:
            if self._state == "search":
                self._search += ch
                if len(self._search) > 64:
                    self._search = self._search[-64:]
                needle = f"\"{self.field_name}\""
                if needle in self._search:
                    self._state = "seek_colon"
            elif self._state == "seek_colon":
                if ch == ":":
                    self._state = "seek_quote"
            elif self._state == "seek_quote":
                if ch == "\"":
                    self._state = "capture"
            elif self._state == "capture":
                if self._unicode_remaining > 0:
                    if ch.lower() in "0123456789abcdef":
                        self._unicode_buffer += ch
                        self._unicode_remaining -= 1
                        if self._unicode_remaining == 0:
                            try:
                                out.append(chr(int(self._unicode_buffer, 16)))
                            except ValueError:
                                out.append("\\u" + self._unicode_buffer)
                            self._unicode_buffer = ""
                    else:
                        out.append("\\u" + self._unicode_buffer + ch)
                        self._unicode_buffer = ""
                        self._unicode_remaining = 0
                elif self._escape:
                    if ch == "u":
                        self._unicode_remaining = 4
                        self._unicode_buffer = ""
                    else:
                        out.append(self._unescape_char(ch))
                    self._escape = False
                elif ch == "\\":
                    self._escape = True
                elif ch == "\"":
                    self._done = True
                    self._state = "done"
                else:
                    out.append(ch)
        return "".join(out)

    @staticmethod
    def _unescape_char(ch):
        if ch == "n":
            return "\n"
        if ch == "t":
            return "\t"
        if ch == "r":
            return "\r"
        if ch == "\"":
            return "\""
        if ch == "\\":
            return "\\"
        return ch


SENTENCE_ENDINGS = ".!?\u3002\uff01\uff1f"


def _split_for_speech(text, min_chars, max_chars):
    if len(text) < min_chars:
        return None, text
    slice_text = text[:max_chars]
    last_idx = -1
    for ch in SENTENCE_ENDINGS:
        idx = slice_text.rfind(ch)
        if idx > last_idx:
            last_idx = idx
    if last_idx != -1 and last_idx + 1 >= min_chars:
        return slice_text[: last_idx + 1], text[last_idx + 1 :]
    if len(text) >= max_chars:
        return slice_text, text[max_chars:]
    return None, text


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
    if not model_path:
        node.get_logger().error("ASR model path not configured (asr_model_path).")
        return ""

    sample_rate = int(config.get("asr_sample_rate", 16000))
    chunk_ms = float(config.get("asr_chunk_ms", 200))
    max_sec = float(config.get("asr_max_sec", 12.0))
    chunk_size = int(sample_rate * 2 * (chunk_ms / 1000.0))

    node.get_logger().info("ASR: starting microphone capture...")
    model = Model(model_path)
    recognizer = KaldiRecognizer(model, sample_rate)

    command = ["arecord", "-q", "-f", "S16_LE", "-r", str(sample_rate), "-c", "1"]
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    except FileNotFoundError:
        node.get_logger().error("ASR: 'arecord' not found. Install ALSA utilities.")
        return ""
    start_time = time.time()
    final_text = ""
    try:
        for data in _read_process_bytes(proc, chunk_size):
            if time.time() - start_time > max_sec:
                node.get_logger().info("ASR: max duration reached, stopping.")
                break
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result() or "{}")
                final_text = (result.get("text") or "").strip()
                if final_text:
                    break
            else:
                partial = json.loads(recognizer.PartialResult() or "{}").get("partial", "")
                if partial:
                    node.get_logger().info(f"ASR (partial): {partial}")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    if final_text:
        node.get_logger().info(f"ASR (final): {final_text}")
    else:
        node.get_logger().info("ASR: no speech detected.")
    return final_text


def run_asr(config, node):
    backend = (config.get("stt_backend") or config.get("stt_provider") or "vosk").lower()
    if backend in ("openai", "openai_stt", "openai-stt", "whisper"):
        return run_stt_openai(config, node=node)
    return run_asr_vosk(config, node)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ai-chat",
        action="store_true",
        help="Enable StartAiChat (ASR/LLM/TTS pipeline).",
    )
    parser.add_argument(
        "--input-mode",
        choices=("text", "asr"),
        default="text",
        help="Use typed text or ASR microphone input.",
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent.parent / "config" / "ai_chat_web.json"),
        help="Path to JSON config containing LLM endpoint settings.",
    )
    parser.add_argument(
        "--no-tts",
        action="store_true",
        help="Disable TTS output (print response only).",
    )
    args, ros_args = parser.parse_known_args()

    config = load_config(args.config)
    load_action_and_song_lists(args.config, config)
    rclpy.init(args=ros_args)
    node = Node("llm_web_test")
    client = node.create_client(RpcService, "booster_rtc_service")

    joystick_topic = config.get("joystick_topic", "/remote_controller_state")
    joystick_publisher = node.create_publisher(
        RemoteControllerState, joystick_topic, 10
    )

    subtitle_topic = node.declare_parameter("subtitle_topic", "/ai_subtitle").get_parameter_value().string_value
    enable_face_tracking = node.declare_parameter("enable_face_tracking", False).get_parameter_value().bool_value

    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            return 1
        node.get_logger().info("booster_rtc_service not available, waiting...")

    def on_subtitle(msg):
        if msg.text:
            print(f"AI: {msg.text}")

    if Subtitle is None:
        node.get_logger().warn("Subtitle message type unavailable; skipping /ai_subtitle subscription.")
    else:
        node.create_subscription(Subtitle, subtitle_topic, on_subtitle, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    enable_ai_chat = args.ai_chat
    enable_tts = bool(config.get("enable_tts", True)) and not args.no_tts
    if enable_ai_chat:
        start_ai_chat(node, client)
        if enable_face_tracking:
            start_face_tracking(node, client)
        if config.get("enter_booster_agent_mode"):
            node.get_logger().info("Entering BoosterAgentMode via joystick script.")
            script_path = config.get("joystick_script", "").strip()
            if not script_path:
                script_path = str(Path(__file__).resolve().parent / "keyboard_joystick.py")
            command = [
                sys.executable,
                script_path,
                "--topic",
                joystick_topic,
                "--event-press",
                str(config.get("joystick_event_press", 1539)),
                "--event-release",
                str(config.get("joystick_event_release", 1540)),
                "--combo",
                "LT+RT+A",
                "--combo-hold",
                str(config.get("joystick_hold_sec", 0.2)),
            ]
            result = subprocess.run(command, check=False)
            node.get_logger().info(f"Joystick combo exit code: {result.returncode}")

    if enable_ai_chat:
        print("LLM mode with AI chat pipeline. Type text to send, or /quit to exit.")
    else:
        print("LLM mode (RAGFlow + Speak). Type text to send, or /quit to exit.")

    stream_llm = bool(config.get("stream_llm", False))
    ignore_action = bool(config.get("ignore_action", False))
    response_field = config.get("response_field", "response")
    stream_response_json = bool(config.get("stream_response_json", True))
    stream_min_chars = int(config.get("stream_chunk_min_chars", 40))
    stream_max_chars = int(config.get("stream_chunk_max_chars", 160))
    if response_field and not stream_response_json:
        node.get_logger().warn(
            "stream_response_json disabled while response_field is set; forcing response-only streaming."
        )
        stream_response_json = True

    try:
        while True:
            if args.input_mode == "asr":
                line = run_asr(config, node)
                if not line:
                    continue
            else:
                line = input().strip()
                if line == "/quit":
                    break
                if not line:
                    continue

            if stream_llm:
                raw_text = ""
                speech_text = ""
                action_cmd = ""
                pending = ""
                extractor = (
                    ResponseJsonStreamExtractor(response_field)
                    if stream_response_json
                    else None
                )
                try:
                    for delta in request_llm_stream(line, config):
                        raw_text += delta
                        new_text = extractor.feed(delta) if extractor else delta
                        if not new_text:
                            continue
                        speech_text += new_text
                        pending += new_text
                        while True:
                            chunk, pending = _split_for_speech(
                                pending, stream_min_chars, stream_max_chars
                            )
                            if chunk is None:
                                break
                            chunk = chunk.strip()
                            if chunk:
                                if enable_tts:
                                    speak_with_backend(node, client, chunk, config)
                except RuntimeError as exc:
                    print(f"LLM error: {exc}")
                    continue

                trailing = pending.strip()
                if trailing and enable_tts:
                    speak_with_backend(node, client, trailing, config)
                payload = extract_payload_from_text(raw_text)
                song_value = payload.get("song") if isinstance(payload, dict) else None
                if not ignore_action and isinstance(payload, dict):
                    action_cmd = payload.get("action", "") or ""
                if not ignore_action and not action_cmd:
                    _, action_cmd = parse_response_action(raw_text)
                print(speech_text)
                handle_song(node, song_value, config)
                if not song_value and isinstance(action_cmd, str) and action_cmd.strip().lower() in MUSIC_STOP_WORDS:
                    handle_song(node, "stop", config)
                if not ignore_action:
                    handle_action(node, action_cmd, config, joystick_publisher)
                continue

            response, err = request_llm(line, config)
            if err:
                print(f"LLM error: {err}")
                continue
            print(f"LLM raw response: {response}")
            speech_text = response
            action_cmd = ""
            payload = None
            if isinstance(response, str):
                payload = extract_payload_from_text(response)
                if payload is None and response_field == "response":
                    speech_text, action_cmd = parse_response_action(response)
                    if not speech_text:
                        speech_text = response
            elif isinstance(response, dict):
                payload = response
            song_value = payload.get("song") if isinstance(payload, dict) else None
            if isinstance(payload, dict):
                speech_text = payload.get(response_field, "") or ""
                action_cmd = payload.get("action", "") or ""
                if isinstance(action_cmd, str):
                    action_cmd = action_cmd.strip().strip("`").strip()
                if not speech_text:
                    speech_text = response if isinstance(response, str) else ""
            print(speech_text)
            handle_song(node, song_value, config)
            if not song_value and isinstance(action_cmd, str) and action_cmd.strip().lower() in MUSIC_STOP_WORDS:
                handle_song(node, "stop", config)
            if not ignore_action:
                handle_action(node, action_cmd, config, joystick_publisher)
            if enable_tts:
                speak_with_backend(node, client, speech_text, config)
    except EOFError:
        pass

    if enable_ai_chat:
        if enable_face_tracking:
            stop_face_tracking(node, client)
        stop_ai_chat(node, client)

    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
