import json
import os
import shlex
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path


def _log(node, level, message):
    if node:
        logger = node.get_logger()
        if level == "warn":
            logger.warn(message)
        else:
            getattr(logger, level)(message)
    else:
        print(message)


def _get_tts_api_key(config):
    return (
        (config.get("tts_api_key") or config.get("openai_api_key") or config.get("api_key") or "")
        .strip()
    )


def _build_tts_request(text, config):
    api_key = _get_tts_api_key(config)
    if not api_key:
        return None, "tts api_key is not configured"

    api_base = (config.get("tts_api_base") or config.get("api_base") or "https://api.openai.com").rstrip("/")
    path = config.get("openai_tts_path", "/v1/audio/speech")
    endpoint = f"{api_base}{path}"

    headers = {"Content-Type": "application/json"}
    headers.update(config.get("tts_headers") or config.get("headers", {}))
    auth_header = config.get("tts_auth_header") or config.get("auth_header", "Authorization")
    auth_prefix = config.get("tts_auth_prefix") or config.get("auth_prefix", "Bearer ")
    headers[auth_header] = f"{auth_prefix}{api_key}"

    body = dict(config.get("tts_extra_body", {}))
    body.update(
        {
            "model": config.get("tts_model", "tts-1"),
            "voice": config.get("tts_voice", "alloy"),
            "input": text,
            "response_format": config.get("tts_response_format", "wav"),
        }
    )
    speed = config.get("tts_speed")
    if speed is not None:
        body["speed"] = float(speed)

    body_bytes = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(endpoint, data=body_bytes, headers=headers, method="POST")
    return req, None


def request_openai_tts(text, config, node=None):
    req, err = _build_tts_request(text, config)
    if err:
        return None, err

    timeout = float(config.get("tts_timeout_sec") or config.get("request_timeout_sec", 120))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = resp.read()
        if not data:
            return None, "TTS returned empty audio"
        return data, None
    except urllib.error.HTTPError as exc:
        return None, f"HTTP error {exc.code}: {exc.reason}"
    except urllib.error.URLError as exc:
        return None, f"URL error: {exc.reason}"


def _tts_extension(config):
    fmt = (config.get("tts_response_format") or "wav").strip().lower()
    if not fmt:
        return "wav"
    if fmt in ("mp3", "wav", "opus", "aac", "flac", "pcm"):
        return fmt
    return "wav"


def _build_output_path(config):
    output_dir = (config.get("tts_output_dir") or "").strip()
    suffix = f".{_tts_extension(config)}"
    if output_dir:
        out_path = Path(os.path.expanduser(output_dir))
        out_path.mkdir(parents=True, exist_ok=True)
        name = f"tts_{int(time.time() * 1000)}_{os.getpid()}{suffix}"
        return out_path / name
    temp = tempfile.NamedTemporaryFile(delete=False, suffix=suffix)
    temp.close()
    return Path(temp.name)


def _build_play_command(audio_path, config):
    cmd = (config.get("tts_play_command") or "aplay").strip()
    if not cmd:
        return None
    args = shlex.split(cmd)
    if any("{path}" in arg for arg in args):
        return [arg.replace("{path}", audio_path) for arg in args]
    return args + [audio_path]


def play_audio(audio_path, config, node=None):
    args = _build_play_command(audio_path, config)
    if not args:
        _log(node, "warn", "TTS playback disabled (tts_play_command is empty).")
        return False
    try:
        result = subprocess.run(args, check=False)
    except FileNotFoundError:
        _log(node, "error", f"TTS player not found: {args[0]}")
        return False
    if result.returncode != 0:
        _log(node, "warn", f"TTS player exited with code {result.returncode}")
        return False
    return True


def speak_openai_tts(text, config, node=None):
    if not text:
        return False
    audio, err = request_openai_tts(text, config, node=node)
    if err:
        _log(node, "error", f"OpenAI TTS error: {err}")
        return False

    audio_path = _build_output_path(config)
    try:
        with open(audio_path, "wb") as handle:
            handle.write(audio)
        return play_audio(str(audio_path), config, node=node)
    finally:
        if not bool(config.get("tts_keep_audio", False)):
            try:
                os.remove(audio_path)
            except OSError:
                _log(node, "warn", f"Failed to remove TTS audio: {audio_path}")
