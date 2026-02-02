import audioop
import json
import os
import tempfile
import subprocess
import time
import wave
import urllib.error
import urllib.request


def _log(node, level, message):
    prefix = level.upper()
    if node:
        logger = node.get_logger()
        try:
            if level == "warn":
                logger.warn(message)
            elif level == "error":
                logger.error(message)
            else:
                logger.info(message)
        except ValueError:
            print(f"[{prefix}] {message}")
    else:
        print(f"[{prefix}] {message}")


def _get_stt_api_key(config):
    return (
        (config.get("stt_api_key") or config.get("openai_api_key") or config.get("api_key") or "")
        .strip()
    )


def _build_multipart(boundary, fields, file_field):
    crlf = "\r\n"
    parts = []
    for name, value in fields:
        parts.append(f"--{boundary}")
        parts.append(f'Content-Disposition: form-data; name="{name}"')
        parts.append("")
        parts.append(str(value))
    parts.append(f"--{boundary}")
    parts.append(
        f'Content-Disposition: form-data; name="{file_field["name"]}"; '
        f'filename="{file_field["filename"]}"'
    )
    parts.append(f'Content-Type: {file_field["content_type"]}')
    parts.append("")
    preamble = crlf.join(parts).encode("utf-8") + crlf.encode("utf-8")
    closing = f"{crlf}--{boundary}--{crlf}".encode("utf-8")
    return preamble, closing


def _build_stt_request(audio_path, config):
    api_key = _get_stt_api_key(config)
    if not api_key:
        return None, "stt api_key is not configured"

    api_base = (config.get("stt_api_base") or config.get("api_base") or "https://api.openai.com").rstrip("/")
    path = config.get("openai_stt_path", "/v1/audio/transcriptions")
    endpoint = f"{api_base}{path}"

    headers = {}
    headers.update(config.get("stt_headers") or config.get("headers", {}))
    auth_header = config.get("stt_auth_header") or config.get("auth_header", "Authorization")
    auth_prefix = config.get("stt_auth_prefix") or config.get("auth_prefix", "Bearer ")
    headers[auth_header] = f"{auth_prefix}{api_key}"

    boundary = f"----sttBoundary{os.getpid()}"
    headers["Content-Type"] = f"multipart/form-data; boundary={boundary}"

    model = config.get("stt_model", "gpt-4o-mini-transcribe")
    # Force English for ASR intent detection and transcription.
    language = "en"
    prompt = (config.get("stt_prompt") or "").strip()

    filename = os.path.basename(audio_path)
    fields = [("model", model)]
    if language:
        fields.append(("language", language))
    if prompt:
        fields.append(("prompt", prompt))
    preamble, closing = _build_multipart(
        boundary,
        fields,
        {
            "name": "file",
            "filename": filename,
            "content_type": "audio/wav",
        },
    )

    with open(audio_path, "rb") as handle:
        audio_bytes = handle.read()

    body_bytes = preamble + audio_bytes + closing
    req = urllib.request.Request(endpoint, data=body_bytes, headers=headers, method="POST")
    return req, None


def _extract_after_trigger(text, trigger_phrase):
    if not text:
        return ""
    if not trigger_phrase:
        return text
    lowered = text.lower()
    trigger = trigger_phrase.lower()
    idx = lowered.find(trigger)
    if idx == -1:
        return ""
    after = text[idx + len(trigger) :].strip(" ,.!?\t\r\n")
    return after


def request_openai_stt(audio_path, config, node=None):
    req, err = _build_stt_request(audio_path, config)
    if err:
        return None, err

    timeout = float(config.get("stt_timeout_sec") or config.get("request_timeout_sec", 120))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        try:
            detail = exc.read().decode("utf-8", errors="replace")
        except Exception:
            detail = ""
        if detail:
            return None, f"HTTP error {exc.code}: {exc.reason} - {detail}"
        return None, f"HTTP error {exc.code}: {exc.reason}"
    except urllib.error.URLError as exc:
        return None, f"URL error: {exc.reason}"

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return raw.strip(), None

    text = payload.get("text") if isinstance(payload, dict) else None
    if not isinstance(text, str):
        return "", "STT response missing text"
    return text.strip(), None


def _record_with_silence(
    sample_rate,
    max_sec,
    silence_ms,
    rms_threshold,
    node=None,
    log_rms=False,
    min_speech_ms=200,
):
    chunk_ms = 20
    bytes_per_sample = 2
    chunk_size = int(sample_rate * (chunk_ms / 1000.0)) * bytes_per_sample
    silence_chunks = max(1, int(silence_ms / chunk_ms))

    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as handle:
        output_path = handle.name

    command = [
        "arecord",
        "-q",
        "-f",
        "S16_LE",
        "-r",
        str(sample_rate),
        "-c",
        "1",
        "-t",
        "raw",
    ]
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    except FileNotFoundError:
        _log(node, "error", "STT: 'arecord' not found. Install ALSA utilities.")
        return "", ""

    silence_count = 0
    started = False
    start_time = time.time()
    last_log = start_time
    speech_chunks = 0
    frames = []
    try:
        while True:
            if time.time() - start_time > max_sec:
                break
            data = proc.stdout.read(chunk_size)
            if not data:
                break
            frames.append(data)
            rms = audioop.rms(data, bytes_per_sample)
            if log_rms and (time.time() - last_log) >= 0.5:
                _log(node, "info", f"STT RMS: {rms}")
                last_log = time.time()
            if rms >= rms_threshold:
                started = True
                silence_count = 0
                speech_chunks += 1
            elif started:
                silence_count += 1
                if silence_count >= silence_chunks:
                    break
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    speech_ms = speech_chunks * chunk_ms
    if not frames or not started or speech_ms < min_speech_ms:
        try:
            os.remove(output_path)
        except OSError:
            pass
        return "", ""

    with wave.open(output_path, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(bytes_per_sample)
        wav.setframerate(sample_rate)
        wav.writeframes(b"".join(frames))

    return output_path, "ok"


def run_stt_openai(config, node=None):
    sample_rate = int(config.get("stt_sample_rate", 16000))
    max_sec = float(config.get("stt_max_sec", 10.0))
    silence_ms = float(config.get("stt_silence_ms", 500))
    rms_threshold = int(config.get("stt_rms_threshold", 500))
    log_rms = bool(config.get("stt_log_rms", False))
    min_speech_ms = float(config.get("stt_min_speech_ms", 200))
    trigger_phrase = (config.get("stt_trigger_phrase") or "").strip()
    trigger_required = bool(config.get("stt_trigger_required", False))
    trigger_timeout = float(config.get("stt_trigger_timeout_sec", 30.0))
    start_wait = time.time()

    while True:
        _log(node, "info", "STT: listening for speech...")
        print("STT: listening for speech...")
        output_path, status = _record_with_silence(
            sample_rate,
            max_sec,
            silence_ms,
            rms_threshold,
            node=node,
            log_rms=log_rms,
            min_speech_ms=min_speech_ms,
        )
        if not status:
            if trigger_required and (time.time() - start_wait) < trigger_timeout:
                continue
            return ""

        try:
            text, err = request_openai_stt(output_path, config, node=node)
        finally:
            try:
                os.remove(output_path)
            except OSError:
                pass

        if err:
            _log(node, "error", f"OpenAI STT error: {err}")
            return ""
        if text:
            _log(node, "info", f"STT (final): {text}")
        else:
            _log(node, "info", "STT: no speech detected.")
            if trigger_required and (time.time() - start_wait) < trigger_timeout:
                continue
            return ""

        if trigger_required:
            after = _extract_after_trigger(text, trigger_phrase)
            if after:
                return after
            if (time.time() - start_wait) >= trigger_timeout:
                return ""
            _log(node, "info", f"STT: waiting for trigger phrase '{trigger_phrase}'...")
            continue

        return text
