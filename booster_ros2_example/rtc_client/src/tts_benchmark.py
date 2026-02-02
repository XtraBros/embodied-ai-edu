import argparse
import json
import os
import statistics
import time
import urllib.error
import urllib.request


def _build_request(endpoint, payload, api_key=None, headers=None, auth_header="Authorization", auth_prefix="Bearer "):
    merged_headers = {"Content-Type": "application/json"}
    if headers:
        merged_headers.update(headers)
    if api_key:
        merged_headers[auth_header] = f"{auth_prefix}{api_key}"
    body = json.dumps(payload).encode("utf-8")
    return urllib.request.Request(endpoint, data=body, headers=merged_headers, method="POST")


def _post_tts(endpoint, payload, api_key=None, headers=None, auth_header="Authorization", auth_prefix="Bearer "):
    req = _build_request(
        endpoint,
        payload,
        api_key=api_key,
        headers=headers,
        auth_header=auth_header,
        auth_prefix=auth_prefix,
    )
    start = time.perf_counter()
    with urllib.request.urlopen(req, timeout=120) as resp:
        data = resp.read()
    elapsed = time.perf_counter() - start
    return elapsed, len(data or b"")


def _run_trials(label, endpoint, payload, api_key=None, headers=None, auth_header="Authorization", auth_prefix="Bearer ", runs=5, warmup=1):
    durations = []
    sizes = []
    for _ in range(warmup):
        _post_tts(endpoint, payload, api_key=api_key, headers=headers, auth_header=auth_header, auth_prefix=auth_prefix)
    for _ in range(runs):
        elapsed, size = _post_tts(endpoint, payload, api_key=api_key, headers=headers, auth_header=auth_header, auth_prefix=auth_prefix)
        durations.append(elapsed)
        sizes.append(size)
        print(f"{label}: {elapsed:.3f}s ({size} bytes)")
    return durations, sizes


def _print_stats(label, durations):
    if not durations:
        return
    avg = statistics.mean(durations)
    med = statistics.median(durations)
    p90 = statistics.quantiles(durations, n=10)[-1] if len(durations) >= 2 else durations[0]
    print(
        f"{label} summary: avg={avg:.3f}s median={med:.3f}s p90={p90:.3f}s min={min(durations):.3f}s max={max(durations):.3f}s"
    )


def _load_config(path):
    if not path:
        return {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        raise SystemExit(f"Config not found: {path}")
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid JSON in config: {path} ({exc})")


def main():
    parser = argparse.ArgumentParser(description="Benchmark TTS request latency.")
    parser.add_argument("--text", default="Hello from Booster Robotics.")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--response-format", default="wav")
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--stream", action="store_true", help="Enable streaming if supported by server.")
    parser.add_argument(
        "--config",
        default="booster_robotics_sdk_ros2/booster_ros2_example/rtc_client/config/ai_chat_web.json",
        help="Path to config JSON with API keys/defaults.",
    )

    parser.add_argument("--local-api-base", default=None)
    parser.add_argument("--local-api-key", default=None)
    parser.add_argument("--local-model", default=None)
    parser.add_argument("--local-voice", default=None)
    parser.add_argument("--local-auth-header", default=None)
    parser.add_argument("--local-auth-prefix", default=None)
    parser.add_argument("--local-path", default="/v1/audio/speech")

    parser.add_argument("--openai-api-key", default=None)
    parser.add_argument("--openai-api-base", default=None)
    parser.add_argument("--openai-model", default=None)
    parser.add_argument("--openai-voice", default=None)
    parser.add_argument("--openai-path", default="/v1/audio/speech")
    args = parser.parse_args()

    config = _load_config(args.config)

    local_api_base = (
        args.local_api_base
        or config.get("local_tts_api_base")
        or config.get("tts_api_base")
        or os.environ.get("LOCAL_TTS_API_BASE", "")
    )
    local_api_key = (
        args.local_api_key
        or config.get("local_tts_api_key")
        or config.get("tts_api_key")
        or os.environ.get("LOCAL_TTS_API_KEY", "")
    )
    local_model = (
        args.local_model
        or config.get("local_tts_model")
        or config.get("tts_model")
        or os.environ.get("LOCAL_TTS_MODEL", "kokoro")
    )
    local_voice = (
        args.local_voice
        or config.get("local_tts_voice")
        or config.get("tts_voice")
        or os.environ.get("LOCAL_TTS_VOICE", "af_heart")
    )
    local_auth_header = (
        args.local_auth_header
        or config.get("local_tts_auth_header")
        or config.get("tts_auth_header")
        or os.environ.get("LOCAL_TTS_AUTH_HEADER", "Authorization")
    )
    local_auth_prefix = (
        args.local_auth_prefix
        or config.get("local_tts_auth_prefix")
        or config.get("tts_auth_prefix")
        or os.environ.get("LOCAL_TTS_AUTH_PREFIX", "Bearer ")
    )

    openai_api_key = (
        args.openai_api_key
        or config.get("openai_api_key")
        or config.get("tts_api_key")
        or os.environ.get("OPENAI_API_KEY", "")
    )
    openai_api_base = (
        args.openai_api_base
        or config.get("openai_api_base")
        or "https://api.openai.com"
    )
    openai_model = (
        args.openai_model
        or config.get("openai_tts_model")
        or config.get("tts_model")
        or os.environ.get("OPENAI_TTS_MODEL", "tts-1")
    )
    openai_voice = (
        args.openai_voice
        or config.get("openai_tts_voice")
        or config.get("tts_voice")
        or os.environ.get("OPENAI_TTS_VOICE", "alloy")
    )

    if not local_api_base:
        raise SystemExit("--local-api-base is required (or set local_tts_api_base/tts_api_base).")
    if not openai_api_key:
        raise SystemExit("--openai-api-key is required (or set openai_api_key).")

    local_endpoint = f"{local_api_base.rstrip('/')}{args.local_path}"
    openai_endpoint = f"{openai_api_base.rstrip('/')}{args.openai_path}"

    local_payload = {
        "model": local_model,
        "voice": local_voice,
        "input": args.text,
        "response_format": args.response_format,
        "stream": bool(args.stream),
    }
    if args.speed is not None:
        local_payload["speed"] = float(args.speed)

    openai_payload = {
        "model": openai_model,
        "voice": openai_voice,
        "input": args.text,
        "response_format": args.response_format,
    }
    if args.speed is not None:
        openai_payload["speed"] = float(args.speed)

    print("Running local TTS benchmark...")
    local_times, _ = _run_trials(
        "local",
        local_endpoint,
        local_payload,
        api_key=local_api_key or None,
        auth_header=local_auth_header,
        auth_prefix=local_auth_prefix,
        runs=args.runs,
        warmup=args.warmup,
    )
    _print_stats("local", local_times)

    print("\nRunning OpenAI TTS benchmark...")
    openai_times, _ = _run_trials(
        "openai",
        openai_endpoint,
        openai_payload,
        api_key=openai_api_key,
        runs=args.runs,
        warmup=args.warmup,
    )
    _print_stats("openai", openai_times)


if __name__ == "__main__":
    main()
