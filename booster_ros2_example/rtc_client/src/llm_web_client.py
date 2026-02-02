import json
import os
import threading
import urllib.error
import urllib.parse
import urllib.request


DEFAULT_CONFIG = {
    "provider": "ragflow",
    "api_base": "",
    "agent_id": "",
    "api_key": "",
    "ragflow_api_key": "",
    "openai_api_key": "",
    "model": "default",
    "agent_completion_path": "/api/v1/agents_openai/{agent_id}/chat/completions",
    "openai_chat_path": "/v1/chat/completions",
    "request_timeout_sec": 120,
    "llm_url": "",
    "http_method": "POST",
    "headers": {"Content-Type": "application/json"},
    "auth_header": "Authorization",
    "auth_prefix": "Bearer ",
    "input_field": "query",
    "extra_body": {},
    "response_text_key": "text",
    "response_text_path": [],
    "command_triggers": [],
    "enable_command_exec": False,
    "system_prompt": "",
}


def load_config(path, node=None):
    config_path = os.path.expanduser(path)
    config = dict(DEFAULT_CONFIG)
    try:
        with open(config_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        config.update(data)
        if "headers" in data:
            merged_headers = dict(DEFAULT_CONFIG["headers"])
            merged_headers.update(data.get("headers", {}))
            config["headers"] = merged_headers
        if "extra_body" in data:
            merged_body = dict(DEFAULT_CONFIG["extra_body"])
            merged_body.update(data.get("extra_body", {}))
            config["extra_body"] = merged_body
    except FileNotFoundError:
        if node:
            node.get_logger().warn(f"Config not found at {config_path}.")
    except json.JSONDecodeError:
        if node:
            node.get_logger().warn(f"Failed to parse JSON config at {config_path}.")
    return config


def _extract_by_path(data, path):
    current = data
    for key in path:
        if isinstance(current, dict):
            current = current.get(key)
        elif isinstance(current, list) and isinstance(key, int) and 0 <= key < len(current):
            current = current[key]
        else:
            return None
    return current


def _parse_ragflow_content(data):
    choices = data.get("choices") if isinstance(data, dict) else None
    first = choices[0] if isinstance(choices, list) and choices else None
    content = ""
    if isinstance(first, dict):
        message = first.get("message") or {}
        content = (
            message.get("content")
            or (first.get("delta") or {}).get("content")
            or ""
        )
    if not content and isinstance(data, dict):
        content = data.get("content") or ""
    if isinstance(content, list):
        content = "\n".join(part.get("text", "") for part in content if isinstance(part, dict))
    return content.strip() if isinstance(content, str) else ""


def _parse_openai_content(data):
    if not isinstance(data, dict):
        return ""
    choices = data.get("choices")
    first = choices[0] if isinstance(choices, list) and choices else None
    if not isinstance(first, dict):
        return ""
    message = first.get("message") or {}
    content = message.get("content") or ""
    return content.strip() if isinstance(content, str) else ""


def _request_ragflow(text, config):
    api_base = (config.get("api_base") or "").rstrip("/")
    agent_id = (config.get("agent_id") or "").strip()
    api_key = (config.get("ragflow_api_key") or config.get("api_key") or "").strip()
    if not api_base:
        return "", "api_base is not configured"
    if not agent_id:
        return "", "agent_id is not configured"
    if not api_key:
        return "", "api_key is not configured"

    path = config.get("agent_completion_path", DEFAULT_CONFIG["agent_completion_path"])
    endpoint = f"{api_base}{path.replace('{agent_id}', agent_id)}"
    headers = dict(config.get("headers", {}))
    auth_header = config.get("auth_header", "Authorization")
    auth_prefix = config.get("auth_prefix", "Bearer ")
    headers[auth_header] = f"{auth_prefix}{api_key}"

    body = {
        "model": config.get("model", "default"),
        "messages": [{"role": "user", "content": text}],
        "stream": False,
    }
    body_bytes = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(endpoint, data=body_bytes, headers=headers, method="POST")
    timeout = float(config.get("request_timeout_sec", 120))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        return "", f"HTTP error {exc.code}: {exc.reason}"
    except urllib.error.URLError as exc:
        return "", f"URL error: {exc.reason}"

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return raw.strip(), None
    content = _parse_ragflow_content(payload)
    if not content:
        return "", "RAGFlow response missing content"
    return content, None


def _build_openai_request(text, config, stream=False):
    api_key = (config.get("openai_api_key") or config.get("api_key") or "").strip()
    if not api_key:
        return None, "api_key is not configured"
    api_base = (config.get("api_base") or "https://api.openai.com").rstrip("/")
    path = config.get("openai_chat_path", DEFAULT_CONFIG["openai_chat_path"])
    endpoint = config.get("llm_url") or f"{api_base}{path}"

    headers = dict(config.get("headers", {}))
    auth_header = config.get("auth_header", "Authorization")
    auth_prefix = config.get("auth_prefix", "Bearer ")
    headers[auth_header] = f"{auth_prefix}{api_key}"

    system_prompt = (config.get("system_prompt") or "").strip()
    messages = []
    if system_prompt:
        messages.append({"role": "system", "content": system_prompt})
    messages.append({"role": "user", "content": text})

    body = dict(config.get("extra_body", {}))
    body.update(
        {
            "model": config.get("model", "gpt-4o-mini"),
            "messages": messages,
            "stream": bool(stream),
        }
    )
    body_bytes = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(endpoint, data=body_bytes, headers=headers, method="POST")
    return req, None


def _request_openai(text, config):
    req, err = _build_openai_request(text, config, stream=False)
    if err:
        return "", err
    timeout = float(config.get("request_timeout_sec", 120))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        return "", f"HTTP error {exc.code}: {exc.reason}"
    except urllib.error.URLError as exc:
        return "", f"URL error: {exc.reason}"

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return raw.strip(), None
    content = _parse_openai_content(payload)
    if not content:
        return "", "OpenAI response missing content"
    return content, None


def _iter_sse_lines(resp):
    while True:
        line = resp.readline()
        if not line:
            break
        decoded = line.decode("utf-8", errors="replace").strip()
        if not decoded:
            continue
        yield decoded


def request_llm_stream(text, config):
    provider = (config.get("provider") or "ragflow").lower()
    if provider != "openai":
        content, err = request_llm(text, config)
        if err:
            raise RuntimeError(err)
        yield content
        return

    req, err = _build_openai_request(text, config, stream=True)
    if err:
        raise RuntimeError(err)
    timeout = float(config.get("request_timeout_sec", 120))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            for line in _iter_sse_lines(resp):
                if not line.startswith("data:"):
                    continue
                data = line[5:].strip()
                if data == "[DONE]":
                    break
                try:
                    payload = json.loads(data)
                except json.JSONDecodeError:
                    continue
                choices = payload.get("choices") if isinstance(payload, dict) else None
                first = choices[0] if isinstance(choices, list) and choices else None
                if not isinstance(first, dict):
                    continue
                delta = first.get("delta") or {}
                content = delta.get("content")
                if isinstance(content, str) and content:
                    yield content
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f"HTTP error {exc.code}: {exc.reason}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"URL error: {exc.reason}") from exc


def request_llm(text, config):
    provider = (config.get("provider") or "ragflow").lower()
    if provider == "ragflow":
        return _request_ragflow(text, config)
    if provider == "openai":
        return _request_openai(text, config)

    url = config.get("llm_url", "").strip()
    if not url:
        return "", "llm_url is not configured"

    method = config.get("http_method", "POST").upper()
    headers = dict(config.get("headers", {}))
    api_key = config.get("api_key", "").strip()
    auth_header = config.get("auth_header", "").strip()
    auth_prefix = config.get("auth_prefix", "")
    if api_key and auth_header:
        headers[auth_header] = f"{auth_prefix}{api_key}"

    if method == "GET":
        params = {config.get("input_field", "query"): text}
        query = urllib.parse.urlencode(params)
        url = f"{url}?{query}"
        body_bytes = None
    else:
        body = dict(config.get("extra_body", {}))
        body[config.get("input_field", "query")] = text
        body_bytes = json.dumps(body).encode("utf-8")

    req = urllib.request.Request(url, data=body_bytes, headers=headers, method=method)
    timeout = float(config.get("request_timeout_sec", 20))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        return "", f"HTTP error {exc.code}: {exc.reason}"
    except urllib.error.URLError as exc:
        return "", f"URL error: {exc.reason}"

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return raw.strip(), None

    path = config.get("response_text_path") or []
    if path:
        extracted = _extract_by_path(payload, path)
        if isinstance(extracted, str):
            return extracted.strip(), None
    key = config.get("response_text_key", "")
    if key and isinstance(payload, dict) and key in payload:
        value = payload.get(key)
        if isinstance(value, str):
            return value.strip(), None
    if isinstance(payload, str):
        return payload.strip(), None
    return json.dumps(payload, ensure_ascii=True), None


def find_triggers(text, config):
    triggers = []
    for entry in config.get("command_triggers", []):
        phrase = entry.get("phrase", "")
        if not phrase:
            continue
        haystack = text if entry.get("case_sensitive", False) else text.lower()
        needle = phrase if entry.get("case_sensitive", False) else phrase.lower()
        if needle in haystack:
            triggers.append(entry)
    return triggers


def trigger_actions(node, triggers, config):
    if not triggers:
        return
    enable_exec = bool(config.get("enable_command_exec", False))
    for entry in triggers:
        command = entry.get("command", "")
        label = entry.get("label") or entry.get("phrase", "")
        if enable_exec and command:
            threading.Thread(
                target=os.system, args=(command,), daemon=True
            ).start()
            node.get_logger().info(f"Triggered command for '{label}': {command}")
        else:
            node.get_logger().info(f"Matched trigger '{label}', exec disabled.")
