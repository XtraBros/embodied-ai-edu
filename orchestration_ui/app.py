import base64
import json
import logging
import os
import secrets
import shlex
import time

import paramiko
from flask import Flask, jsonify, request, send_from_directory


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("orchestration-ui")

app = Flask(__name__, static_folder="public", static_url_path="")


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


def _sftp_exists(sftp, path):
    try:
        sftp.stat(path)
        return True
    except (FileNotFoundError, OSError):
        return False


def _sftp_mkdirs(sftp, path):
    parts = path.strip("/").split("/")
    current = ""
    for part in parts:
        current = f"{current}/{part}"
        if not _sftp_exists(sftp, current):
            sftp.mkdir(current)


def _read_json_file(sftp, path, default):
    if not _sftp_exists(sftp, path):
        return default
    with sftp.open(path, "r") as handle:
        raw = handle.read()
    text = raw.decode("utf-8") if isinstance(raw, (bytes, bytearray)) else str(raw)
    if not text.strip():
        return default
    return json.loads(text)


def _write_json_file(sftp, path, payload):
    data = json.dumps(payload, indent=2, ensure_ascii=True)
    with sftp.open(path, "w") as handle:
        handle.write(f"{data}\n")


LEFT_SHORTCUTS = {"LT", "LB", "RT", "RB"}
RIGHT_SHORTCUTS = {"UP", "DOWN", "LEFT", "RIGHT", "X", "Y", "A", "B"}
ALLOWED_SHORTCUTS = LEFT_SHORTCUTS | RIGHT_SHORTCUTS


def _normalize_shortcut(value):
    if isinstance(value, list):
        parts = [str(entry).strip().upper() for entry in value]
    elif isinstance(value, str):
        parts = [part.strip().upper() for part in value.replace(",", "+").split("+")]
    else:
        parts = []
    return [part for part in parts if part]


def _validate_shortcut(shortcut):
    normalized = _normalize_shortcut(shortcut)
    if not normalized:
        return []
    if len(normalized) != 2:
        raise ValueError("Shortcut must include exactly two buttons.")
    if normalized[0] == normalized[1]:
        raise ValueError("Shortcut must use two different buttons.")
    for entry in normalized:
        if entry not in ALLOWED_SHORTCUTS:
            raise ValueError(f"Invalid shortcut button: {entry}")
    left = next((entry for entry in normalized if entry in LEFT_SHORTCUTS), None)
    right = next((entry for entry in normalized if entry in RIGHT_SHORTCUTS), None)
    if not left or not right:
        raise ValueError("Shortcut must include one trigger/bumper and one face/dpad button.")
    return [left, right]


def _ensure_track_entry(tracks, track_id, track_type):
    for entry in tracks:
        if entry.get("track_id") == track_id:
            entry["track_type"] = track_type
            return
    tracks.append({"track_id": track_id, "track_type": track_type})


def _run_sudo(client, password, command):
    quoted = shlex.quote(command)
    stdin, stdout, stderr = client.exec_command(f"sudo -S bash -lc {quoted}")
    if password:
        stdin.write(f"{password}\n")
        stdin.flush()
    exit_code = stdout.channel.recv_exit_status()
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return exit_code, out, err


def _sudo_mkdirs(client, password, path):
    cmd = f"mkdir -p {shlex.quote(path)}"
    exit_code, _, err = _run_sudo(client, password, cmd)
    if exit_code != 0:
        raise OSError(f"sudo mkdir failed: {err.strip()}")


def _sudo_read_json(client, password, path, default):
    cmd = f"test -f {shlex.quote(path)} && cat {shlex.quote(path)}"
    exit_code, out, err = _run_sudo(client, password, cmd)
    if exit_code != 0:
        if err.strip():
            logger.warning("sudo read failed: %s", err.strip())
        return default
    text = out.strip()
    if not text:
        return default
    return json.loads(text)


def _sudo_write_json(client, password, path, payload):
    data = json.dumps(payload, indent=2, ensure_ascii=True) + "\n"
    encoded = base64.b64encode(data.encode("utf-8")).decode("ascii")
    cmd = (
        f"echo {shlex.quote(encoded)} | base64 -d | "
        f"tee {shlex.quote(path)} >/dev/null"
    )
    exit_code, _, err = _run_sudo(client, password, cmd)
    if exit_code != 0:
        raise OSError(f"sudo write failed: {err.strip()}")


def _run_remote_python(client, script, input_data=None):
    command = f"python3 - <<'PY'\n{script}\nPY"
    stdin, stdout, stderr = client.exec_command(command)
    if input_data is not None:
        if isinstance(input_data, bytes):
            data = input_data.decode("utf-8", errors="replace")
        else:
            data = str(input_data)
        stdin.write(data)
        stdin.flush()
        stdin.channel.shutdown_write()
    exit_code = stdout.channel.recv_exit_status()
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return exit_code, out, err


def _orch_request(client, params, request="orchestration_operation"):
    request_obj = {
        "request": request,
        "params": params,
        "language": "en",
        "app_api_level": 110,
        "app_platform": "Android",
    }
    encoded = base64.b64encode(
        json.dumps(request_obj, ensure_ascii=True).encode("utf-8")
    ).decode("ascii")
    script = f"""import base64, json, socket, struct, sys
req = json.loads(base64.b64decode("{encoded}").decode("utf-8"))
payload = json.dumps(req).encode("utf-8")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(8)
sock.connect(("127.0.0.1", 6868))
sock.sendall(struct.pack("<I", len(payload)))
sock.sendall(payload)
size_raw = sock.recv(4)
if len(size_raw) != 4:
    print("{{}}")
    sys.exit(1)
resp_size = struct.unpack("<I", size_raw)[0]
resp = b""
while len(resp) < resp_size:
    chunk = sock.recv(resp_size - len(resp))
    if not chunk:
        break
    resp += chunk
sock.close()
text = resp.decode("utf-8") if resp else ""
print(text)
"""
    exit_code, out, err = _run_remote_python(client, script)
    if exit_code != 0:
        message = err.strip() or out.strip() or "orchestration request failed"
        raise OSError(message)
    text = out.strip()
    if not text:
        raise OSError("Empty response from orchestration request.")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise OSError(f"Invalid response: {text}") from exc
    if not payload.get("success", False):
        raise OSError(payload.get("error") or text)
    return payload


def _extract_orch_id(payload):
    result = payload.get("result") or {}
    if isinstance(result, dict):
        orch = result.get("orch") if "orch" in result else result
        if isinstance(orch, dict):
            orch_id = orch.get("id") or orch.get("orch_id")
            if orch_id:
                return orch_id
    raise OSError("orch_create response did not include orch_id.")


def _orch_create(client, agent_id="com.boosterobotics.default"):
    payload = _orch_request(
        client,
        {"agent_id": agent_id, "operation": "orch_create"},
    )
    return _extract_orch_id(payload)


def _orch_set_name(client, orch_id, name, agent_id="com.boosterobotics.default"):
    _orch_request(
        client,
        {
            "agent_id": agent_id,
            "operation": "orch_set_name",
            "orch_id": orch_id,
            "name": name,
        },
    )


def _orch_set_track_data(
    client,
    orch_id,
    track_id,
    track_type,
    content,
    agent_id="com.boosterobotics.default",
):
    _orch_request(
        client,
        {
            "agent_id": agent_id,
            "operation": "orch_set_track_data",
            "orch_id": orch_id,
            "track_id": track_id,
            "track_type": track_type,
            "content": content,
        },
    )


def _orch_set_shortcut(client, orch_id, shortcut, agent_id="com.boosterobotics.default"):
    _orch_request(
        client,
        {
            "agent_id": agent_id,
            "operation": "orch_set_shortcut",
            "orch_id": orch_id,
            "shortcut": shortcut,
        },
    )


def _orch_set_activated(client, orch_id, activated=True, agent_id="com.boosterobotics.default"):
    _orch_request(
        client,
        {
            "agent_id": agent_id,
            "operation": "orch_set_activated",
            "orch_id": orch_id,
            "activated": bool(activated),
        },
    )


def _orch_get_all(client, agent_id="com.boosterobotics.default"):
    payload = _orch_request(
        client,
        {"agent_id": agent_id, "operation": "orch_get_all"},
    )
    result = payload.get("result") or {}
    if isinstance(result, dict):
        orch_list = result.get("orch_list")
        if isinstance(orch_list, list):
            return orch_list
    return []


def _publish_to_base(
    client,
    password,
    base_dir,
    orch_id,
    track_id,
    track_type,
    name,
    shortcut,
    content,
):
    orch_dir = f"{base_dir}/orchestrations/{orch_id}"
    track_dir = f"{orch_dir}/track_{track_id}"
    track_path = f"{track_dir}/{track_id}.json"
    index_path = f"{base_dir}/orchestrations.json"

    try:
        sftp = client.open_sftp()
        _sftp_mkdirs(sftp, track_dir)
        _write_json_file(sftp, track_path, content)
        orch_index = _read_json_file(sftp, index_path, {"orch_list": []})
    except PermissionError:
        logger.warning("SFTP permission denied for %s; falling back to sudo.", base_dir)
        _sudo_mkdirs(client, password, track_dir)
        _sudo_write_json(client, password, track_path, content)
        orch_index = _sudo_read_json(client, password, index_path, {"orch_list": []})

    if not isinstance(orch_index, dict):
        orch_index = {"orch_list": []}
    orch_list = orch_index.get("orch_list")
    if not isinstance(orch_list, list):
        orch_list = []
        orch_index["orch_list"] = orch_list

    orch_entry = next((entry for entry in orch_list if entry.get("id") == orch_id), None)
    if orch_entry is None:
        orch_entry = {
            "activated": False,
            "allow_delete": True,
            "create_time": int(time.time()),
            "icon_name": "",
            "id": orch_id,
            "name": name,
            "shortcut": shortcut,
            "tracks": []
        }
        orch_list.append(orch_entry)
    else:
        orch_entry["name"] = name or orch_entry.get("name") or orch_id
        orch_entry["shortcut"] = shortcut

    tracks = orch_entry.get("tracks")
    if not isinstance(tracks, list):
        tracks = []
        orch_entry["tracks"] = tracks
    _ensure_track_entry(tracks, track_id, track_type)

    try:
        if "sftp" in locals():
            _write_json_file(sftp, index_path, orch_index)
        else:
            _sudo_write_json(client, password, index_path, orch_index)
    except PermissionError:
        _sudo_write_json(client, password, index_path, orch_index)


@app.post("/api/publish")
def publish():
    data = request.get_json(silent=True) or {}
    robot = data.get("robot") or {}
    payload = data.get("payload") or {}
    host = (robot.get("host") or "").strip()
    username = (robot.get("username") or "").strip()
    password = robot.get("password") or ""
    logger.info(
        "Publish request host=%s user=%s orch_id=%s track_id=%s",
        host,
        username,
        (payload.get("orch_id") or "").strip(),
        (payload.get("track_id") or "").strip(),
    )

    if not host or not username or not password:
        return jsonify({"error": "Robot host, username, and password are required."}), 400

    content = payload.get("content")
    if not isinstance(content, list) or not content:
        return jsonify({"error": "Payload content must be a non-empty list."}), 400

    track_id = (payload.get("track_id") or "").strip() or secrets.token_hex(16)
    try:
        track_type = int(payload.get("track_type") or 0)
    except (TypeError, ValueError):
        track_type = 0

    try:
        shortcut = _validate_shortcut(payload.get("shortcut"))
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    name = (payload.get("name") or "").strip()
    agent_id = "com.boosterobotics.default"

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(hostname=host, username=username, password=password, timeout=10)
        orch_id = _orch_create(client, agent_id=agent_id)
        if name:
            _orch_set_name(client, orch_id, name, agent_id=agent_id)
        _orch_set_track_data(
            client,
            orch_id,
            track_id,
            track_type,
            content,
            agent_id=agent_id,
        )
        if shortcut:
            _orch_set_shortcut(client, orch_id, shortcut, agent_id=agent_id)
        _orch_set_activated(client, orch_id, True, agent_id=agent_id)
        time.sleep(0.2)
        orch_list = _orch_get_all(client, agent_id=agent_id)
        orch_entry = next(
            (entry for entry in orch_list if entry.get("id") == orch_id),
            None,
        )
        if not orch_entry:
            raise OSError(f"Orchestration not found after publish: {orch_id}")
        if not orch_entry.get("activated", False):
            _orch_set_activated(client, orch_id, True, agent_id=agent_id)
            time.sleep(0.2)
            orch_list = _orch_get_all(client, agent_id=agent_id)
            orch_entry = next(
                (entry for entry in orch_list if entry.get("id") == orch_id),
                None,
            )
            if not orch_entry or not orch_entry.get("activated", False):
                raise OSError(
                    f"Orchestration published but still inactive: {orch_id}"
                )
    except (OSError, ValueError) as exc:
        logger.exception("SSH publish failed")
        return jsonify({"error": f"SSH publish failed: {exc}"}), 500
    finally:
        try:
            client.close()
        except Exception:
            pass

    return jsonify(
        {
            "orch_id": orch_id,
            "track_id": track_id,
            "track_type": track_type,
            "published": True
        }
    )


if __name__ == "__main__":
    port = int(os.environ.get("PORT", "3000"))
    print(f"UI server on http://localhost:{port}")
    app.run(host="0.0.0.0", port=port)
