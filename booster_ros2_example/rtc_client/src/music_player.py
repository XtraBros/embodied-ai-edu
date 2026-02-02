#!/usr/bin/env python3
import argparse
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_PLAYERS = {
    ".wav": ["aplay", "paplay", "ffplay -nodisp -autoexit"],
    ".mp3": ["ffplay -nodisp -autoexit", "mpg123", "mpg321", "paplay"],
    ".aac": ["ffplay -nodisp -autoexit", "paplay"],
    ".flac": ["ffplay -nodisp -autoexit", "paplay"],
    ".ogg": ["ffplay -nodisp -autoexit", "paplay"],
}


def _expand_player(cmd):
    return shlex.split(cmd) if isinstance(cmd, str) else list(cmd)


def _resolve_path(path_str):
    path = Path(os.path.expanduser(path_str)).resolve()
    if not path.exists():
        raise FileNotFoundError(f"Audio file not found: {path}")
    return path


def _build_player_candidates(ext, override):
    if override:
        return [_expand_player(override)]
    return [_expand_player(cmd) for cmd in DEFAULT_PLAYERS.get(ext, [])]


def _pick_player(players):
    for player in players:
        if shutil.which(player[0]):
            return player
    return None


def _pid_paths():
    runtime_dir = Path(os.path.expanduser("~/.cache/music-player"))
    runtime_dir.mkdir(parents=True, exist_ok=True)
    return runtime_dir / "player.pid", runtime_dir / "player.cmd"


def _read_pid():
    pid_file, cmd_file = _pid_paths()
    if not pid_file.exists():
        return None, None
    try:
        pid = int(pid_file.read_text().strip())
    except ValueError:
        pid = None
    cmd = cmd_file.read_text().strip() if cmd_file.exists() else None
    return pid, cmd


def _write_pid(pid, cmd):
    pid_file, cmd_file = _pid_paths()
    pid_file.write_text(str(pid))
    cmd_file.write_text(cmd)


def _clear_pid():
    pid_file, cmd_file = _pid_paths()
    if pid_file.exists():
        pid_file.unlink()
    if cmd_file.exists():
        cmd_file.unlink()


def _is_running(pid):
    if not pid:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def cmd_play(args):
    path = _resolve_path(args.file)
    ext = path.suffix.lower()
    players = _build_player_candidates(ext, args.player)
    if not players:
        raise RuntimeError(f"No default players configured for {ext}.")
    player = _pick_player(players)
    if not player:
        raise RuntimeError("No available player found on PATH.")

    pid, existing_cmd = _read_pid()
    if _is_running(pid):
        raise RuntimeError(f"Music already playing (PID {pid}): {existing_cmd}")

    cmd = player + [str(path)]
    proc = subprocess.Popen(cmd)
    _write_pid(proc.pid, " ".join(cmd))
    print(f"Started: {' '.join(cmd)} (PID {proc.pid})")


def cmd_stop(_args):
    pid, cmd = _read_pid()
    if not _is_running(pid):
        _clear_pid()
        print("No active music player.")
        return
    os.kill(pid, 15)
    _clear_pid()
    print(f"Stopped: {cmd}")


def cmd_status(_args):
    pid, cmd = _read_pid()
    if _is_running(pid):
        print(f"Playing (PID {pid}): {cmd}")
    else:
        print("Not playing.")


def parse_args():
    parser = argparse.ArgumentParser(description="Play/stop music files on the robot.")
    sub = parser.add_subparsers(dest="command", required=True)

    play = sub.add_parser("play", help="Play an audio file.")
    play.add_argument("file", help="Path to audio file (mp3/wav/aac/flac/ogg).")
    play.add_argument(
        "--player",
        default="",
        help="Override playback command (example: 'ffplay -nodisp -autoexit').",
    )
    play.set_defaults(func=cmd_play)

    stop = sub.add_parser("stop", help="Stop current playback.")
    stop.set_defaults(func=cmd_stop)

    status = sub.add_parser("status", help="Show playback status.")
    status.set_defaults(func=cmd_status)

    return parser.parse_args()


def main():
    args = parse_args()
    args.func(args)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
