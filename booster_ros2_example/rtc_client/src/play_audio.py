#!/usr/bin/env python3
import argparse
import os
import shlex
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


def _try_play(path, players):
    errors = []
    for player in players:
        cmd = player + [str(path)]
        try:
            result = subprocess.run(cmd, check=False)
        except FileNotFoundError:
            errors.append(f"Player not found: {cmd[0]}")
            continue
        if result.returncode == 0:
            return True
        errors.append(f"Player failed ({result.returncode}): {' '.join(cmd)}")
    raise RuntimeError("; ".join(errors) if errors else "No available player.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Play a local audio file on the robot speaker."
    )
    parser.add_argument("file", help="Path to audio file (mp3/wav/aac/flac/ogg).")
    parser.add_argument(
        "--player",
        default="",
        help="Override playback command (example: 'ffplay -nodisp -autoexit').",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    path = _resolve_path(args.file)
    ext = path.suffix.lower()
    players = _build_player_candidates(ext, args.player)
    if not players:
        raise RuntimeError(f"No default players configured for {ext}.")
    _try_play(path, players)
    print(f"Played: {path}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
