#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate walk_step_sec for llm_web_test walk commands."
    )
    parser.add_argument(
        "--config",
        default="config/ai_chat_web.json",
        help="Path to the LLM config JSON to read/write.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=4,
        help="Requested steps used during calibration.",
    )
    parser.add_argument(
        "--direction",
        default="forward",
        choices=["forward", "backward", "left", "right"],
        help="Direction to test.",
    )
    parser.add_argument(
        "--step-sec",
        type=float,
        default=None,
        help="Override walk_step_sec instead of reading from config.",
    )
    parser.add_argument(
        "--update-config",
        action="store_true",
        help="Write the calibrated walk_step_sec back to the config file.",
    )
    return parser.parse_args()


def load_config(path):
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_config(path, data):
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=True)
        handle.write("\n")


def main():
    args = parse_args()
    if args.steps <= 0:
        raise ValueError("--steps must be greater than 0.")

    config_path = Path(args.config).expanduser()
    config = load_config(config_path)

    current = args.step_sec
    if current is None:
        current = float(config.get("walk_step_sec", 0.6))

    print(
        f"Calibration setup: direction={args.direction}, requested_steps={args.steps}, "
        f"walk_step_sec={current:.3f}"
    )
    print("Run the walk command now and count actual steps.")
    actual = input("Actual steps taken: ").strip()
    if not actual:
        raise ValueError("Actual steps cannot be empty.")
    actual_steps = int(actual)
    if actual_steps <= 0:
        raise ValueError("Actual steps must be greater than 0.")

    new_value = current * (actual_steps / args.steps)
    print(f"Suggested walk_step_sec: {new_value:.3f}")

    if args.update_config:
        config["walk_step_sec"] = new_value
        save_config(config_path, config)
        print(f"Updated {config_path} with walk_step_sec={new_value:.3f}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
