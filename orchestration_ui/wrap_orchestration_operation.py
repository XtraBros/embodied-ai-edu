#!/usr/bin/env python3
import argparse
import json
import sys


def load_payload(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def build_wrapped(payload, agent_id, operation, orch_id, track_id, track_type):
    if isinstance(payload, list):
        content = payload
    else:
        content = payload.get("content")

    wrapped = {
        "agent_id": agent_id,
        "operation": operation,
    }

    orch_value = orch_id or (payload.get("orch_id") if isinstance(payload, dict) else None)
    track_value = track_id or (payload.get("track_id") if isinstance(payload, dict) else None)
    track_type_value = (
        track_type if track_type is not None else (payload.get("track_type") if isinstance(payload, dict) else None)
    )

    if orch_value is not None:
        wrapped["orch_id"] = orch_value
    if track_value is not None:
        wrapped["track_id"] = track_value
    if track_type_value is not None:
        wrapped["track_type"] = track_type_value
    if content is not None:
        wrapped["content"] = content

    return wrapped


def main():
    parser = argparse.ArgumentParser(
        description="Wrap orchestration-ui export JSON for orchestration_operation requests."
    )
    parser.add_argument("input", help="Path to export JSON from orchestration-ui.")
    parser.add_argument("--agent-id", required=True, help="Agent id to include in the request.")
    parser.add_argument(
        "--operation",
        default="orch_set_track_data",
        help="Operation name for orchestration_operation (default: orch_set_track_data).",
    )
    parser.add_argument("--orch-id", help="Override orch_id from the export.")
    parser.add_argument("--track-id", help="Override track_id from the export.")
    parser.add_argument("--track-type", type=int, help="Override track_type from the export.")
    parser.add_argument("--out", help="Output path. Defaults to stdout when omitted.")

    args = parser.parse_args()
    payload = load_payload(args.input)
    wrapped = build_wrapped(
        payload,
        args.agent_id,
        args.operation,
        args.orch_id,
        args.track_id,
        args.track_type,
    )

    output = json.dumps(wrapped, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(output)
            handle.write("\n")
    else:
        sys.stdout.write(output + "\n")


if __name__ == "__main__":
    main()
