#!/usr/bin/env python3

import argparse
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


def parse_args():
    parser = argparse.ArgumentParser(description="Audio capture node (arecord).")
    parser.add_argument("--topic", default="/audio/frames", help="Output topic.")
    parser.add_argument("--sample-rate", type=int, default=16000, help="Sample rate.")
    parser.add_argument("--channels", type=int, default=1, help="Channel count.")
    parser.add_argument("--frame-ms", type=int, default=20, help="Frame size in ms.")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("audio_capture")
    pub = node.create_publisher(UInt8MultiArray, args.topic, 10)

    bytes_per_sample = 2
    frame_bytes = int(args.sample_rate * args.channels * bytes_per_sample * (args.frame_ms / 1000.0))

    command = [
        "arecord",
        "-q",
        "-f",
        "S16_LE",
        "-r",
        str(args.sample_rate),
        "-c",
        str(args.channels),
        "-t",
        "raw",
    ]

    node.get_logger().info(
        f"Starting arecord: rate={args.sample_rate} channels={args.channels} frame_ms={args.frame_ms}"
    )
    try:
        proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    except FileNotFoundError:
        node.get_logger().error("arecord not found. Install ALSA utilities.")
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            data = proc.stdout.read(frame_bytes)
            if not data:
                break
            msg = UInt8MultiArray()
            msg.data = list(data)
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
