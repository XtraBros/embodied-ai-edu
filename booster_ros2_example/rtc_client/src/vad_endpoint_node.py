#!/usr/bin/env python3

import argparse
import audioop
import collections
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, UInt8MultiArray

try:
    import webrtcvad
except ImportError:
    webrtcvad = None


def parse_args():
    parser = argparse.ArgumentParser(description="VAD/endpointing node.")
    parser.add_argument("--input-topic", default="/audio/frames", help="Audio frame topic.")
    parser.add_argument("--utterance-topic", default="/audio/utterance_audio", help="Utterance audio topic.")
    parser.add_argument("--start-topic", default="/audio/utterance_start", help="Utterance start topic.")
    parser.add_argument("--end-topic", default="/audio/utterance_end", help="Utterance end topic.")
    parser.add_argument("--sample-rate", type=int, default=16000, help="Sample rate.")
    parser.add_argument("--frame-ms", type=int, default=20, help="Frame size in ms.")
    parser.add_argument("--vad-mode", default="webrtc", choices=("webrtc", "energy"), help="VAD backend.")
    parser.add_argument("--vad-aggressiveness", type=int, default=2, help="WebRTC VAD aggressiveness (0-3).")
    parser.add_argument("--energy-threshold", type=int, default=500, help="Energy threshold for fallback VAD.")
    parser.add_argument("--pre-roll-ms", type=int, default=400, help="Pre-roll buffer in ms.")
    parser.add_argument("--speech-start-ms", type=int, default=240, help="Speech start threshold in ms.")
    parser.add_argument("--speech-end-silence-ms", type=int, default=700, help="Silence end threshold in ms.")
    parser.add_argument("--min-utterance-ms", type=int, default=400, help="Minimum utterance length.")
    parser.add_argument("--max-utterance-ms", type=int, default=15000, help="Maximum utterance length.")
    return parser.parse_args()


class VadEndpointNode(Node):
    def __init__(self, args):
        super().__init__("vad_endpoint")
        self.args = args
        self.bytes_per_sample = 2
        self.frame_bytes = int(
            args.sample_rate * self.bytes_per_sample * (args.frame_ms / 1000.0)
        )

        self.pub_audio = self.create_publisher(UInt8MultiArray, args.utterance_topic, 10)
        self.pub_start = self.create_publisher(Empty, args.start_topic, 10)
        self.pub_end = self.create_publisher(Empty, args.end_topic, 10)
        self.sub = self.create_subscription(UInt8MultiArray, args.input_topic, self._on_frame, 10)

        self.vad = None
        self.vad_mode = args.vad_mode
        if self.vad_mode == "webrtc":
            if webrtcvad is None:
                self.get_logger().warn("webrtcvad not installed; falling back to energy VAD.")
                self.vad_mode = "energy"
            else:
                self.vad = webrtcvad.Vad(args.vad_aggressiveness)

        self.pre_roll = collections.deque(
            maxlen=max(1, int(args.pre_roll_ms / args.frame_ms))
        )
        self.in_utterance = False
        self.speech_frames = 0
        self.silence_frames = 0
        self.utterance_frames = 0
        self.start_time = None

        self.speech_start_frames = max(1, int(args.speech_start_ms / args.frame_ms))
        self.speech_end_frames = max(1, int(args.speech_end_silence_ms / args.frame_ms))
        self.min_utterance_frames = max(1, int(args.min_utterance_ms / args.frame_ms))
        self.max_utterance_frames = max(1, int(args.max_utterance_ms / args.frame_ms))

    def _is_speech(self, pcm_bytes):
        if self.vad_mode == "webrtc" and self.vad:
            return self.vad.is_speech(pcm_bytes, self.args.sample_rate)
        rms = audioop.rms(pcm_bytes, self.bytes_per_sample)
        return rms >= self.args.energy_threshold

    def _publish_audio(self, pcm_bytes):
        msg = UInt8MultiArray()
        msg.data = list(pcm_bytes)
        self.pub_audio.publish(msg)

    def _start_utterance(self):
        self.in_utterance = True
        self.silence_frames = 0
        self.utterance_frames = 0
        self.start_time = time.time()
        self.pub_start.publish(Empty())
        for frame in self.pre_roll:
            self._publish_audio(frame)

    def _end_utterance(self):
        if self.utterance_frames >= self.min_utterance_frames:
            self.pub_end.publish(Empty())
        self.in_utterance = False
        self.speech_frames = 0
        self.silence_frames = 0
        self.utterance_frames = 0
        self.start_time = None

    def _on_frame(self, msg):
        pcm_bytes = bytes(msg.data)
        if len(pcm_bytes) != self.frame_bytes:
            return
        self.pre_roll.append(pcm_bytes)
        speech = self._is_speech(pcm_bytes)

        if not self.in_utterance:
            if speech:
                self.speech_frames += 1
            else:
                self.speech_frames = 0
            if self.speech_frames >= self.speech_start_frames:
                self._start_utterance()
            return

        self._publish_audio(pcm_bytes)
        self.utterance_frames += 1

        if speech:
            self.silence_frames = 0
        else:
            self.silence_frames += 1

        if self.silence_frames >= self.speech_end_frames:
            self._end_utterance()
            return

        if self.utterance_frames >= self.max_utterance_frames:
            self._end_utterance()


def main():
    args = parse_args()
    rclpy.init()
    node = VadEndpointNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
