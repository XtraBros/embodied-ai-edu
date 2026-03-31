#!/usr/bin/env python3

import argparse
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String, UInt8MultiArray


def parse_args():
    parser = argparse.ArgumentParser(description="Riva ASR client node (streaming).")
    parser.add_argument("--input-topic", default="/audio/utterance_audio", help="Utterance audio topic.")
    parser.add_argument("--start-topic", default="/audio/utterance_start", help="Utterance start topic.")
    parser.add_argument("--end-topic", default="/audio/utterance_end", help="Utterance end topic.")
    parser.add_argument("--partial-topic", default="/asr/partial", help="Partial ASR output topic.")
    parser.add_argument("--final-topic", default="/asr/text", help="Final ASR output topic.")
    parser.add_argument("--language", default="en-US", help="Language code (param override).")
    return parser.parse_args()


class RivaAsrNode(Node):
    def __init__(self, args):
        super().__init__("riva_asr")
        self.args = args
        self.language = args.language
        self.frames = []
        self.started = False
        self.last_warn = 0.0

        self.pub_partial = self.create_publisher(String, args.partial_topic, 10)
        self.pub_final = self.create_publisher(String, args.final_topic, 10)
        self.sub_audio = self.create_subscription(
            UInt8MultiArray, args.input_topic, self._on_audio, 10
        )
        self.sub_start = self.create_subscription(
            Empty, args.start_topic, self._on_start, 10
        )
        self.sub_end = self.create_subscription(
            Empty, args.end_topic, self._on_end, 10
        )

        self.declare_parameter("language", self.language)
        self.add_on_set_parameters_callback(self._on_params)

    def _on_params(self, params):
        for param in params:
            if param.name == "language" and param.value:
                self.language = str(param.value)
                self.get_logger().info(f"ASR language set to {self.language}")
        return rclpy.parameter.SetParametersResult(successful=True)

    def _on_start(self, msg):
        self.frames = []
        self.started = True

    def _on_audio(self, msg):
        if not self.started:
            return
        self.frames.append(bytes(msg.data))

    def _on_end(self, msg):
        if not self.started:
            return
        self.started = False
        audio = b"".join(self.frames)
        self.frames = []
        if not audio:
            return
        text = self._run_riva(audio)
        if text:
            out = String()
            out.data = text
            self.pub_final.publish(out)

    def _run_riva(self, audio_bytes):
        try:
            import riva.client  # type: ignore
        except ImportError:
            now = time.time()
            if now - self.last_warn > 5.0:
                self.get_logger().error(
                    "riva.client not installed. Install Riva Python client to enable ASR."
                )
                self.last_warn = now
            return ""

        server = os.environ.get("RIVA_URI", "localhost:50051")
        use_ssl = bool(int(os.environ.get("RIVA_USE_SSL", "0")))
        auth = os.environ.get("RIVA_API_KEY", "")
        channel = riva.client.AuthenticatedChannel(server, use_ssl=use_ssl, auth_token=auth)
        asr_service = riva.client.ASRService(channel)
        config = riva.client.RecognitionConfig(
            encoding=riva.client.AudioEncoding.LINEAR_PCM,
            sample_rate_hertz=16000,
            language_code=self.language,
            max_alternatives=1,
            enable_automatic_punctuation=True,
        )
        results = asr_service.offline_recognize(audio_bytes, config)
        if results and results.results:
            return results.results[0].alternatives[0].transcript
        return ""


def main():
    args = parse_args()
    rclpy.init()
    node = RivaAsrNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
