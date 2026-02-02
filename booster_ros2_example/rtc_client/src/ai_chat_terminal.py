import argparse
import json
import shlex
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from booster_interface.msg import BoosterApiReqMsg, Subtitle
from booster_interface.srv import RpcService


WELCOME_MSG = "Welcome! It's great to see you"
DEFAULT_SYSTEM_PROMPT = """
## prompt
You're a robot named Adam that can offer services like emotional chatting to users.

## skill
I am a professional chatbot capable of providing conversation services with a variety of voices.
""".strip()


def make_msg(api_id, body):
    msg = BoosterApiReqMsg()
    msg.api_id = int(api_id)
    msg.body = body
    return msg


def call_api(node, client, api_id, body, timeout_sec=5.0):
    node.get_logger().info(f"RPC api_id={api_id} body={body}")
    req = RpcService.Request()
    req.msg = make_msg(api_id, body)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f"RpcService call timed out (api_id={api_id})")
        return False
    response = future.result().msg
    if response.status != 0:
        node.get_logger().warn(f"RpcService returned status={response.status}")
    return True


def run_llm_command(command, user_input):
    placeholder = "{input}"
    escaped = shlex.quote(user_input)
    if placeholder in command:
        command = command.replace(placeholder, escaped)
    else:
        command = f"{command} {escaped}"
    try:
        output = subprocess.check_output(command, shell=True, text=True).strip()
        return output if output else user_input
    except subprocess.CalledProcessError:
        return user_input


def start_ai_chat(node, client):
    param = {
        "interrupt_mode": False,
        "asr_config": {
            "interrupt_speech_duration": 200,
            "interrupt_keywords": [],
        },
        "llm_config": {
            "system_prompt": DEFAULT_SYSTEM_PROMPT,
            "welcome_msg": WELCOME_MSG,
            "prompt_name": "",
        },
        "tts_config": {
            "voice_type": "zh_female_shuangkuaisisi_emo_v2_mars_bigtts",
            "ignore_bracket_text": [3],
        },
        "enable_face_tracking": False,
    }
    body = json.dumps(param)
    return call_api(node, client, 2000, body)


def stop_ai_chat(node, client):
    return call_api(node, client, 2001, "")


def start_face_tracking(node, client):
    return call_api(node, client, 2003, "")


def stop_face_tracking(node, client):
    return call_api(node, client, 2004, "")


def speak(node, client, text):
    body = json.dumps({"msg": text})
    return call_api(node, client, 2002, body)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tts-only", action="store_true", help="Skip starting AI chat; speak input only.")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("ai_chat_terminal")
    client = node.create_client(RpcService, "booster_rtc_service")

    subtitle_topic = node.declare_parameter("subtitle_topic", "/ai_subtitle").get_parameter_value().string_value
    llm_cmd = node.declare_parameter("llm_cmd", "").get_parameter_value().string_value
    enable_face_tracking = node.declare_parameter("enable_face_tracking", False).get_parameter_value().bool_value

    while not client.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            return 1
        node.get_logger().info("booster_rtc_service not available, waiting...")

    def on_subtitle(msg):
        if msg.text:
            print(f"AI: {msg.text}")

    node.create_subscription(Subtitle, subtitle_topic, on_subtitle, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if not args.tts_only:
        if not start_ai_chat(node, client):
            executor.shutdown()
            rclpy.shutdown()
            return 1
        if enable_face_tracking:
            start_face_tracking(node, client)

    if args.tts_only:
        print("TTS-only mode. Type text to speak, or /quit to exit.")
    elif not llm_cmd:
        print("AI chat started. Type text to speak, or /quit to exit.")
    else:
        print("AI chat started. Type text to get an AI response, or /quit to exit.")

    try:
        for line in iter(input, ""):
            line = line.strip()
            if line == "/quit":
                break
            if not line:
                continue

            response = line
            if llm_cmd:
                response = run_llm_command(llm_cmd, line)
                if response != line:
                    print(f"AI: {response}")
            speak(node, client, response)
    except EOFError:
        pass

    if not args.tts_only:
        if enable_face_tracking:
            stop_face_tracking(node, client)
        stop_ai_chat(node, client)

    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
