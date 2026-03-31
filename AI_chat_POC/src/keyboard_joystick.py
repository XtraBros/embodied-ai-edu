#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from booster_interface.msg import RemoteControllerState


SDL_HAT_CENTERED = 0x00
SDL_HAT_UP = 0x01
SDL_HAT_RIGHT = 0x02
SDL_HAT_DOWN = 0x04
SDL_HAT_LEFT = 0x08


class TerminalRawMode:
    def __init__(self, stream):
        self.stream = stream
        self.fd = stream.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


class KeyboardJoystick(Node):
    def __init__(self, args):
        super().__init__("keyboard_joystick")
        self.args = args
        self.publisher = self.create_publisher(
            RemoteControllerState, args.topic, 10
        )

        self.axis_values = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
        self.axis_last = {key: 0.0 for key in self.axis_values}

        self.button_values = {
            "a": False,
            "b": False,
            "x": False,
            "y": False,
            "lb": False,
            "rb": False,
            "lt": False,
            "rt": False,
            "ls": False,
            "rs": False,
            "back": False,
            "start": False,
        }
        self.button_last = {key: 0.0 for key in self.button_values}

        self.hat_pos = SDL_HAT_CENTERED
        self.hat_last = 0.0
        self.last_snapshot = None

        self.axis_map = {
            "w": ("ly", -1.0),
            "s": ("ly", 1.0),
            "a": ("lx", -1.0),
            "d": ("lx", 1.0),
            "i": ("ry", -1.0),
            "k": ("ry", 1.0),
            "j": ("rx", -1.0),
            "l": ("rx", 1.0),
        }

        self.button_map = {
            "1": "a",
            "2": "b",
            "3": "x",
            "4": "y",
            "5": "lb",
            "6": "rb",
            "7": "lt",
            "8": "rt",
            "z": "ls",
            "x": "rs",
            "b": "back",
            "n": "start",
        }

    def log_controls(self):
        controls = [
            "WASD: left stick (lx/ly)",
            "IJKL: right stick (rx/ry)",
            "Arrow keys: D-pad (hat)",
            "1/2/3/4: A/B/X/Y",
            "5/6: LB/RB",
            "7/8: LT/RT",
            "z/x: LS/RS",
            "b/n: Back/Start",
            "Space: zero all",
            "q: quit",
        ]
        for line in controls:
            self.get_logger().info(line)

    def read_key(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        if select.select([sys.stdin], [], [], 0)[0]:
            ch += sys.stdin.read(1)
        if select.select([sys.stdin], [], [], 0)[0]:
            ch += sys.stdin.read(1)
        return ch

    def handle_key(self, key, now):
        if key is None:
            return False
        self.get_logger().info(f"Key pressed: {repr(key)}")
        if key == "q":
            return True
        if key == " ":
            for axis_key in self.axis_values:
                self.axis_values[axis_key] = 0.0
            for button_key in self.button_values:
                self.button_values[button_key] = False
            self.hat_pos = SDL_HAT_CENTERED
            return False
        if key in self.axis_map:
            axis, direction = self.axis_map[key]
            self.axis_values[axis] = direction * self.args.scale
            self.axis_last[axis] = now
            return False
        if key in self.button_map:
            button = self.button_map[key]
            self.button_values[button] = True
            self.button_last[button] = now
            return False
        if key.startswith("\x1b"):
            self.handle_arrow(key, now)
        return False

    def handle_arrow(self, key, now):
        if key == "\x1b[A":
            self.hat_pos = SDL_HAT_UP
        elif key == "\x1b[B":
            self.hat_pos = SDL_HAT_DOWN
        elif key == "\x1b[C":
            self.hat_pos = SDL_HAT_RIGHT
        elif key == "\x1b[D":
            self.hat_pos = SDL_HAT_LEFT
        else:
            return
        self.hat_last = now

    def decay_state(self, now):
        for axis_key, last_time in self.axis_last.items():
            if now - last_time > self.args.axis_hold:
                self.axis_values[axis_key] = 0.0
        for button_key, last_time in self.button_last.items():
            if now - last_time > self.args.button_hold:
                self.button_values[button_key] = False
        if now - self.hat_last > self.args.hat_hold:
            self.hat_pos = SDL_HAT_CENTERED

    def _snapshot_state(self):
        axes = tuple(self.axis_values[key] for key in ("lx", "ly", "rx", "ry"))
        pressed = tuple(key for key, value in self.button_values.items() if value)
        hat = int(self.hat_pos)
        return axes, pressed, hat

    def publish_state(self):
        msg = RemoteControllerState()
        msg.event = self.args.event
        msg.lx = float(self.axis_values["lx"])
        msg.ly = float(self.axis_values["ly"])
        msg.rx = float(self.axis_values["rx"])
        msg.ry = float(self.axis_values["ry"])
        msg.a = self.button_values["a"]
        msg.b = self.button_values["b"]
        msg.x = self.button_values["x"]
        msg.y = self.button_values["y"]
        msg.lb = self.button_values["lb"]
        msg.rb = self.button_values["rb"]
        msg.lt = self.button_values["lt"]
        msg.rt = self.button_values["rt"]
        msg.ls = self.button_values["ls"]
        msg.rs = self.button_values["rs"]
        msg.back = self.button_values["back"]
        msg.start = self.button_values["start"]
        msg.hat_pos = int(self.hat_pos)
        msg.hat_c = self.hat_pos == SDL_HAT_CENTERED
        msg.hat_u = bool(self.hat_pos & SDL_HAT_UP)
        msg.hat_d = bool(self.hat_pos & SDL_HAT_DOWN)
        msg.hat_l = bool(self.hat_pos & SDL_HAT_LEFT)
        msg.hat_r = bool(self.hat_pos & SDL_HAT_RIGHT)
        msg.hat_lu = self.hat_pos == (SDL_HAT_LEFT | SDL_HAT_UP)
        msg.hat_ld = self.hat_pos == (SDL_HAT_LEFT | SDL_HAT_DOWN)
        msg.hat_ru = self.hat_pos == (SDL_HAT_RIGHT | SDL_HAT_UP)
        msg.hat_rd = self.hat_pos == (SDL_HAT_RIGHT | SDL_HAT_DOWN)
        self.publisher.publish(msg)
        snapshot = self._snapshot_state()
        if snapshot != self.last_snapshot:
            axes, pressed, hat = snapshot
            self.get_logger().info(
                f"Published state: axes={axes} buttons={list(pressed)} hat={hat}"
            )
            self.last_snapshot = snapshot


def publish_combo(topic, combo, event_press, event_release, hold_sec, auto_stop_sec):
    rclpy.init()
    node = Node("keyboard_joystick_combo")
    publisher = node.create_publisher(RemoteControllerState, topic, 10)
    logger = node.get_logger()
    logger.info(f"Publishing combo '{combo}' on {topic}")
    logger.info(f"Press event={event_press} hold={hold_sec:.2f}s")

    button_map = {
        "A": "a",
        "B": "b",
        "X": "x",
        "Y": "y",
        "LB": "lb",
        "RB": "rb",
        "LT": "lt",
        "RT": "rt",
        "LS": "ls",
        "RS": "rs",
        "BACK": "back",
        "START": "start",
    }

    msg = RemoteControllerState()
    msg.event = int(event_press)
    msg.lx = 0.0
    msg.ly = 0.0
    msg.rx = 0.0
    msg.ry = 0.0
    for name, field in button_map.items():
        setattr(msg, field, False)

    for part in combo.split("+"):
        part = part.strip().upper()
        field = button_map.get(part)
        if field:
            setattr(msg, field, True)

    msg.hat_pos = SDL_HAT_CENTERED
    msg.hat_c = True
    msg.hat_u = False
    msg.hat_d = False
    msg.hat_l = False
    msg.hat_r = False
    msg.hat_lu = False
    msg.hat_ld = False
    msg.hat_ru = False
    msg.hat_rd = False

    publisher.publish(msg)
    time.sleep(max(0.0, hold_sec))
    logger.info(f"Released combo '{combo}' (event={event_release})")

    release = RemoteControllerState()
    release.event = int(event_release)
    release.lx = 0.0
    release.ly = 0.0
    release.rx = 0.0
    release.ry = 0.0
    for name, field in button_map.items():
        setattr(release, field, False)
    release.hat_pos = SDL_HAT_CENTERED
    release.hat_c = True
    release.hat_u = False
    release.hat_d = False
    release.hat_l = False
    release.hat_r = False
    release.hat_lu = False
    release.hat_ld = False
    release.hat_ru = False
    release.hat_rd = False
    publisher.publish(release)

    if auto_stop_sec and auto_stop_sec > 0:
        logger.info(f"Auto-stop enabled; re-sending combo after {auto_stop_sec:.2f}s")
        time.sleep(auto_stop_sec)
        stop_msg = RemoteControllerState()
        stop_msg.event = int(event_press)
        stop_msg.lx = 0.0
        stop_msg.ly = 0.0
        stop_msg.rx = 0.0
        stop_msg.ry = 0.0
        for name, field in button_map.items():
            setattr(stop_msg, field, False)
        for part in combo.split("+"):
            part = part.strip().upper()
            field = button_map.get(part)
            if field:
                setattr(stop_msg, field, True)
        stop_msg.hat_pos = SDL_HAT_CENTERED
        stop_msg.hat_c = True
        stop_msg.hat_u = False
        stop_msg.hat_d = False
        stop_msg.hat_l = False
        stop_msg.hat_r = False
        stop_msg.hat_lu = False
        stop_msg.hat_ld = False
        stop_msg.hat_ru = False
        stop_msg.hat_rd = False
        publisher.publish(stop_msg)
        time.sleep(max(0.0, hold_sec))
        publisher.publish(release)
        logger.info(f"Auto-stop release sent for '{combo}'")

    node.destroy_node()
    rclpy.shutdown()


def publish_axis_once(topic, lx, ly, ry, event_press, event_release, hold_sec, rate_hz):
    rclpy.init()
    node = Node("keyboard_joystick_axis_once")
    publisher = node.create_publisher(RemoteControllerState, topic, 10)
    logger = node.get_logger()
    logger.info(f"Publishing axis lx={lx} ly={ly} ry={ry} for {hold_sec:.2f}s on {topic}")

    msg = RemoteControllerState()
    msg.event = int(event_press)
    msg.lx = float(lx)
    msg.ly = float(ly)
    msg.rx = 0.0
    msg.ry = float(ry)
    msg.a = False
    msg.b = False
    msg.x = False
    msg.y = False
    msg.lb = False
    msg.rb = False
    msg.lt = False
    msg.rt = False
    msg.ls = False
    msg.rs = False
    msg.back = False
    msg.start = False
    msg.hat_pos = SDL_HAT_CENTERED
    msg.hat_c = True
    msg.hat_u = False
    msg.hat_d = False
    msg.hat_l = False
    msg.hat_r = False
    msg.hat_lu = False
    msg.hat_ld = False
    msg.hat_ru = False
    msg.hat_rd = False

    start = time.monotonic()
    interval = 1.0 / max(rate_hz, 1.0)
    while True:
        publisher.publish(msg)
        if hold_sec <= 0:
            break
        if time.monotonic() - start >= hold_sec:
            break
        time.sleep(interval)

    release = RemoteControllerState()
    release.event = int(event_release)
    release.lx = 0.0
    release.ly = 0.0
    release.rx = 0.0
    release.ry = 0.0
    release.a = False
    release.b = False
    release.x = False
    release.y = False
    release.lb = False
    release.rb = False
    release.lt = False
    release.rt = False
    release.ls = False
    release.rs = False
    release.back = False
    release.start = False
    release.hat_pos = SDL_HAT_CENTERED
    release.hat_c = True
    release.hat_u = False
    release.hat_d = False
    release.hat_l = False
    release.hat_r = False
    release.hat_lu = False
    release.hat_ld = False
    release.hat_ru = False
    release.hat_rd = False
    publisher.publish(release)

    node.destroy_node()
    rclpy.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish keyboard input as RemoteControllerState."
    )
    parser.add_argument(
        "--topic",
        default="/remote_controller_state",
        help="ROS2 topic for RemoteControllerState.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Axis magnitude (0.0 - 1.0).",
    )
    parser.add_argument(
        "--axis-hold",
        type=float,
        default=0.2,
        help="Seconds to keep axis active without repeat input.",
    )
    parser.add_argument(
        "--button-hold",
        type=float,
        default=0.2,
        help="Seconds to keep button active without repeat input.",
    )
    parser.add_argument(
        "--hat-hold",
        type=float,
        default=0.2,
        help="Seconds to keep hat active without repeat input.",
    )
    parser.add_argument(
        "--event",
        type=int,
        default=0,
        help="SDL event field value to publish.",
    )
    parser.add_argument(
        "--event-press",
        type=int,
        default=1539,
        help="SDL event value for button press.",
    )
    parser.add_argument(
        "--event-release",
        type=int,
        default=1540,
        help="SDL event value for button release.",
    )
    parser.add_argument(
        "--combo",
        default="",
        help="Send a button combo once (example: LT+RT+A) and exit.",
    )
    parser.add_argument(
        "--axis",
        default="",
        help="Send axis once and exit (format: lx,ly,ry). Example: 0,-0.6,0",
    )
    parser.add_argument(
        "--axis-rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz for --axis one-shot streaming.",
    )
    parser.add_argument(
        "--combo-hold",
        type=float,
        default=0.2,
        help="Seconds to hold combo before release.",
    )
    parser.add_argument(
        "--auto-stop",
        type=float,
        default=0.0,
        help="Seconds after combo to send the combo again (mimics LLM auto-stop).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.combo:
        publish_combo(
            args.topic,
            args.combo,
            args.event_press,
            args.event_release,
            args.combo_hold,
            args.auto_stop,
        )
        return
    if args.axis:
        try:
            parts = [float(x.strip()) for x in args.axis.split(",")]
        except ValueError as exc:
            raise SystemExit(f"Invalid --axis value: {args.axis}") from exc
        if len(parts) != 3:
            raise SystemExit("Invalid --axis value; expected format: lx,ly,ry")
        publish_axis_once(
            args.topic,
            parts[0],
            parts[1],
            parts[2],
            args.event_press,
            args.event_release,
            args.axis_hold,
            args.axis_rate,
        )
        return
    rclpy.init()
    node = KeyboardJoystick(args)
    node.log_controls()
    rate = node.create_rate(args.rate)

    with TerminalRawMode(sys.stdin):
        try:
            while rclpy.ok():
                now = time.monotonic()
                key = node.read_key()
                should_quit = node.handle_key(key, now)
                node.decay_state(now)
                node.publish_state()
                if should_quit:
                    break
                rclpy.spin_once(node, timeout_sec=0.0)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
