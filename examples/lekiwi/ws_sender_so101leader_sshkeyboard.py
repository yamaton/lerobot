"""
Sending robot-teleoperation signal over WebSocket

How to run:

1. Run the receiver (host) script

    python -m examples.lekiwi.ws_receiver

2. Run the sender (client) script

    python -m examples.lekiwi.ws_sender_so101leader_keyboard --host <host-ip-address>

"""

import asyncio
import json
import time
import argparse
import logging
from queue import Queue
from typing import Any
import threading


from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.teleoperators.keyboard import KeyboardTeleopConfig
from lerobot.teleoperators import Teleoperator
import websockets

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


SSHKEYBOARD_AVAILABLE = True
try:
    from sshkeyboard import listen_keyboard, stop_listening
except ImportError:
    SSHKEYBOARD_AVAILABLE = False
    logging.info("sshkeyboard is not available. Install with: pip install sshkeyboard")
except Exception as e:
    SSHKEYBOARD_AVAILABLE = False
    logging.info(f"Could not import sshkeyboard: {e}")


class SshKeyboardTeleop(Teleoperator):
    """
    Teleop class to use keyboard inputs for control using sshkeyboard library.
    Works without X server or uinput, suitable for SSH connections and headless systems.
    """

    config_class = KeyboardTeleopConfig
    name = "sshkeyboard"

    def __init__(self, config: KeyboardTeleopConfig):
        super().__init__(config)
        self.config = config
        self.robot_type = config.type

        self.event_queue = Queue()
        self.current_pressed = {}
        self.listener_thread = None
        self.listener_active = False
        self.logs = {}

    @property
    def action_features(self) -> dict:
        return {
            "dtype": "float32",
            "shape": "unknown",
            "names": "unknown",
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return (
            SSHKEYBOARD_AVAILABLE
            and self.listener_thread is not None
            and self.listener_active
        )

    @property
    def is_calibrated(self) -> bool:
        pass

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                "SshKeyboard is already connected. Do not run `robot.connect()` twice."
            )

        if not SSHKEYBOARD_AVAILABLE:
            logging.error(
                "sshkeyboard not available. Install with: pip install sshkeyboard"
            )
            return

        logging.info("sshkeyboard is available - enabling keyboard listener.")
        self.listener_active = True

        # Start listener in a separate thread
        self.listener_thread = threading.Thread(
            target=self._start_listener,
            daemon=True
        )
        self.listener_thread.start()

    def _start_listener(self):
        """Start the sshkeyboard listener in a separate thread."""
        try:
            listen_keyboard(
                on_press=self._on_press,
                on_release=self._on_release,
                until=None,  # Don't stop automatically
                sequential=False,  # Allow concurrent callbacks
            )
        except Exception as e:
            logging.error(f"Keyboard listener error: {e}")
            self.listener_active = False

    def calibrate(self) -> None:
        pass

    def _on_press(self, key):
        """Callback when a key is pressed."""
        self.event_queue.put((key, True))

        # Check for ESC key to disconnect
        if key == "esc":
            logging.info("ESC pressed, disconnecting.")
            self.disconnect()

    def _on_release(self, key):
        """Callback when a key is released."""
        self.event_queue.put((key, False))

    def _drain_pressed_keys(self):
        """Process all pending key events from the queue."""
        while not self.event_queue.empty():
            key_char, is_pressed = self.event_queue.get_nowait()
            self.current_pressed[key_char] = is_pressed

    def configure(self):
        pass

    def get_action(self) -> dict[str, Any]:
        before_read_t = time.perf_counter()

        if not self.is_connected:
            raise DeviceNotConnectedError(
                "SshKeyboardTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()

        # Generate action based on current key states
        action = {key for key, val in self.current_pressed.items() if val}
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        return dict.fromkeys(action, None)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "SshKeyboardTeleop is not connected. You need to run `connect()` before `disconnect()`."
            )

        logging.info("Stopping keyboard listener...")
        self.listener_active = False

        # Stop the sshkeyboard listener
        if SSHKEYBOARD_AVAILABLE:
            try:
                stop_listening()
            except Exception as e:
                logging.warning(f"Error stopping listener: {e}")

        # Wait for thread to finish (with timeout)
        if self.listener_thread is not None:
            self.listener_thread.join(timeout=1.0)
            self.listener_thread = None


FPS = 30
DEADZONE = 0.15

JOINT_SO101ARM_TO_LEKIWI = {
    "shoulder_pan.pos": "arm_shoulder_pan.pos",
    "shoulder_lift.pos": "arm_shoulder_lift.pos",
    "elbow_flex.pos": "arm_elbow_flex.pos",
    "wrist_flex.pos": "arm_wrist_flex.pos",
    "wrist_roll.pos": "arm_wrist_roll.pos",
    "gripper.pos": "arm_gripper.pos",
}


async def send_so101_leader_keyboard(uri: str, so101_leader_port: str, so101_leader_id: str):
    # Initialize the leader arm
    teleop_config = SO101LeaderConfig(port=so101_leader_port, id=so101_leader_id)
    teleop_leader_arm = SO101Leader(teleop_config)
    teleop_leader_arm.connect()

    # Initialize keyboard
    teleop_keyboard = SshKeyboardTeleop(KeyboardTeleopConfig())
    teleop_keyboard.connect()

    async with websockets.connect(uri) as ws:
        # Disable Nagle on the client socket
        try:
            sock = ws.transport.get_extra_info("socket")
            if sock:
                import socket

                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass

        seq = 0
        rtts = []
        action = {
            "arm_shoulder_pan.pos": 0.0,
            "arm_shoulder_lift.pos": 0.0,
            "arm_elbow_flex.pos": 0.0,
            "arm_wrist_flex.pos": 0.0,
            "arm_wrist_roll.pos": 0.0,
            "arm_gripper.pos": 0.0,
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }

        while True:
            # >>> ------------------------------
            # Base Control via Keyboard
            delta_x, delta_y, theta_vel = 0.0, 0.0, 0.0
            for k, _ in teleop_keyboard.get_action().items():
                if k == 'd':
                    # right
                    delta_x += 1.0
                elif k == 'a':
                    # left
                    delta_x -= 1.0
                elif k == 'w':
                    # up
                    delta_y += 1.0
                elif k == 's':
                    # down
                    delta_y -= 1.0
                elif k == 'e':
                    # clockwise
                    theta_vel -= 1.0
                elif k == 'q':
                    # counter-clockwise
                    theta_vel += 1.0

            x_vel = delta_y
            y_vel = -delta_x
            action["x.vel"] = x_vel * 0.2  # Scale for safety
            action["y.vel"] = y_vel * 0.2  # Scale for safety
            action["theta.vel"] = theta_vel * 30  # Scale for safety
            # <<< ------------------------------


            # >>> ------------------------------
            # Arm Control via SO101 Leader Arm
            arm_action = teleop_leader_arm.get_action()
            for k, v in arm_action.items():
                lekiwi_key = JOINT_SO101ARM_TO_LEKIWI[k]
                action[lekiwi_key] = v
            # <<< ------------------------------

            t_send = time.perf_counter()
            msg = {
                "seq": seq,
                "t_send": t_send,
                **action,
            }
            await ws.send(json.dumps(msg))
            seq += 1

            # check for immediate pong messages without blocking the loop
            try:
                # use short timeout to poll for pong
                pong = await asyncio.wait_for(ws.recv(), timeout=0.001)
                try:
                    data = json.loads(pong)
                    if "echo_seq" in data and data.get("t_send") is not None:
                        rtt = time.perf_counter() - float(data["t_send"])
                        rtts.append(rtt)
                        if len(rtts) >= 100:
                            import statistics

                            print(
                                "RTT stats (s): min/median/90th/max: ",
                                min(rtts),
                                statistics.median(rtts),
                                sorted(rtts)[int(0.9 * len(rtts))],
                                max(rtts),
                            )
                            rtts = []
                except Exception:
                    pass
            except asyncio.TimeoutError:
                pass

            await asyncio.sleep(1.0 / FPS)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--teleop_port", default="/dev/ttyACM0", type=str)
    parser.add_argument("--teleop_id", default="my_awesome_so101_leader_arm", type=str)
    args = parser.parse_args()
    asyncio.run(send_so101_leader_keyboard(f"ws://{args.host}:{args.port}", args.teleop_port, args.teleop_id))
