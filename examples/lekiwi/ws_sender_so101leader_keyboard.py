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

from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
import websockets

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
    teleop_keyboard = KeyboardTeleop(KeyboardTeleopConfig())
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
