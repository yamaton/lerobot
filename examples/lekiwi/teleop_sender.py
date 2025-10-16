"""
Sending robot-teleoperation signal over WebSocket

How to run:

1. Run the receiver (host) script

    python -m examples.lekiwi.teleop_receiver

2. Run the sender (client) script

    python -m examples.lekiwi.teleop_sender --host <host-ip-address>

"""

import asyncio
import json
import time
import argparse

import pygame
import websockets

FPS = 30
DEADZONE = 0.15


async def send_gamepad(uri: str):
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No gamepad detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Gamepad:", joystick.get_name())

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
        while True:
            pygame.event.pump()
            left_stick_x = joystick.get_axis(0)
            left_stick_y = joystick.get_axis(1)
            right_stick_y = joystick.get_axis(4)
            left_trigger = (joystick.get_axis(2) + 1) / 2
            right_trigger = (joystick.get_axis(5) + 1) / 2
            left_bumper = joystick.get_button(4)
            right_bumper = joystick.get_button(5)
            dpad_x, dpad_y = joystick.get_hat(0)

            base_x_vel = dpad_y
            base_y_vel = -dpad_x
            base_theta_vel = 0.0
            if left_bumper:
                base_theta_vel = 1.0
            elif right_bumper:
                base_theta_vel = -1.0

            delta_x = -left_stick_y if abs(left_stick_y) > DEADZONE else 0.0
            delta_y = -left_stick_x if abs(left_stick_x) > DEADZONE else 0.0
            delta_z = -right_stick_y if abs(right_stick_y) > DEADZONE else 0.0
            gripper = right_trigger - left_trigger

            t_send = time.perf_counter()
            msg = {
                "seq": seq,
                "t_send": t_send,
                "delta_x": delta_x,
                "delta_y": delta_y,
                "delta_z": delta_z,
                "gripper": gripper,
                "base_x_vel": base_x_vel,
                "base_y_vel": base_y_vel,
                "base_theta_vel": base_theta_vel,
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
    args = parser.parse_args()
    asyncio.run(send_gamepad(f"ws://{args.host}:{args.port}"))
