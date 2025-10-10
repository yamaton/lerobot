"""
Teleoperation Sender for LeKiwi Robot (WebSocket version)
Reads gamepad input and sends it to a remote robot receiver via WebSocket.
"""

import asyncio
import json
import pygame
import websockets
import argparse

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
    print(f"Gamepad detected: {joystick.get_name()}")

    async with websockets.connect(uri) as websocket:
        print(f"Connected to {uri}")
        while True:
            pygame.event.pump()

            # --- Read Gamepad Inputs ---
            left_stick_x = joystick.get_axis(0)
            left_stick_y = joystick.get_axis(1)
            right_stick_y = joystick.get_axis(4)
            left_trigger = (joystick.get_axis(2) + 1) / 2
            right_trigger = (joystick.get_axis(5) + 1) / 2
            left_bumper = joystick.get_button(4)
            right_bumper = joystick.get_button(5)
            dpad_x, dpad_y = joystick.get_hat(0)

            # Base control
            base_x_vel = dpad_y
            base_y_vel = -dpad_x
            base_theta_vel = 0.0
            if left_bumper:
                base_theta_vel = 1.0
            elif right_bumper:
                base_theta_vel = -1.0

            # Arm deltas
            delta_x = -left_stick_y if abs(left_stick_y) > DEADZONE else 0.0
            delta_y = -left_stick_x if abs(left_stick_x) > DEADZONE else 0.0
            delta_z = -right_stick_y if abs(right_stick_y) > DEADZONE else 0.0
            gripper = right_trigger - left_trigger

            # Pack into a JSON message
            msg = json.dumps({
                "delta_x": delta_x,
                "delta_y": delta_y,
                "delta_z": delta_z,
                "gripper": gripper,
                "base_x_vel": base_x_vel,
                "base_y_vel": base_y_vel,
                "base_theta_vel": base_theta_vel,
            })

            await websocket.send(msg)
            await asyncio.sleep(1.0 / FPS)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Robot receiver host IP")
    parser.add_argument("--port", type=int, default=8765, help="WebSocket port")
    args = parser.parse_args()

    uri = f"ws://{args.host}:{args.port}"
    asyncio.run(send_gamepad(uri))
