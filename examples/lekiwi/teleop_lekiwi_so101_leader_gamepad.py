"""
Teleoperate LeKiwi Directly with Pygame

This script teleoperates the LeKiwi robot directly using the pygame library for gamepad input.
It allows for simultaneous control of the arm and the base.

GAMEPAD MAPPING (XBox Controller Style):
- Left Stick (X/Y): Arm end-effector forward/backward and left/right (X/Y-axis of arm)
- Right Stick (Y-axis): Arm end-effector up/down (Z-axis of arm)
- Left/Right Triggers: Gripper control
- D-Pad (X/Y): Base left/right and forward/backward velocity (Y/X-axis of robot)
- Left/Right Bumpers: Base rotational velocity (theta-axis of robot)

"""

import argparse
import logging
import time

import pygame

from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.robots.lekiwi.lekiwi import LeKiwi, LeKiwiConfig

from lerobot.utils.robot_utils import busy_wait

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

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


def main():
    parser = argparse.ArgumentParser(
        description="Teleoperate the LeKiwi robot directly with a gamepad."
    )
    parser.add_argument(
        "--robot-port",
        type=str,
        default="/dev/ttyACM0",
        help="The serial port of the LeKiwi robot motors.",
    )
    parser.add_argument(
        "--teleop-port",
        type=str,
        default="/dev/ttyACM1",
        help="The serial port of the SO101 Leader motors.",
    )
    parser.add_argument(
        "--robot-id",
        type=str,
        default="my_awesome_lekiwi",
        help="The ID of the LeKiwi robot.",
    )
    parser.add_argument(
        "--teleop-id",
        type=str,
        default="my_awesome_so101_leader_arm",
        help="The ID of the SO101 Leader Arm.",
    )
    args = parser.parse_args()

    # Initialize pygame and joystick
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No gamepad detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Gamepad detected: {joystick.get_name()}")

    # Initialize the robot
    robot_config = LeKiwiConfig(port=args.robot_port, id=args.robot_id, cameras={})
    robot = LeKiwi(robot_config)
    robot.connect()

    teleop_config = SO101LeaderConfig(port=args.teleop_port, id=args.teleop_id)
    teleop = SO101Leader(teleop_config)
    teleop.connect()

    print("Starting teleop loop...")
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

    try:
        while True:
            try:
                t0 = time.perf_counter()
                pygame.event.pump()

                # --- Read Gamepad Inputs ---
                # Axes
                left_stick_x = joystick.get_axis(0)
                left_stick_y = joystick.get_axis(1)
                # Buttons
                left_bumper = joystick.get_button(4)
                right_bumper = joystick.get_button(5)

                # --- Base Control ---
                # should be `x_vel = stick_y` but this works...
                base_x_vel = -left_stick_y
                base_y_vel = -left_stick_x
                base_theta_vel = 0.0
                if left_bumper:
                    base_theta_vel = 1.0
                elif right_bumper:
                    base_theta_vel = -1.0

                action["x.vel"] = base_x_vel * 0.2  # Scale for safety
                action["y.vel"] = base_y_vel * 0.2  # Scale for safety
                action["theta.vel"] = base_theta_vel * 30  # Scale for safety

                # add arm actions
                arm_action = teleop.get_action()
                for k, v in arm_action.items():
                    lekiwi_key = JOINT_SO101ARM_TO_LEKIWI[k]
                    action[lekiwi_key] = v

                # --- Send Action to Robot ---
                robot.send_action(action)

                busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

            except ConnectionError as e:
                logger.warning(
                    f"A communication error occurred: {e}. Skipping this cycle."
                )
                time.sleep(0.1)
                continue

    except KeyboardInterrupt:
        print("User interrupted program.")

    finally:
        print("Disconnecting robot and gamepad...")
        robot.disconnect()
        teleop.disconnect()
        pygame.quit()


if __name__ == "__main__":
    main()
