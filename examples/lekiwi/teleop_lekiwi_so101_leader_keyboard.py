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

from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.robots.lekiwi.lekiwi import LeKiwi, LeKiwiConfig

from lerobot.utils.robot_utils import busy_wait


# Setup logging
logging.basicConfig(
    level=logging.DEBUG, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
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

    # Initialize the robot
    robot_config = LeKiwiConfig(port=args.robot_port, id=args.robot_id, cameras={})
    robot = LeKiwi(robot_config)
    robot.connect()

    # Initialize the leader arm
    teleop_config = SO101LeaderConfig(port=args.teleop_port, id=args.teleop_id)
    teleop_leader_arm = SO101Leader(teleop_config)
    teleop_leader_arm.connect()

    # Initialize keyboard
    teleop_keyboard = KeyboardTeleop(KeyboardTeleopConfig())
    teleop_keyboard.connect()


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

                action_keyboard = teleop_keyboard.get_action()
                delta_x = 0.0
                delta_y = 0.0
                theta_vel = 0.0

                for k, _ in action_keyboard.items():
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

                # --- Base Control ---
                base_x_vel = delta_y
                base_y_vel = -delta_x
                action["x.vel"] = base_x_vel * 0.2  # Scale for safety
                action["y.vel"] = base_y_vel * 0.2  # Scale for safety
                action["theta.vel"] = theta_vel * 30  # Scale for safety

                # add arm actions
                arm_action = teleop_leader_arm.get_action()
                assert set(arm_action) == set(JOINT_SO101ARM_TO_LEKIWI)
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
        teleop_keyboard.disconnect()
        teleop_leader_arm.disconnect()


if __name__ == "__main__":
    main()
