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

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    MapDeltaActionToRobotActionStep,
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
)
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.lekiwi.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.utils.robot_utils import busy_wait

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

FPS = 30
DEADZONE = 0.15


def main():
    parser = argparse.ArgumentParser(description="Teleoperate the LeKiwi robot directly with a gamepad.")
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="The serial port of the LeKiwi robot.",
    )
    parser.add_argument(
        "--robot-id",
        type=str,
        default="my_awesome_lekiwi",
        help="The ID of the LeKiwi robot to connect to.",
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
    robot_config = LeKiwiConfig(port=args.port, id=args.robot_id, cameras={})
    robot = LeKiwi(robot_config)

    # Initialize the arm pipeline
    kinematics_solver = RobotKinematics(
        urdf_path="./SO101/lekiwi_so101_new_calib.urdf",
        target_frame_name="gripper_frame_link",
        joint_names=[
            "arm_shoulder_pan",
            "arm_shoulder_lift",
            "arm_elbow_flex",
            "arm_wrist_flex",
            "arm_wrist_roll",
            "arm_gripper",
        ],
    )

    arm_pipeline = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        [
            MapDeltaActionToRobotActionStep(),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.01, "y": 0.01, "z": 0.01},
                motor_names=list(kinematics_solver.joint_names),
                use_latched_reference=False,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={
                    "min": [-1.0, -1.0, -1.0],
                    "max": [1.0, 1.0, 1.0],
                },
                max_ee_step_m=0.05,
            ),
            GripperVelocityToJoint(
                speed_factor=20.0,
            ),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(kinematics_solver.joint_names),
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    # Connect to the robot
    robot.connect()

    print("Starting teleop loop...")
    full_action = {
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
                right_stick_y = joystick.get_axis(4)
                left_trigger = (joystick.get_axis(2) + 1) / 2
                right_trigger = (joystick.get_axis(5) + 1) / 2
                # Buttons
                left_bumper = joystick.get_button(4)
                right_bumper = joystick.get_button(5)
                # D-Pad
                dpad_x, dpad_y = joystick.get_hat(0)

                # --- Base Control ---
                base_x_vel = dpad_y
                base_y_vel = -dpad_x
                base_theta_vel = 0.0
                if left_bumper:
                    base_theta_vel = 1.0
                elif right_bumper:
                    base_theta_vel = -1.0

                full_action["x.vel"] = base_x_vel * 0.2  # Scale for safety
                full_action["y.vel"] = base_y_vel * 0.2  # Scale for safety
                full_action["theta.vel"] = base_theta_vel * 30  # Scale for safety

                # --- Arm Control ---
                delta_x = -left_stick_y if abs(left_stick_y) > DEADZONE else 0.0
                delta_y = -left_stick_x if abs(left_stick_x) > DEADZONE else 0.0
                delta_z = -right_stick_y if abs(right_stick_y) > DEADZONE else 0.0

                gripper_action = right_trigger - left_trigger

                gamepad_action = {
                    "delta_x": delta_x,
                    "delta_y": delta_y,
                    "delta_z": delta_z,
                    "gripper": gripper_action,
                }

                robot_obs = robot.get_observation()
                arm_action = arm_pipeline((gamepad_action, robot_obs))
                for key, value in arm_action.items():
                    full_action[key] = value

                # --- Send Action to Robot ---
                robot.send_action(full_action)

                busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

            except ConnectionError as e:
                logger.warning(f"A communication error occurred: {e}. Skipping this cycle.")
                time.sleep(0.1)
                continue

    except KeyboardInterrupt:
        print("User interrupted program.")

    finally:
        print("Disconnecting robot and gamepad...")
        robot.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()