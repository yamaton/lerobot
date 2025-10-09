"""
Teleoperate LeKiwi

1. Run the host script: In a separate terminal, run the LeKiwi host:

   python -m examples.lekiwi.lekiwi_host_no_camera

2. Run the teleop script: In another terminal, run the new teleop script:

   python -m examples.lekiwi.teleop_lekiwi_gamepad

"""

import argparse
import logging
import time
from enum import Enum

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
from lerobot.robots.lekiwi.lekiwi_client import LeKiwiClient, LeKiwiClientConfig
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.teleoperators.gamepad import GamepadTeleop, GamepadTeleopConfig
from lerobot.utils.robot_utils import busy_wait

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

FPS = 30


class ControlMode(Enum):
    ARM = 1
    BASE = 2


def main():
    parser = argparse.ArgumentParser(description="Teleoperate the LeKiwi robot with a gamepad.")
    parser.add_argument(
        "--remote-ip",
        type=str,
        default="127.0.0.1",
        help="The remote IP address of the LeKiwi host.",
    )
    parser.add_argument(
        "--robot-id",
        type=str,
        default="my_lekiwi",
        help="The ID of the LeKiwi robot to connect to.",
    )
    args = parser.parse_args()

    # Initialize the robot and teleoperator config
    robot_config = LeKiwiClientConfig(remote_ip=args.remote_ip, id=args.robot_id, cameras={})
    gamepad_config = GamepadTeleopConfig()

    # Initialize the robot and teleoperator
    robot = LeKiwiClient(robot_config)
    gamepad = GamepadTeleop(gamepad_config)

    # NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
    kinematics_solver = RobotKinematics(
        urdf_path="./SO101/so101_new_calib.urdf",
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

    # build pipeline to convert EE action to robot joints
    arm_pipeline = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        [
            MapDeltaActionToRobotActionStep(),
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.01, "y": 0.01, "z": 0.01},
                motor_names=list(kinematics_solver.joint_names),
                use_latched_reference=False,  # if False, uses the current pose as reference
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=0.05,
            ),
            GripperVelocityToJoint(
                speed_factor=20.0,
            ),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(kinematics_solver.joint_names),
                initial_guess_current_joints=True,  # if False, uses the solution from the previous step
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    def map_gamepad_to_base_velocity(
        action: RobotAction, xy_speed: float = 0.2, theta_speed: float = 60.0
    ) -> dict:
        x_cmd = -action["delta_y"] * xy_speed
        y_cmd = -action["delta_x"] * xy_speed
        theta_cmd = -action["delta_z"] * theta_speed
        return {"x.vel": x_cmd, "y.vel": y_cmd, "theta.vel": theta_cmd}

    # Connect to the robot and teleoperator
    robot.connect()
    gamepad.connect()

    # Initialize pygame for button press detection
    pygame.init()

    print("Starting teleop loop...")
    print("Press the 'A' button to switch between ARM and BASE control modes.")

    current_mode = ControlMode.ARM
    mode_switch_pressed = False

    try:
        while True:
            try:
                t0 = time.perf_counter()

                # Check for mode switch
                pygame.event.pump()
                # Assuming joystick is already initialized by gamepad.connect()
                joystick = pygame.joystick.Joystick(0)
                if joystick.get_button(0):  # 'A' button
                    if not mode_switch_pressed:
                        current_mode = (
                            ControlMode.BASE
                            if current_mode == ControlMode.ARM
                            else ControlMode.ARM
                        )
                        print(f"Switched to {current_mode.name} mode.")
                        mode_switch_pressed = True
                else:
                    mode_switch_pressed = False

                # Get robot observation
                robot_obs = robot.get_observation()

                # Get teleop observation
                gamepad_action = gamepad.get_action()

                action = {}
                if current_mode == ControlMode.ARM:
                    # Process arm action
                    action = arm_pipeline((gamepad_action, robot_obs))
                else:  # current_mode == ControlMode.BASE
                    # Process base action
                    action = map_gamepad_to_base_velocity(gamepad_action)

                # Send action to robot
                if action:
                    _ = robot.send_action(action)

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
        gamepad.disconnect()
        pygame.quit()

if __name__ == "__main__":
    main()
