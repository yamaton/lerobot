"""
Teleoperation Receiver for LeKiwi Robot (WebSocket version)
Receives gamepad input over WebSocket and commands the LeKiwi robot.
"""

import asyncio
import json
import logging
import time
import websockets

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

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TeleopReceiver")

FPS = 30

async def handle_client(websocket):
    print("Client connected.")

    # --- Initialize Robot ---
    robot_config = LeKiwiConfig(port="/dev/ttyACM0", id="my_awesome_lekiwi", cameras={})
    robot = LeKiwi(robot_config)

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
                end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
                max_ee_step_m=0.05,
            ),
            GripperVelocityToJoint(speed_factor=20.0),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(kinematics_solver.joint_names),
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    robot.connect()

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
        async for message in websocket:
            t0 = time.perf_counter()
            data = json.loads(message)

            gamepad_action = {
                "delta_x": data["delta_x"],
                "delta_y": data["delta_y"],
                "delta_z": data["delta_z"],
                "gripper": data["gripper"],
            }

            robot_obs = robot.get_observation()
            arm_action = arm_pipeline((gamepad_action, robot_obs))
            for key, value in arm_action.items():
                full_action[key] = value

            full_action["x.vel"] = data["base_x_vel"] * 0.2
            full_action["y.vel"] = data["base_y_vel"] * 0.2
            full_action["theta.vel"] = data["base_theta_vel"] * 30

            robot.send_action(full_action)
            busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

    except websockets.ConnectionClosed:
        print("Client disconnected.")
    finally:
        robot.disconnect()


async def main():
    server = await websockets.serve(handle_client, "0.0.0.0", 8765)
    print("WebSocket receiver running at ws://0.0.0.0:8765")
    await server.wait_closed()


if __name__ == "__main__":
    asyncio.run(main())
