import asyncio
import json
import logging
import time
import socket

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

import websockets

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TeleopReceiver")

FPS = 30

# shared latest state (thread-safe access via asyncio.Lock)
latest_state = None
latest_lock = asyncio.Lock()


async def websocket_handler(websocket) -> None:
    global latest_state
    # disable Nagle on server side socket
    try:
        sock = websocket.transport.get_extra_info("socket")
        if sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    print("Client connected")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except Exception:
                continue

            # update latest_state quickly and return
            async with latest_lock:
                latest_state = data

            # echo minimal pong so sender can measure RTT
            pong = {"echo_seq": data.get("seq"), "t_send": data.get("t_send")}
            try:
                await websocket.send(json.dumps(pong))
            except Exception:
                pass

    except websockets.ConnectionClosed:
        print("Client disconnected")


async def control_loop(robot, arm_pipeline):
    global latest_state
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

    while True:
        t0 = time.perf_counter()
        async with latest_lock:
            state = latest_state

        if state is not None:
            # build gamepad_action
            gamepad_action = {
                "delta_x": float(state.get("delta_x", 0.0)),
                "delta_y": float(state.get("delta_y", 0.0)),
                "delta_z": float(state.get("delta_z", 0.0)),
                "gripper": float(state.get("gripper", 0.0)),
            }

            # run blocking robot_obs + pipeline in a thread so we don't block event loop
            def blocking_cycle():
                obs = robot.get_observation()
                arm_action = arm_pipeline((gamepad_action, obs))
                return obs, arm_action

            obs, arm_action = await asyncio.to_thread(blocking_cycle)

            for k, v in arm_action.items():
                full_action[k] = v

            # base velocities
            full_action["x.vel"] = float(state.get("base_x_vel", 0.0)) * 0.2
            full_action["y.vel"] = float(state.get("base_y_vel", 0.0)) * 0.2
            full_action["theta.vel"] = float(state.get("base_theta_vel", 0.0)) * 30

            # send_action may block too -> run in thread
            await asyncio.to_thread(robot.send_action, full_action)

        # enforce loop rate
        elapsed = time.perf_counter() - t0
        sleep_for = max(0.0, 1.0 / FPS - elapsed)
        if sleep_for > 0.0:
            await asyncio.sleep(sleep_for)


async def main():
    # init robot and pipeline
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
    server = await websockets.serve(websocket_handler, "127.0.0.1", 8765)
    print("Websocket server listening at ws://127.0.0.1:8765")

    # start control loop
    loop = asyncio.get_running_loop()
    loop.create_task(control_loop(robot, arm_pipeline))

    await server.wait_closed()


if __name__ == "__main__":
    asyncio.run(main())
