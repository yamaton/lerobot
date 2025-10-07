import logging
import time

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import MapDeltaActionToRobotActionStep, RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.teleoperators.gamepad import GamepadTeleop, GamepadTeleopConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

FPS = 30

# Initialize the robot and teleoperator config
follower_config = SO101FollowerConfig(port="/dev/ttyACM0", id="my_awesome_follower_arm", use_degrees=True)

# Initialize the robot and teleoperator
follower = SO101Follower(follower_config)
gamepad = GamepadTeleop(GamepadTeleopConfig())

# NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
follower_kinematics_solver = RobotKinematics(
    urdf_path="./SO101/so101_new_calib.urdf",
    target_frame_name="gripper_frame_link",
    joint_names=list(follower.bus.motors.keys()),
)

# build pipeline to convert EE action to robot joints
ee_to_follower_joints = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
    [
        MapDeltaActionToRobotActionStep(),
        EEReferenceAndDelta(
            kinematics=follower_kinematics_solver,
            end_effector_step_sizes={"x": 0.01, "y": 0.01, "z": 0.01},
            motor_names=list(follower.bus.motors.keys()),
            use_latched_reference=True,
        ),
        EEBoundsAndSafety(
            end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
            max_ee_step_m=0.05,
        ),
        GripperVelocityToJoint(
            speed_factor=20.0,
        ),
        InverseKinematicsEEToJoints(
            kinematics=follower_kinematics_solver,
            motor_names=list(follower.bus.motors.keys()),
            initial_guess_current_joints=True,
        ),
    ],
    to_transition=robot_action_observation_to_transition,
    to_output=transition_to_robot_action,
)

# Connect to the robot and teleoperator
follower.connect()
gamepad.connect()

# Init rerun viewer
init_rerun(session_name="teleop_so101_gamepad")

print("Starting teleop loop...")
try:
    while True:
        try:
            t0 = time.perf_counter()

            # Get robot observation
            robot_obs = follower.get_observation()

            # Get teleop observation
            gamepad_action = gamepad.get_action()

            # teleop EE -> robot joints
            follower_joints_act = ee_to_follower_joints((gamepad_action, robot_obs))

            # Send action to robot
            _ = follower.send_action(follower_joints_act)

            # Visualize
            log_rerun_data(observation=gamepad_action, action=follower_joints_act)

            busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
        except ConnectionError as e:
            logger.warning(f"A communication error occurred: {e}. Skipping this cycle.")
            time.sleep(0.1)
            continue
except KeyboardInterrupt:
    print("User interrupted program.")
finally:
    print("Disconnecting robot and gamepad...")
    follower.disconnect()
    gamepad.disconnect()
