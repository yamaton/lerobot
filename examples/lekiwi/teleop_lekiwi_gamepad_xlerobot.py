"""
Teleoperate LeKiwi using XLeRobot's kinematics

# To Run the teleop:
PYTHONPATH=src python -m examples.lekiwi.teleop_lekiwi_gamepad_xlerobot
"""

import argparse
import logging
import time

import pygame

from lerobot.robots.lekiwi.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.model.SO101Robot import SO101Kinematics


# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

FPS = 30
DEADZONE = 0.15

# GLOBAL VARIABLE
CURRENT_BASE_SPEED_LEVEL = 1.0

# Keymaps (semantic action: controller mapping) - Intuitive human control
KEYMAP = {
    # Left stick controls left arm XY (when not pressed)
    "x+": "left_stick_right",
    "x-": "left_stick_left",
    "y+": "left_stick_up",
    "y-": "left_stick_down",
    # Left stick pressed controls left arm shoulder_pan
    "shoulder_pan+": "right_stick_right",
    "shoulder_pan-": "right_stick_left",
    # LB pressed controls left arm pitch and wrist_roll
    "pitch+": "right_stick_up",
    "pitch-": "right_stick_down",
    "wrist_roll+": "x",
    "wrist_roll-": "a",
    # Left trigger controls left gripper
    "gripper": "left_trigger",
}

# Global reset key for all components
RESET_KEY = "back"

JOINT_MAP = {
    "shoulder_pan": "arm_shoulder_pan",
    "shoulder_lift": "arm_shoulder_lift",
    "elbow_flex": "arm_elbow_flex",
    "wrist_flex": "arm_wrist_flex",
    "wrist_roll": "arm_wrist_roll",
    "gripper": "arm_gripper",
}


class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=1):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"arm_wrist_roll.pos"],
            "gripper": initial_obs[f"arm_gripper.pos"],
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 2
        self.xy_step = 0.005
        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }

    def move_to_zero_position(self, robot):
        print(f"Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = (
            self.zero_pos.copy()
        )  # Use copy to avoid reference issues

        # Reset kinematic variables to their initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0

        # Don't let handle_keys recalculate wrist_flex - set it explicitly
        self.target_positions["wrist_flex"] = 0.0

        arm_action = self.p_control_action(robot)
        base_action = {
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }
        robot.send_action({**arm_action, **base_action})

    def handle_keys(self, key_state):
        # Joint increments
        if key_state.get("shoulder_pan+"):
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get("shoulder_pan-"):
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get("wrist_roll+"):
            self.target_positions["wrist_roll"] += self.degree_step
            print(f"wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get("wrist_roll-"):
            self.target_positions["wrist_roll"] -= self.degree_step
            print(f"wrist_roll: {self.target_positions['wrist_roll']}")

        # Gripper control with auto-close functionality
        if key_state.get("gripper") > 0:
            # Trigger pressed - open gripper (0.1)
            self.target_positions["gripper"] = 2
            print(f"gripper: CLOSED: {self.target_positions['gripper']:.1f}")
        else:
            self.target_positions["gripper"] = 90
            print(f"gripper: OPEN: {self.target_positions['gripper']:.1f}")

        if key_state.get("pitch+"):
            self.pitch += self.degree_step
            print(f"pitch: {self.pitch}")
        if key_state.get("pitch-"):
            self.pitch -= self.degree_step
            print(f"pitch: {self.pitch}")

        # XY plane (IK)
        moved = False
        if key_state.get("x+"):
            self.current_x += self.xy_step
            moved = True
            print(f"x+: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get("x-"):
            self.current_x -= self.xy_step
            moved = True
            print(f"x-: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get("y+"):
            self.current_y += self.xy_step
            moved = True
            print(f"x: {self.current_x:.4f}, y+: {self.current_y:.4f}")
        if key_state.get("y-"):
            self.current_y -= self.xy_step
            moved = True
            print(f"x: {self.current_x:.4f}, y-: {self.current_y:.4f}")
        if moved:
            joint2, joint3 = self.kinematics.inverse_kinematics(
                self.current_x, self.current_y
            )
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
            print(f"shoulder_lift: {joint2}, elbow_flex: {joint3}")

        # Wrist flex is always coupled to pitch and the other two
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            - self.target_positions["elbow_flex"]
            + self.pitch
        )
        # print(f"wrist_flex: {self.target_positions['wrist_flex']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


# --- XBOX Controller Mapping ---
def get_xbox_key_state(joystick, keymap):
    """
    Map XBOX controller state to semantic action booleans using the provided keymap.
    """
    # Read axes, buttons, hats
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    hats = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)

    # Get stick pressed states
    left_stick_pressed = bool(buttons[9]) if len(buttons) > 9 else False
    lb_pressed = bool(buttons[4]) if len(buttons) > 4 else False

    # Map controller state to semantic actions
    state = {}
    for action, control in keymap.items():
        if control == "left_trigger":
            state[action] = axes[2]
        elif control == "right_trigger":
            state[action] = axes[5] > 0.5 if len(axes) > 5 else False
        elif control == "a":
            state[action] = bool(buttons[0])
        elif control == "b":
            state[action] = bool(buttons[1])
        elif control == "x":
            state[action] = bool(buttons[2])
        elif control == "y":
            state[action] = bool(buttons[3])
        elif control == "back":
            state[action] = bool(buttons[6])
        elif control == "dpad_up":
            state[action] = hats[1] == 1
        elif control == "dpad_down":
            state[action] = hats[1] == -1
        elif control == "dpad_left":
            state[action] = hats[0] == -1
        elif control == "dpad_right":
            state[action] = hats[0] == 1
        # Left stick controls (when not pressed)
        elif control == "left_stick_up":
            state[action] = (
                (not left_stick_pressed) and (axes[1] < -0.3)
                if len(axes) > 1
                else False
            )
        elif control == "left_stick_down":
            state[action] = (
                (not left_stick_pressed) and (axes[1] > 0.3) if len(axes) > 1 else False
            )
        elif control == "left_stick_left":
            state[action] = (
                (not left_stick_pressed) and (axes[0] < -0.3)
                if len(axes) > 0
                else False
            )
        elif control == "left_stick_right":
            state[action] = (
                (not left_stick_pressed) and (axes[0] > 0.3) if len(axes) > 0 else False
            )
        # Left stick pressed controls
        elif control == "right_stick_right":
            state[action] = axes[3] > 0.3 if len(axes) > 0 else False
        elif control == "right_stick_left":
            state[action] = axes[3] < -0.3 if len(axes) > 0 else False
        elif control == "right_stick_up":
            state[action] = axes[4] > 0.3 if len(axes) > 4 else False
        elif control == "right_stick_down":
            state[action] = axes[4] < -0.3 if len(axes) > 4 else False
        else:
            state[action] = False
    return state


def get_base_action(joystick):
    """
    Get base action from XBOX controller input - simplified to only forward/backward and rotate.
    """
    base_action = {
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }

    # Read controller state
    left_bumper = joystick.get_button(4)
    right_bumper = joystick.get_button(5)
    dpad_x, dpad_y = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)

    base_x_vel = dpad_y
    base_y_vel = -dpad_x
    base_theta_vel = 0.0

    if left_bumper:
        base_theta_vel = 1.0
    elif right_bumper:
        base_theta_vel = -1.0

    base_action["x.vel"] = base_x_vel * 0.2  # Scale for safety
    base_action["y.vel"] = base_y_vel * 0.2  # Scale for safety
    base_action["theta.vel"] = base_theta_vel * 30  # Scale for safety

    return base_action


def main():
    parser = argparse.ArgumentParser(
        description="Teleoperate the LeKiwi robot directly with a gamepad."
    )
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

    FPS = 30
    robot_config = LeKiwiConfig(port=args.port, id=args.robot_id, cameras={})
    robot = LeKiwi(robot_config)
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        return

    # Init XBOX controller
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No XBOX controller detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[MAIN] Using controller: {joystick.get_name()}")

    # Init the arm and head instances
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    arm = SimpleTeleopArm(kin_left, JOINT_MAP, obs, prefix="left")

    # Move both arms and head to zero position at start
    arm.move_to_zero_position(robot)

    try:
        while True:
            try:
                t0 = time.perf_counter()
                pygame.event.pump()
                left_key_state = get_xbox_key_state(joystick, KEYMAP)

                # Check for global reset (back button)
                buttons = [
                    joystick.get_button(i) for i in range(joystick.get_numbuttons())
                ]
                global_reset = bool(buttons[6]) if len(buttons) > 6 else False

                # Handle global reset for all components
                if global_reset:
                    print("[MAIN] Global reset triggered!")
                    arm.move_to_zero_position(robot)
                    continue

                # Handle both arms separately and simultaneously
                arm.handle_keys(left_key_state)

                left_action = arm.p_control_action(robot)

                # Get base action and speed control from controller
                base_action = get_base_action(joystick)

                # Merge all actions
                action = {**left_action, **base_action}
                robot.send_action(action)

                busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

                obs = robot.get_observation()

            except ConnectionError as e:
                logger.warning(
                    f"A communication error occurred: {e}. Skipping this cycle."
                )
                time.sleep(0.1)
                continue
    finally:
        print("Disconnecting robot and gamepad...")
        robot.disconnect()
        pygame.quit()


if __name__ == "__main__":
    main()
