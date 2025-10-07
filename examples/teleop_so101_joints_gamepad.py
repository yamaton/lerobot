#!/usr/bin/env python
"""
This script allows direct joint control of a SO-101/SO-100 follower arm using a standard gamepad.
It uses a proportional controller for smooth movements.

GAMEPAD MAPPING (typical XInput controller like Xbox):
- Left Stick (X/Y): Controls shoulder_pan / shoulder_lift
- Right Stick (X/Y): Controls wrist_roll / wrist_flex
- Left/Right Triggers: Controls elbow_flex (LT: negative, RT: positive)
- Left/Right Bumpers: Controls gripper (LB: close, RB: open)
- 'A' Button: Move all joints to zero position (homing).
- 'B' Button: Exit the program.
"""

import time
import logging
import traceback
import pygame

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- Constants ---
FPS = 30  # Control frequency in Hz
KP = 0.1  # Proportional gain for the P-controller
DEADZONE = 0.15  # Deadzone for joystick axes to prevent drift
SPEED_FACTOR = 2.0  # General speed multiplier for joint movements

# --- Main Control Functions ---

def p_control_loop(robot, joystick):
    """
    Main proportional control loop for teleoperation.
    """
    control_period = 1.0 / FPS

    # Initialize target positions with the robot's current state
    try:
        initial_obs = robot.get_observation()
        target_positions = {
            'shoulder_pan': initial_obs.get('shoulder_pan.pos', 0.0),
            'shoulder_lift': initial_obs.get('shoulder_lift.pos', 0.0),
            'elbow_flex': initial_obs.get('elbow_flex.pos', 0.0),
            'wrist_flex': initial_obs.get('wrist_flex.pos', 0.0),
            'wrist_roll': initial_obs.get('wrist_roll.pos', 0.0),
            'gripper': initial_obs.get('gripper.pos', 0.0),
        }
        logger.info(f"Initial target positions set to: {target_positions}")
    except Exception as e:
        logger.error(f"Failed to get initial robot observation: {e}")
        return

    print("Starting teleoperation loop. Press 'B' button to exit.")

    while True:
        start_time = time.time()

        # Pump pygame events to handle OS messages
        pygame.event.pump()

        # --- Read Gamepad Inputs ---
        # Joysticks (typically range from -1.0 to 1.0)
        left_stick_x = joystick.get_axis(0)
        left_stick_y = joystick.get_axis(1)
        right_stick_x = joystick.get_axis(3)
        right_stick_y = joystick.get_axis(4)

        # Triggers can be a single combined axis or two separate axes depending on the OS.
        # This code assumes two separate axes (common on Linux).
        try:
            # Axes 2 and 5 are commonly LT and RT on Linux
            left_trigger = (joystick.get_axis(2) + 1) / 2  # Normalize to 0-1 range
            right_trigger = (joystick.get_axis(5) + 1) / 2 # Normalize to 0-1 range
            trigger_axis_val = right_trigger - left_trigger
        except pygame.error:
            logger.warning("Could not read trigger axes 2 and 5. Check controller configuration.")
            trigger_axis_val = 0.0

        # Buttons
        btn_a = joystick.get_button(0)
        btn_b = joystick.get_button(1)
        left_bumper = joystick.get_button(4)
        right_bumper = joystick.get_button(5)

        if btn_b: # Exit condition
            print("Exit button pressed. Shutting down.")
            break

        if btn_a: # Homing
            print("Homing button pressed. Moving to zero position.")
            target_positions = {name: 0.0 for name in target_positions}

        # --- Update Target Positions based on Gamepad Input ---

        # Apply deadzone to prevent drift from sensitive joysticks
        if abs(left_stick_x) > DEADZONE:
            target_positions['shoulder_pan'] -= left_stick_x * SPEED_FACTOR
        if abs(left_stick_y) > DEADZONE:
            target_positions['shoulder_lift'] -= left_stick_y * SPEED_FACTOR

        if abs(right_stick_x) > DEADZONE:
            target_positions['wrist_roll'] += right_stick_x * SPEED_FACTOR
        if abs(right_stick_y) > DEADZONE:
            target_positions['wrist_flex'] -= right_stick_y * SPEED_FACTOR

        if abs(trigger_axis_val) > DEADZONE:
            target_positions['elbow_flex'] += trigger_axis_val * SPEED_FACTOR * 1.5 # Elbow may need more speed

        # Bumper control for gripper
        gripper_delta = 0
        if right_bumper:
            gripper_delta = 1 * SPEED_FACTOR * 2.0  # Open
        elif left_bumper:
            gripper_delta = -1 * SPEED_FACTOR * 2.0 # Close
        target_positions['gripper'] += gripper_delta

        # Clamp gripper values to a safe range
        target_positions['gripper'] = max(-10, min(90, target_positions['gripper']))

        # --- P-Control Calculation ---
        try:
            current_obs = robot.get_observation()
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                # Use .get for safety in case a joint isn't reported
                current_pos = current_obs.get(f"{joint_name}.pos", target_pos)
                error = target_pos - current_pos

                # Proportional control output
                control_output = KP * error

                # The action is the desired new position
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position

            # Send the calculated action to the robot
            robot.send_action(robot_action)
        except ConnectionError as e:
            logger.warning(f"A communication error occurred: {e}. Skipping this cycle.")
            # Pass to ignore the error for this cycle and continue the loop
            pass
        except Exception as e:
            logger.error(f"An unexpected error occurred in control loop: {e}")
            traceback.print_exc()
            break

        # Maintain control frequency
        elapsed_time = time.time() - start_time
        sleep_time = control_period - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

def main():
    """Main function to set up and run the teleoperation."""
    print("SO-101 Gamepad Joint Control Example")
    print("="*50)

    # --- Pygame and Joystick Initialization ---
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No gamepad detected! Please connect a gamepad.")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Gamepad detected: {joystick.get_name()}")

    # --- Robot Initialization ---
    try:
        port = input("Enter robot USB port (e.g., /dev/ttyACM0) or press Enter for default: ").strip()
        if not port:
            port = "/dev/ttyACM0"
            logger.info(f"Using default port: {port}")

        robot_config = SO101FollowerConfig(port=port, id="teleop_so101_gamepad_joints")
        robot = SO101Follower(robot_config)

        robot.connect()

        if not robot.is_calibrated:
            print("Robot is not calibrated. Starting calibration...")
            robot.calibrate()
        else:
            print("Robot is already calibrated.")

    except Exception as e:
        print(f"Failed to initialize robot: {e}")
        traceback.print_exc()
        return

    # --- Start Control Loop ---
    try:
        p_control_loop(robot, joystick)
    finally:
        # --- Cleanup ---
        print("Disconnecting robot...")
        robot.disconnect()
        pygame.quit()
        print("Program finished.")

if __name__ == "__main__":
    main()
