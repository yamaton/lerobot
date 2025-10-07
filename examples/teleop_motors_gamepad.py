#!/usr/bin/env python
"""
A low-level script to control up to 9 individual FeeTech motors using a gamepad.

This script bypasses the high-level Robot classes to send `Goal_Position` commands
directly to motors by their ID. It uses a proportional controller for smooth movements.

GAMEPAD MAPPING (XBox Controller Style):
- Motor 1: Left Stick Up/Down
- Motor 2: Left Stick Left/Right
- Motor 3: Right Stick Up/Down
- Motor 4: Right Stick Left/Right
- Motor 5: Right Trigger (positive) / Left Trigger (negative)
- Motor 6: D-Pad Up/Down
- Motor 7: D-Pad Left/Right
- Motor 8: Right Bumper (positive) / Left Bumper (negative)
- Motor 9: 'B' button (positive) / 'A' button (negative)

- Start Button: Reset all motors to position 0.
- Back/Select Button: Exit the program.
"""

import argparse
import logging
import time
import traceback

import pygame

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.utils.utils import init_logging

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# --- Constants ---
FPS = 50  # Control frequency in Hz
KP = 0.1  # Proportional gain for the P-controller
DEADZONE = 0.15  # Deadzone for joystick axes
SPEED_FACTOR = 1.5  # Multiplier for joystick sensitivity
MAX_MOTORS_TO_CONTROL = 9


def p_control_loop(bus, joystick):
    """Main proportional control loop for motor teleoperation."""
    control_period = 1.0 / FPS
    active_motor_ids = []

    # Ping motors to see which ones are active
    print("Pinging motor IDs 1-9...")
    for motor_id in range(1, MAX_MOTORS_TO_CONTROL + 1):
        if bus.ping(motor_id):
            active_motor_ids.append(motor_id)
            print(f"  - Motor ID {motor_id} is active.")

    if not active_motor_ids:
        print("No active motors found. Exiting.")
        return

    # Initialize target positions with the motors' current states
    try:
        initial_positions = bus.sync_read("Present_Position", ids=active_motor_ids)
        target_positions = {motor_id: pos for motor_id, pos in initial_positions.items()}
        logger.info(f"Initial target positions: {target_positions}")
    except Exception as e:
        logger.error(f"Failed to read initial motor positions: {e}. Starting at 0.")
        target_positions = {motor_id: 0.0 for motor_id in active_motor_ids}

    print("\nStarting teleoperation loop. Press Back/Select button to exit.")

    while True:
        start_time = time.time()
        pygame.event.pump()

        # --- Read Gamepad Inputs ---
        inputs = {
            1: -joystick.get_axis(1),  # Motor 1: Left Stick Y (inverted)
            2: joystick.get_axis(0),  # Motor 2: Left Stick X
            3: -joystick.get_axis(4),  # Motor 3: Right Stick Y (inverted)
            4: joystick.get_axis(3),  # Motor 4: Right Stick X
            5: (joystick.get_axis(5) + 1) / 2 - (joystick.get_axis(2) + 1) / 2,  # Motor 5: RT - LT
            6: joystick.get_hat(0)[1],  # Motor 6: D-Pad Y
            7: joystick.get_hat(0)[0],  # Motor 7: D-Pad X
            8: joystick.get_button(5) - joystick.get_button(4),  # Motor 8: RB - LB
            9: joystick.get_button(1) - joystick.get_button(0),  # Motor 9: B - A
        }

        # Exit button
        if joystick.get_button(6):  # Back/Select button
            print("Exit button pressed. Shutting down.")
            break

        # Homing button
        if joystick.get_button(7):  # Start button
            print("Homing button pressed. Resetting all motor targets to 0.")
            for motor_id in target_positions:
                target_positions[motor_id] = 0.0

        # --- Update Target Positions ---
        for motor_id, value in inputs.items():
            if motor_id in target_positions and abs(value) > DEADZONE:
                target_positions[motor_id] += value * SPEED_FACTOR

        # --- P-Control Calculation ---
        try:
            current_positions = bus.sync_read("Present_Position", ids=active_motor_ids)
            goal_positions = {}
            for motor_id, target_pos in target_positions.items():
                current_pos = current_positions.get(motor_id, target_pos)
                error = target_pos - current_pos
                control_output = KP * error
                goal_positions[motor_id] = current_pos + control_output

            bus.sync_write("Goal_Position", goal_positions)

        except ConnectionError as e:
            logger.warning(f"Communication error: {e}. Skipping cycle.")
        except Exception as e:
            logger.error(f"An unexpected error occurred: {e}")
            traceback.print_exc()
            break

        # Maintain control frequency
        elapsed_time = time.time() - start_time
        sleep_time = control_period - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)


def main():
    """Main function to set up and run the motor teleoperation."""
    init_logging()
    parser = argparse.ArgumentParser(description="Control individual FeeTech motors with a gamepad.")
    parser.add_argument("--port", type=str, required=True, help="Serial port for the motor bus.")
    args = parser.parse_args()

    # --- Pygame and Joystick Initialization ---
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No gamepad detected! Please connect a gamepad.")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Gamepad detected: {joystick.get_name()}")

    # --- Motor Bus Initialization ---
    # Instantiate the bus with an empty motors dict to enable low-level communication
    bus = FeetechMotorsBus(port=args.port, motors={})
    try:
        bus.connect()
        p_control_loop(bus, joystick)
    except Exception as e:
        logger.error(f"Failed to initialize or run control loop: {e}")
        traceback.print_exc()
    finally:
        if bus.is_connected:
            bus.disconnect()
        pygame.quit()
        print("Program finished.")

if __name__ == "__main__":
    main()
