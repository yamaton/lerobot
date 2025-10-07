#!/usr/bin/env python
"""
A low-level script to control up to 9 individual FeeTech motors using a gamepad.

This script bypasses the high-level Robot classes to send `Goal_Position` commands
directly to motors by their ID using the `scservo_sdk`. It uses a proportional controller
for smooth movements.

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
import sys
import time
import traceback

import pygame

try:
    from scservo_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
except ImportError:
    print("Error: scservo_sdk not found.")
    print("Please ensure the lerobot development environment is set up and sourced correctly.")
    sys.exit(1)

from lerobot.motors.feetech.tables import MODEL_CONTROL_TABLE
from lerobot.utils.utils import init_logging

# Setup logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# --- Constants ---
FPS = 50
KP = 0.5
DEADZONE = 0.15
SPEED_FACTOR = 5.0
MAX_MOTORS_TO_CONTROL = 9
PROTOCOL_VERSION = 0.0
BAUDRATE = 1_000_000

# Get register addresses and lengths from a sample control table (they are common for this purpose)
# We assume all motors are STS/SMS series, which is standard for lerobot SO-100/101.
CONTROL_TABLE = MODEL_CONTROL_TABLE["sts_series"]
ADDR_GOAL_POSITION, LEN_GOAL_POSITION = CONTROL_TABLE["Goal_Position"]
ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION = CONTROL_TABLE["Present_Position"]


def p_control_loop(port_handler, packet_handler, joystick):
    """Main proportional control loop for motor teleoperation."""
    control_period = 1.0 / FPS

    # --- Discover Active Motors ---
    active_motor_ids = []
    print("Pinging motor IDs 1-9...")
    for motor_id in range(1, MAX_MOTORS_TO_CONTROL + 1):
        _, comm_result, _ = packet_handler.ping(port_handler, motor_id)
        if comm_result == COMM_SUCCESS:
            active_motor_ids.append(motor_id)
            print(f"  - Motor ID {motor_id} is active.")

    if not active_motor_ids:
        print("No active motors found. Exiting.")
        return

    # --- Initialize GroupSyncRead and GroupSyncWrite ---
    group_sync_read = GroupSyncRead(port_handler, packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    for motor_id in active_motor_ids:
        group_sync_read.addParam(motor_id)

    group_sync_write = GroupSyncWrite(port_handler, packet_handler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # --- Initialize Target Positions ---
    try:
        # Set initial targets to the robot's current positions to prevent initial movement
        comm_result = group_sync_read.txRxPacket()
        if comm_result != COMM_SUCCESS:
            raise ConnectionError(packet_handler.getTxRxResult(comm_result))

        target_positions = {}
        for motor_id in active_motor_ids:
            pos = group_sync_read.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            target_positions[motor_id] = float(pos)
        logger.info(f"Initial target positions set to: {target_positions}")

    except Exception as e:
        logger.error(f"Failed to read initial motor positions: {e}. Starting all at 0.")
        target_positions = {motor_id: 0.0 for motor_id in active_motor_ids}

    print("\nStarting teleoperation loop. Press Back/Select button to exit.")

    while True:
        start_time = time.time()
        pygame.event.pump()

        # --- Read Gamepad and Update Targets ---
        inputs = {
            1: -joystick.get_axis(1), 2: joystick.get_axis(0),
            3: -joystick.get_axis(4), 4: joystick.get_axis(3),
            5: (joystick.get_axis(5) + 1) / 2 - (joystick.get_axis(2) + 1) / 2,
            6: joystick.get_hat(0)[1], 7: joystick.get_hat(0)[0],
            8: joystick.get_button(5) - joystick.get_button(4),
            9: joystick.get_button(1) - joystick.get_button(0),
        }
        # DEBUG: Log raw inputs
        print(f"\rGamepad Inputs: {[(k, f'{v:.2f}') for k, v in inputs.items()]}", end="")

        if joystick.get_button(6): break
        if joystick.get_button(7):
            print("\nResetting targets to 0...")
            for mid in target_positions: target_positions[mid] = 0.0

        for motor_id, value in inputs.items():
            if motor_id in target_positions and abs(value) > DEADZONE:
                delta = value * SPEED_FACTOR
                target_positions[motor_id] += delta
                # DEBUG: Log target position change
                print(f"\nMotor {motor_id}: value={value:.2f}, delta={delta:.2f}, new_target={target_positions[motor_id]:.2f}")

        # --- P-Control Calculation ---
        try:
            # 1. Read all motor positions at once
            comm_result = group_sync_read.txRxPacket()
            if comm_result != COMM_SUCCESS:
                logger.warning(f"sync_read failed: {packet_handler.getTxRxResult(comm_result)}")
                continue

            # 2. Calculate goal positions and prepare sync_write packet
            for motor_id in active_motor_ids:
                current_pos = group_sync_read.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                target_pos = target_positions[motor_id]
                error = target_pos - current_pos
                control_output = KP * error
                goal_pos = int(current_pos + control_output)

                # DEBUG: Log P-control values
                print(f"\nMotor {motor_id}: current={current_pos}, target={target_pos:.2f}, error={error:.2f}, goal={goal_pos}")

                # Prepare data for sync_write (split 16-bit int into two 8-bit bytes)
                param_goal_position = [goal_pos & 0xFF, (goal_pos >> 8) & 0xFF]
                group_sync_write.addParam(motor_id, param_goal_position)

            # 3. Send all goal positions at once
            comm_result = group_sync_write.txPacket()
            if comm_result != COMM_SUCCESS:
                logger.warning(f"sync_write failed: {packet_handler.getTxRxResult(comm_result)}")

            group_sync_write.clearParam()

        except Exception as e:
            logger.error(f"An unexpected error occurred: {e}")
            traceback.print_exc()
            break

        # Maintain control frequency
        elapsed_time = time.time() - start_time
        if (sleep_time := control_period - elapsed_time) > 0:
            time.sleep(sleep_time)

def main():
    """Main function to set up and run the motor teleoperation."""
    init_logging()
    parser = argparse.ArgumentParser(description="Control individual FeeTech motors with a gamepad.")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port for the motor bus.")
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No gamepad detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Gamepad detected: {joystick.get_name()}")

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort() or not port_handler.setBaudRate(BAUDRATE):
        print("Failed to open port or set baudrate.")
        return

    try:
        p_control_loop(port_handler, packet_handler, joystick)
    finally:
        port_handler.closePort()
        pygame.quit()
        print("Program finished.")

if __name__ == "__main__":
    main()