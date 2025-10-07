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
    from scservo_sdk import (
        COMM_SUCCESS,
        GroupSyncRead,
        GroupSyncWrite,
        PacketHandler,
        PortHandler,
    )
except ImportError:
    print("Error: scservo_sdk not found.")
    print(
        "Please ensure the lerobot development environment is set up and sourced correctly."
    )
    sys.exit(1)

from lerobot.motors.feetech.tables import MODEL_CONTROL_TABLE
from lerobot.utils.utils import init_logging

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
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
ADDR_TORQUE_ENABLE, _ = CONTROL_TABLE["Torque_Enable"]


# --- Gamepad Axis/Button Constants (for XBox-style controller) ---
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 5

BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LEFT_BUMPER = 4
BUTTON_RIGHT_BUMPER = 5
BUTTON_BACK = 6
BUTTON_START = 7


def p_control_loop(port_handler, packet_handler, joystick, active_motor_ids):
    """Main proportional control loop for motor teleoperation."""
    control_period = 1.0 / FPS

    # --- Initialize GroupSyncRead and GroupSyncWrite ---
    group_sync_read = GroupSyncRead(
        port_handler, packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
    )
    for motor_id in active_motor_ids:
        group_sync_read.addParam(motor_id)

    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
    )

    # --- Initialize Target Positions ---
    try:
        # Set initial targets to the robot's current positions to prevent initial movement
        comm_result = group_sync_read.txRxPacket()
        if comm_result != COMM_SUCCESS:
            raise ConnectionError(packet_handler.getTxRxResult(comm_result))

        target_positions = {}
        for motor_id in active_motor_ids:
            pos = group_sync_read.getData(
                motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            # Handle case where getData might fail for one motor
            if pos is not None:
                target_positions[motor_id] = float(pos)
            else:
                target_positions[motor_id] = 0.0
                logger.warning(
                    f"Failed to read initial position for motor {motor_id}. Defaulting to 0."
                )

        logger.info(f"Initial target positions set to: {target_positions}")

    except Exception as e:
        logger.error(f"Failed to read initial motor positions: {e}. Starting all at 0.")
        target_positions = {motor_id: 0.0 for motor_id in active_motor_ids}

    print("\nStarting teleoperation loop. Press Back/Select button to exit.")

    while True:
        start_time = time.time()
        pygame.event.pump()

        # --- Read Gamepad and Update Targets ---
        left_trigger = (joystick.get_axis(AXIS_LEFT_TRIGGER) + 1) / 2
        right_trigger = (joystick.get_axis(AXIS_RIGHT_TRIGGER) + 1) / 2

        inputs = {
            1: -joystick.get_axis(AXIS_LEFT_STICK_Y),
            2: joystick.get_axis(AXIS_LEFT_STICK_X),
            3: -joystick.get_axis(AXIS_RIGHT_STICK_Y),
            4: joystick.get_axis(AXIS_RIGHT_STICK_X),
            5: right_trigger - left_trigger,
            6: joystick.get_hat(0)[1],  # D-Pad Up/Down
            7: joystick.get_hat(0)[0],  # D-Pad Left/Right
            8: joystick.get_button(BUTTON_RIGHT_BUMPER)
            - joystick.get_button(BUTTON_LEFT_BUMPER),
            9: joystick.get_button(BUTTON_B) - joystick.get_button(BUTTON_A),
        }

        if joystick.get_button(BUTTON_BACK):
            break
        if joystick.get_button(BUTTON_START):
            print("\nResetting targets to 0...")
            for mid in target_positions:
                target_positions[mid] = 0.0

        for motor_id, value in inputs.items():
            if motor_id in target_positions and abs(value) > DEADZONE:
                target_positions[motor_id] += value * SPEED_FACTOR

        # --- P-Control Calculation ---
        try:
            # 1. Read all motor positions at once
            comm_result = group_sync_read.txRxPacket()
            if comm_result != COMM_SUCCESS:
                logger.warning(
                    f"sync_read failed: {packet_handler.getTxRxResult(comm_result)}"
                )
                continue

            # 2. Calculate goal positions and prepare sync_write packet
            for motor_id in active_motor_ids:
                current_pos = group_sync_read.getData(
                    motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                )
                if current_pos is None:
                    continue  # Skip if read failed for this motor

                target_pos = target_positions[motor_id]
                error = target_pos - current_pos
                control_output = KP * error
                goal_pos = int(current_pos + control_output)

                param_goal_position = [goal_pos & 0xFF, (goal_pos >> 8) & 0xFF]
                group_sync_write.addParam(motor_id, param_goal_position)

            # 3. Send all goal positions at once
            comm_result = group_sync_write.txPacket()
            if comm_result != COMM_SUCCESS:
                logger.warning(
                    f"sync_write failed: {packet_handler.getTxRxResult(comm_result)}"
                )

            # 4. Clear the write list AFTER sending
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
    parser = argparse.ArgumentParser(
        description="Control individual FeeTech motors with a gamepad."
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="Serial port for the motor bus.",
    )
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

    # --- Discover active motors before starting the loop ---
    active_motor_ids = []
    print("Pinging motor IDs 1-9 to find active motors...")
    for motor_id in range(1, MAX_MOTORS_TO_CONTROL + 1):
        _, comm_result, _ = packet_handler.ping(port_handler, motor_id)
        if comm_result == COMM_SUCCESS:
            active_motor_ids.append(motor_id)

    if not active_motor_ids:
        print("No active motors found.")
        port_handler.closePort()
        pygame.quit()
        return
    print(f"Found active motors: {active_motor_ids}")

    # Identify motors requiring torque (IDs >= 7)
    torque_motors = [mid for mid in active_motor_ids if mid >= 7]

    try:
        # --- Enable Torque on high-ID motors (7, 8, 9) ---
        if torque_motors:
            print(f"Enabling torque for wheel motors: {torque_motors}...")
            for motor_id in torque_motors:
                packet_handler.write1ByteTxRx(
                    port_handler, motor_id, ADDR_TORQUE_ENABLE, 1
                )

        # --- Start Control Loop ---
        p_control_loop(port_handler, packet_handler, joystick, active_motor_ids)

    finally:
        # --- Disable Torque on high-ID motors for safety ---
        if torque_motors:
            print(f"\nDisabling torque for wheel motors: {torque_motors}...")
            for motor_id in torque_motors:
                try:
                    packet_handler.write1ByteTxRx(
                        port_handler, motor_id, ADDR_TORQUE_ENABLE, 0
                    )
                except Exception as e:
                    print(f"Could not disable torque for motor {motor_id}: {e}")

        port_handler.closePort()
        pygame.quit()
        print("Program finished.")


if __name__ == "__main__":
    main()
