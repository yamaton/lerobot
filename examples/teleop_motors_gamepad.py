#!/usr/bin/env python
"""
A low-level script to control up to 9 individual FeeTech motors using a gamepad.

This script automatically detects if a motor is in "Position Control" or "Velocity Control"
(wheel) mode and sends the appropriate commands.

GAMEPAD MAPPING (XBox Controller Style):
- Motor 1 (Pos): Left Stick Up/Down
- Motor 2 (Pos): Left Stick Left/Right
- Motor 3 (Pos): Right Stick Up/Down
- Motor 4 (Pos): Right Stick Left/Right
- Motor 5 (Pos): Right Trigger (positive) / Left Trigger (negative)
- Motor 6 (Vel): D-Pad Up/Down
- Motor 7 (Vel): D-Pad Left/Right
- Motor 8 (Vel): Right Bumper (positive) / Left Bumper (negative)
- Motor 9 (Vel): 'B' button (positive) / 'A' button (negative)

- Start Button: Reset all motors to position/velocity 0.
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
KP = 0.5  # Proportional gain for position control
DEADZONE = 0.15
SPEED_FACTOR_POS = 1.5  # Speed for position-controlled motors
SPEED_FACTOR_VEL = 1023  # Max velocity for velocity-controlled motors
MAX_MOTORS_TO_CONTROL = 9
PROTOCOL_VERSION = 0.0
BAUDRATE = 1_000_000

# --- Motor Register Addresses (assuming sts_series) ---
CONTROL_TABLE = MODEL_CONTROL_TABLE["sts_series"]
ADDR_TORQUE_ENABLE, _ = CONTROL_TABLE["Torque_Enable"]
ADDR_OPERATING_MODE, _ = CONTROL_TABLE["Operating_Mode"]
ADDR_GOAL_POSITION, LEN_GOAL_POSITION = CONTROL_TABLE["Goal_Position"]
ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION = CONTROL_TABLE["Present_Position"]
ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY = CONTROL_TABLE["Goal_Velocity"]

# --- Gamepad Axis/Button Constants ---
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 5
BUTTON_A = 0
BUTTON_B = 1
BUTTON_LEFT_BUMPER = 4
BUTTON_RIGHT_BUMPER = 5
BUTTON_BACK = 6
BUTTON_START = 7


def p_control_loop(
    port_handler, packet_handler, joystick, position_motors, velocity_motors
):
    """Main control loop for motor teleoperation, handling both position and velocity modes."""
    control_period = 1.0 / FPS

    # --- Initialize GroupSync objects ---
    group_read_pos = GroupSyncRead(
        port_handler, packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
    )
    for motor_id in position_motors:
        group_read_pos.addParam(motor_id)

    group_write_pos = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
    )
    group_write_vel = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY
    )

    # --- Initialize Target Positions ---
    target_positions = {motor_id: 0.0 for motor_id in position_motors}
    if position_motors:
        try:
            comm_result = group_read_pos.txRxPacket()
            if comm_result != COMM_SUCCESS:
                raise ConnectionError(packet_handler.getTxRxResult(comm_result))
            for motor_id in position_motors:
                pos = group_read_pos.getData(
                    motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                )
                target_positions[motor_id] = float(pos if pos is not None else 0.0)
            logger.info(f"Initial target positions set to: {target_positions}")
        except Exception as e:
            logger.error(f"Failed to read initial motor positions: {e}. Starting at 0.")

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
            6: joystick.get_hat(0)[1],
            7: joystick.get_hat(0)[0],
            8: joystick.get_button(BUTTON_RIGHT_BUMPER)
            - joystick.get_button(BUTTON_LEFT_BUMPER),
            9: joystick.get_button(BUTTON_B) - joystick.get_button(BUTTON_A),
        }

        if joystick.get_button(BUTTON_BACK):
            break
        if joystick.get_button(BUTTON_START):
            print("\nResetting all targets to 0...")
            for mid in target_positions:
                target_positions[mid] = 0.0
            # For velocity motors, we will send a 0 velocity command below

        # --- Prepare motor commands ---
        try:
            # For position motors, update target positions
            for motor_id in position_motors:
                value = inputs.get(motor_id, 0.0)
                if abs(value) > DEADZONE:
                    target_positions[motor_id] += value * SPEED_FACTOR_POS

            # For velocity motors, calculate velocity command directly
            for motor_id in velocity_motors:
                value = inputs.get(motor_id, 0.0)
                velocity_cmd = 0
                if abs(value) > DEADZONE:
                    velocity_cmd = int(abs(value) * SPEED_FACTOR_VEL)
                    if value < 0:  # Set direction bit for negative velocity
                        velocity_cmd |= 1 << 15
                param_goal_velocity = [velocity_cmd & 0xFF, (velocity_cmd >> 8) & 0xFF]
                group_write_vel.addParam(motor_id, param_goal_velocity)

            # For position motors, run P-control loop
            if position_motors:
                comm_result = group_read_pos.txRxPacket()
                if comm_result != COMM_SUCCESS:
                    logger.warning(
                        f"sync_read failed: {packet_handler.getTxRxResult(comm_result)}"
                    )
                    continue

                for motor_id in position_motors:
                    current_pos = group_read_pos.getData(
                        motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    )
                    if current_pos is None:
                        continue

                    error = target_positions[motor_id] - current_pos
                    goal_pos = int(current_pos + KP * error)
                    param_goal_position = [goal_pos & 0xFF, (goal_pos >> 8) & 0xFF]
                    group_write_pos.addParam(motor_id, param_goal_position)

            # --- Send Commands to Motors ---
            if position_motors:
                group_write_pos.txPacket()
            if velocity_motors:
                group_write_vel.txPacket()

            group_write_pos.clearParam()
            group_write_vel.clearParam()

        except Exception as e:
            logger.error(f"An unexpected error occurred: {e}")
            traceback.print_exc()
            break

        # Maintain control frequency
        if (sleep_time := control_period - (time.time() - start_time)) > 0:
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

    # --- Discover motors and determine their operating mode ---
    position_motors = []
    velocity_motors = []
    print("Discovering motors and their operating modes...")
    for motor_id in range(1, MAX_MOTORS_TO_CONTROL + 1):
        if packet_handler.ping(port_handler, motor_id)[1] == COMM_SUCCESS:
            try:
                mode, _, _ = packet_handler.read1ByteTxRx(
                    port_handler, motor_id, ADDR_OPERATING_MODE
                )
                if mode == 0:  # Position Control
                    position_motors.append(motor_id)
                    print(f"  - Found Position Motor (ID: {motor_id})")
                elif mode == 1:  # Velocity Control
                    velocity_motors.append(motor_id)
                    print(f"  - Found Velocity Motor (ID: {motor_id})")
                else:
                    print(
                        f"  - Found Motor with unknown mode (ID: {motor_id}, Mode: {mode})"
                    )
            except Exception as e:
                print(
                    f"  - Found Motor (ID: {motor_id}), but could not read its mode: {e}"
                )

    if not position_motors and not velocity_motors:
        print("No active motors found.")
        port_handler.closePort()
        pygame.quit()
        return

    try:
        # --- Enable Torque on velocity motors ---
        if velocity_motors:
            print(f"Enabling torque for velocity motors: {velocity_motors}...")
            for motor_id in velocity_motors:
                packet_handler.write1ByteTxRx(
                    port_handler, motor_id, ADDR_TORQUE_ENABLE, 1
                )

        # --- Start Control Loop ---
        p_control_loop(
            port_handler, packet_handler, joystick, position_motors, velocity_motors
        )

    finally:
        # --- Disable Torque on all active motors for safety ---
        all_motors = position_motors + velocity_motors
        if all_motors:
            print(f"\nDisabling torque for all motors: {all_motors}...")
            for motor_id in all_motors:
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
