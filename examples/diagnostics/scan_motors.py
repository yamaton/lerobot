#!/usr/bin/env python
"""
A low-level diagnostics utility to scan a FeeTech motor bus and discover all connected motors.

This script is based on the `ping.py` example from the `scservo_sdk` and directly
uses the low-level SDK to communicate with the motor bus.

It pings a range of possible motor IDs and reports the ID and model name of any motor that responds.

Example usage:

```shell
python examples/diagnostics/scan_motors.py --port /dev/ttyACM0
```
"""

import argparse
import sys
import traceback

# The scservo_sdk is a third-party library provided in the lerobot repository.
# We add a try-except block for robustness in case the environment is not sourced correctly.
try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("Error: scservo_sdk not found.")
    print("Please ensure the lerobot development environment is set up and sourced correctly.")
    sys.exit(1)

from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE

# Create a reverse mapping from model number (the value) to model name (the key)
MODEL_NUMBER_TO_NAME = {v: k for k, v in MODEL_NUMBER_TABLE.items()}

# --- Constants for FeeTech Motors used in lerobot ---
PROTOCOL_VERSION = 0.0
BAUDRATE = 1_000_000  # Default baud rate for lerobot-configured motors


def scan_bus(port: str):
    """Scans a FeeTech motor bus using direct SDK calls."""
    port_handler = PortHandler(port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    # --- 1. Open Port ---
    if not port_handler.openPort():
        print(f"Failed to open port: {port}")
        return
    print(f"Successfully opened port: {port}")

    # --- 2. Set Baud Rate ---
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baud rate to {BAUDRATE}")
        port_handler.closePort()
        return
    print(f"Successfully set baud rate to {BAUDRATE}")

    try:
        print("\n" + "-" * 60)
        print(f"Scanning FeeTech bus on port '{port}'...")
        print("-" * 60)

        found_motors_count = 0
        # Ping a reasonable range of IDs (1 to 30)
        for motor_id in range(1, 31):
            # --- 3. Ping Motor ---
            model_number, comm_result, error = packet_handler.ping(port_handler, motor_id)

            if comm_result != COMM_SUCCESS:
                # This is expected for IDs that don't exist, so we don't print an error.
                # print(f"ID {motor_id}: {packet_handler.getTxRxResult(comm_result)}")
                pass
            elif error != 0:
                print(f"ID {motor_id}: {packet_handler.getRxPacketError(error)}")
            else:
                found_motors_count += 1
                model_name = MODEL_NUMBER_TO_NAME.get(model_number, f"Unknown P/N ({model_number})")
                print(f"  SUCCESS! Found Motor -> ID: {motor_id:<5} Model: {model_name}")

        # --- 4. Report Results ---
        print("-" * 60)
        if found_motors_count == 0:
            print("\nNo motors found on the bus.")
            print("Please check:")
            print("  1. The robot is powered on.")
            print("  2. The USB cable is securely connected.")
            print(f"  3. The correct port ('{port}') is being used.")
            print(f"  4. All motors are configured to a {BAUDRATE} baud rate.")
        else:
            print(f"\nScan complete. Found {found_motors_count} motors.")
        print("-" * 60)

    except Exception as e:
        print(f"An unexpected error occurred during the scan: {e}")
        traceback.print_exc()
    finally:
        # --- 5. Close Port ---
        port_handler.closePort()
        print("Port closed.")


def main():
    parser = argparse.ArgumentParser(
        description="Scan a FeeTech motor bus using low-level SDK calls to discover connected motors."
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="The serial port the motor bus is connected to (e.g., /dev/ttyUSB0).",
    )
    args = parser.parse_args()

    scan_bus(args.port)


if __name__ == "__main__":
    main()
