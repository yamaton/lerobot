#!/usr/bin/env python
"""
A diagnostics utility to list all motor names and IDs for a given robot.

Example usage:

```shell
python examples/diagnostics/list_motors.py --robot.type=so101_follower --robot.port=/dev/ttyACM0
```
"""

import logging
from dataclasses import dataclass, field

import draccus

from lerobot.robots import (
    RobotConfig,
    koch_follower,  # noqa: F401
    make_robot_from_config,
    so100_follower,  # noqa: F401
    so101_follower,  # noqa: F401
)
from lerobot.utils.utils import init_logging


@dataclass
class DiagConfig:
    robot: RobotConfig = field(default_factory=SO101FollowerConfig)


@draccus.wrap()
def list_motors(cfg: DiagConfig):
    """Connect to a robot and list its motor names and IDs."""
    init_logging()

    logging.info(f"Initializing robot of type '{cfg.robot.type}'")
    robot = make_robot_from_config(cfg.robot)

    try:
        robot.connect()
        logging.info(f"Successfully connected to robot.")

        print("\n" + "-" * 50)
        print(f"Motor Diagnostics for robot '{robot.name}'")
        print("-" * 50)

        if hasattr(robot, "bus") and hasattr(robot.bus, "motors") and robot.bus.motors:
            print(f"Found {len(robot.bus.motors)} motors:")
            for name, motor in robot.bus.motors.items():
                print(f"  - Name: {name:<20} ID: {motor.id}")
        else:
            print("No motors found or robot does not have a 'bus.motors' attribute.")

        print("-" * 50)

    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        if robot.is_connected:
            logging.info("Disconnecting robot.")
            robot.disconnect()


if __name__ == "__main__":
    list_motors()
