#!/usr/bin/env python
import time
import argparse

# Utility methods
from utility import parse_pt_states

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "lib")
import flexivrdk
# fmt: on


def print_description():
    """
    Print tutorial description.

    """
    print("This will bring robot to home cup state!")
    print()


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of this PC')
    args = argparser.parse_args()

    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print description
    log.info("Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        seconds_waited = 0
        while not robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please check that the robot 1) "
                    "has no fault, 2) is in [Auto (remote)] mode")

        log.info("Robot is now operational")

        # Execute Primitives
        # ==========================================================================================
        # Switch to primitive execution mode
        robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)

        # Move robot joints to target positions
        # ------------------------------------------------------------------------------------------
        # The required parameter <target> takes in 7 target joint positions. Unit: degrees
        log.info("Executing primitive: MoveJ")

        # Send command to robot
        robot.executePrimitive("MoveJ(target=61 -22 -17 129 -47 -16 -25)")

        # Wait for reached target
        while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
            time.sleep(1)

        # All done, stop robot and put into IDLE mode
        robot.stop()

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
