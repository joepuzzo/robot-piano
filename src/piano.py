#!/usr/bin/env python

import time
import argparse
import math

# Utility methods
from utility import quat2eulerZYX
from utility import list2str
from utility import parse_pt_states

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on

# Maximum contact wrench [fx, fy, fz, mx, my, mz] [N][Nm]
MAX_CONTACT_WRENCH = [50.0, 50.0, 50.0, 15.0, 15.0, 15.0]


# Global variables for calibration
A0_position = None
C8_position = None
midi_notes = []


MAX_VELOCITY = 1.0
CONSTANT_TIME = 1.0  # This is an arbitrary value; adjust as needed.

def compute_velocity(current_position, target_position):
    # Calculate Euclidean distance between current and target positions
    distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(current_position, target_position)]))

    # Calculate required velocity to maintain constant time
    required_velocity = distance / CONSTANT_TIME

    # Ensure velocity doesn't exceed MAX_VELOCITY
    return min(required_velocity, MAX_VELOCITY)


def convert_note_to_index(note):
    # Convert MIDI note name to an index
    note_sequence = [
        'A0', 'A#0', 'B0',
        'C1', 'C#1', 'D1', 'D#1', 'E1', 'F1', 'F#1', 'G1', 'G#1', 'A1',
        'A#1', 'B1', 'C2', 'C#2', 'D2', 'D#2', 'E2', 'F2', 'F#2', 'G2', 'G#2', 'A2',
        'A#2', 'B2', 'C3', 'C#3', 'D3', 'D#3', 'E3', 'F3', 'F#3', 'G3', 'G#3', 'A3',
        'A#3', 'B3', 'C4', 'C#4', 'D4', 'D#4', 'E4', 'F4', 'F#4', 'G4', 'G#4', 'A4',
        'A#4', 'B4', 'C5', 'C#5', 'D5', 'D#5', 'E5', 'F5', 'F#5', 'G5', 'G#5', 'A5',
        'A#5', 'B5', 'C6', 'C#6', 'D6', 'D#6', 'E6', 'F6', 'F#6', 'G6', 'G#6', 'A6',
        'A#6', 'B6', 'C7', 'C#7', 'D7', 'D#7', 'E7', 'F7', 'F#7', 'G7', 'G#7', 'A7',
        'A#7', 'B7', 'C8'
    ]
    return note_sequence.index(note)


def calculate_position(index, start_position, end_position):
    # Calculate note's Cartesian position based on the index and calibration data
    # Linear interpolation for simplicity
    factor = index / 87.0
    position = [
        start_position[i] + factor * (end_position[i] - start_position[i])
        for i in range(3)
    ]
    return position


def print_description():
    """
    Print description.

    """
    print("This tutorial demonstrates controlling the robot arm using a teach-by-demonstration approach.")
    print("The following functionalities are provided:")
    print("1. Calibration mode: Calibrate the positions of the A0 and C8 piano keys.")
    print("2. Note recording: Enter a comma-separated list of MIDI notes for the robot to play.")
    print("3. Playback: Have the robot arm play the sequence of recorded notes.")
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
    log.info("Description:")
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

        # Teach By Demonstration
        # ==========================================================================================
        # Recorded robot poses
        saved_poses = []

        # Robot states data
        robot_states = flexivrdk.RobotStates()

        # Acceptable user inputs
        log.info("Accepted key inputs:")
        print("[c] - calibrate to piano")
        print(
            "[r] - record midi notes or start and end positions when in calibration mode")
        print("[e] - start execution")

        # User input polling
        input_buffer = ""
        while True:
            input_buffer = str(input())

            if input_buffer == "c":
                # Clear storage
                saved_poses.clear()

                # Put robot to plan execution mode
                robot.setMode(mode.NRT_PLAN_EXECUTION)

                # Robot run free drive
                robot.executePlan("PLAN-FreeDriveAuto")

                log.info("New Calibration process started")
                log.warn(
                    "Hold down the enabling button on the motion bar to activate free drive")

                log.info(
                    "Calibration Mode. Please move the robot to the A0 key and press 'r'.")
                while A0_position is None:
                    if str(input()) == "r":
                        robot.getRobotStates(robot_states)
                        A0_position = robot_states.tcpPose
                        log.info("A0 pose saved: " + str(A0_position))

                log.info("Now, please move the robot to the C8 key and press 'r'.")
                while C8_position is None:
                    if str(input()) == "r":
                        robot.getRobotStates(robot_states)
                        C8_position = robot_states.tcpPose
                        log.info("C8 pose saved: " + str(A0_position))

                log.info("Calibration completed.")

            elif input_buffer == "r":
                midi_input = input(
                    "Enter comma-separated list of midi notes: ")
                midi_notes = midi_input.split(",")
                log.info("MIDI notes recorded.")

            elif input_buffer == "e":
                if not A0_position or not C8_position:
                    log.warn("Please calibrate first.")
                    continue

                # Put robot to primitive execution mode
                robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)

                for note in midi_notes:
                    note_index = convert_note_to_index(note)
                    note_position = calculate_position(
                        note_index, A0_position, C8_position)
                    
                    # Compute velocity
                    # note_velocity = compute_velocity(robot_states.tcpPose[:3], note_position)
                    note_velocity = "0.3";

                    # Convert quaternion to Euler ZYX required by MoveCompliance primitive
                    target_quat = [A0_position[3],
                                   A0_position[4], A0_position[5], A0_position[6]]

                    target_euler_deg = quat2eulerZYX(target_quat, degree=True)

                    # Move the robot to note_position and make it press the key
                    robot.executePrimitive(
                        "MoveCompliance(target="
                        + list2str(note_position)
                        + list2str(target_euler_deg)
                        + "WORLD WORLD_ORIGIN, maxVel="
                        note_velocity
                        +", enableMaxContactWrench=1, maxContactWrench="
                        + list2str(MAX_CONTACT_WRENCH) + ")"
                    )

                    while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
                        time.sleep(1)

                log.info("All notes played.")
            else:
                log.warn("Invalid input")

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
