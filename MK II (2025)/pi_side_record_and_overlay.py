#!/usr/bin/env python3

"""
Author: Richard Moser
Date: 15Dec25
Purpose: Record and display video from a video feed via a CVBS to USB adapter
"""

import cv2  # OpenCV
import time  # Time
import sys
import os  # OS
import numpy as np
# import tkinter as tk

from cv2.videoio_registry import getBackendName
from cv2_enumerate_cameras import supported_backends
from cv2_enumerate_cameras import enumerate_cameras
os.environ["XDG_SESSION_TYPE"] = "xcb"  # may need to comment this out on certain systems

for backend in supported_backends:
    print(getBackendName(backend))

""" ToF sensor setup """

""" ToF sensor setup """
import sys, time
from ST_VL6180X import VL6180X
import subprocess


# sensor power pin setup
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
sensor0_power_pin = 17
sensor1_power_pin = 27
sensor2_power_pin = 22
sensor3_power_pin = 10
sensor_pins = [sensor0_power_pin, sensor1_power_pin, sensor2_power_pin, sensor3_power_pin]

for pin in sensor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Turn off all sensors initially
time.sleep(0.1)  # Wait for sensors to power down


ignored_addresses = []  # Add any addresses to ignore here

# Sensor configuration
sensor_pins = [sensor0_power_pin, sensor1_power_pin, sensor2_power_pin, sensor3_power_pin]
sensor_addresses = [0x10, 0x11, 0x12, 0x13]  # New I2C addresses for sensors

# Each sensor slot will be a dict with keys:
#   index, power_pin, address, sensor, available, last_value_mm
sensors = []


class SensorUnavailableError(Exception):
    """Custom exception to signal an unavailable sensor."""
    pass


def update_sensor_address(current_address, new_address):
    """Attempt to update a VL6180X sensor I2C address, returning True on success.

    Any exceptions are caught and logged, and False is returned to allow
    graceful degradation when sensors misbehave.
    """
    try:
        sensor = VL6180X(current_address)
        sensor.get_identification()
        if sensor.idModel != 0xB4:
            print(f"Not Valid Sensor at address {hex(current_address)}, Id reported as {hex(sensor.idModel)}")
            return False
        else:
            print(f"Valid Sensor at address {hex(current_address)}, ID reported as {hex(sensor.idModel)}")

        # Change the I2C address
        sensor.change_address(current_address, new_address)
        time.sleep(0.1)  # Wait for the change to take effect

        # Verify the address change
        sensor_new = VL6180X(new_address)
        sensor_new.get_identification()
        if sensor_new.idModel == 0xB4:
            print(f"Successfully changed address to {hex(new_address)}")
            return True
        else:
            print(f"Failed to change address to {hex(new_address)}")
            return False
    except Exception as e:
        print(f"{RED}Exception while updating sensor address from {hex(current_address)} to {hex(new_address)}: {e}{RESET}")
        return False


def init_single_sensor(index, power_pin, target_address):
    """Initialize a single sensor slot.

    Returns a dict describing the sensor slot; failures are logged and
    represented as unavailable sensors so that video can still run.
    """
    slot = {
        "index": index,
        "power_pin": power_pin,
        "address": target_address,
        "sensor": None,
        "available": False,
        "last_value_mm": None,
    }

    print("==============================")
    print(f"Powering on sensor {index}...")
    try:
        GPIO.output(power_pin, GPIO.HIGH)
    except Exception as e:
        print(f"{RED}Failed to power on sensor {index} (pin {power_pin}): {e}{RESET}")
        return slot

    time.sleep(0.1)  # Wait for sensor to power up

    # Scan for new devices
    try:
        p = subprocess.Popen(['i2cdetect', '-y', '1'], stdout=subprocess.PIPE)
        current_addresses = []
        for line in p.stdout:
            line_str = line.decode('utf-8').strip()
            if len(line_str) > 2 and line_str[0:2].isdigit() and line_str[2] == ':':
                parts = line_str.split()
                for part in parts[1:]:
                    if part != '--':
                        current_addresses.append(int(part, 16))
    except Exception as e:
        print(f"{RED}Failed to run i2cdetect for sensor {index}: {e}{RESET}")
        try:
            GPIO.output(power_pin, GPIO.LOW)
        except Exception:
            pass
        return slot

    new_addresses = [addr for addr in current_addresses if addr not in ignored_addresses]
    if len(new_addresses) == 0:
        print(f"No new I2C device found for sensor {index}")
        try:
            GPIO.output(power_pin, GPIO.LOW)  # Power off the sensor
        except Exception:
            pass
        return slot

    current_address = new_addresses[0]
    print(f"Found new I2C device at address {hex(current_address)} for sensor {index}")
    success = update_sensor_address(current_address, target_address)
    if not success:
        try:
            GPIO.output(power_pin, GPIO.LOW)  # Power off the sensor
        except Exception:
            pass
        return slot

    ignored_addresses.append(target_address)

    # Create the sensor object and apply default settings
    try:
        sensor = VL6180X(target_address)
        sensor.default_settings()
        slot["sensor"] = sensor
        slot["available"] = True
        print(f"sensor {index}: initialized at address {hex(target_address)}")
    except Exception as e:
        print(f"{RED}Failed to initialize sensor {index} at address {hex(target_address)}: {e}{RESET}")
        try:
            GPIO.output(power_pin, GPIO.LOW)
        except Exception:
            pass

    return slot


# Initialize sensors one by one using fault-tolerant initialization
sensors = [
    init_single_sensor(i, sensor_pins[i], sensor_addresses[i])
    for i in range(len(sensor_pins))
]

print("==============================")
for slot in sensors:
    status = "available" if slot["available"] else "unavailable"
    print(f"sensor {slot['index']}: {status} at address {hex(slot['address'])}")

#Time allotted to each sensor to make a reading in sec
Range_Convergtime = 0.02


results = 0
L0 = "0"
L1 = "0"
L2 = "0"
L3 = "0"

""" functions and main program """



# set colors
BLACK = '\033[30m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m' # orange on some systems
BLUE = '\033[34m'
MAGENTA = '\033[35m'
CYAN = '\033[36m'
LIGHT_GRAY = '\033[37m'
DARK_GRAY = '\033[90m'
BRIGHT_RED = '\033[91m'
BRIGHT_GREEN = '\033[92m'
BRIGHT_YELLOW = '\033[93m'
BRIGHT_BLUE = '\033[94m'
BRIGHT_MAGENTA = '\033[95m'
BRIGHT_CYAN = '\033[96m'
WHITE = '\033[97m'

RESET = '\033[0m' # called to return to standard terminal text color
CLEAR = '\033[2J\033[H'  # clear screen

# set the directory to save the file in
# dir = "captures/"
# set the directory to {the date}_Captures
dir = time.strftime("/%Y%m%d", time.localtime()) + "_Captures/"



def print_cams():  # Print available cameras
    num_cams = 10  # Number of cameras to check
    cams = [False, False, False, False, False, False, False, False, False, False]  # List of cameras
    good_cams = []  # List of cameras
    while len(good_cams) == 0:  # loop until a camera is found
        for i in range(num_cams):  # Check for  cameras
            cap = cv2.VideoCapture(i)  # Check camera
            if cap.isOpened():  # Check if camera is opened
                # print('Camera', i, 'is available')  # Print camera
                cams[i] = True
                good_cams.append(i)  # Add camera to list
            else:  # Camera is not available
                # print('Camera', i, 'is not available')  # Print camera
                cams[i] = False  # Set camera to not available
            cap.release()  # Release camera
            cv2.destroyAllWindows()  # Close all windows

        # print the list of cameras
        print(f"Checking cameras:")
        for i in range(num_cams):  # Check for  cameras
            # print(f"Camera {i}: {'{GREEN }Available' if cams[i] else 'Not Available'}")
            if cams[i]:  # Camera is available
                print(f"Camera {i}: {GREEN}Available{RESET}")  # Print camera
                # print the camera name
                # print(f"Camera {i}: {GREEN}Available{RESET} ({cap.getBackendName()})")  # Print camera
            else:  # Camera is not available
                print(f"Camera {i}: {RED}Not Available{RESET}")  # Print camera

        # wait a moment before checking again
        # time.sleep(0.25)  # Wait
        time.sleep(0.5)
        # clear the console
        # os.system('cls' if os.name == 'nt' else 'clear')  # Clear console
        # os.system('cls || clear')

    return good_cams  # Return list of cameras

def reconnect_camera():
    print(f"{YELLOW}Scanning for reconnected camera...{RESET}")
    for i in range(5):  # Try first 5 indices
        test_cap = cv2.VideoCapture(i)
        if test_cap.isOpened():
            print(f"{GREEN}Reconnected on camera index {i}{RESET}")
            return test_cap, i
        test_cap.release()
    print(f"{RED}No camera found. Retrying in 1 second...{RESET}")
    return None, None

# define a function to display the video feed from the camera in a window
def record_video(cam_index=0):
    # the window should be resizable and the video feed should adjust to the window size
    cap = cv2.VideoCapture(cam_index)  # Capture video from camera
    if not cap.isOpened():  # Check if camera is opened
        print(f"{RED}Error: Could not open camera with index {cam_index}{RESET}")
        return
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    date_string = time.strftime("%Y%m%d", time.localtime())
    time_string = time.strftime("%H%M%S", time.localtime())
    filename = f"{dir}capture_{date_string}_{time_string}.avi"
    out = cv2.VideoWriter(filename,fourcc, 20.0, (640, 480))  # Output file
    # Create a fullscreen window
    cv2.namedWindow('Video Feed', cv2.WINDOW_NORMAL)  # Create resizable window
    cv2.setWindowProperty('Video Feed', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)  # Set to fullscreen

    # Create ToF log file alongside the video file
    tof_log_filename = f"{dir}ToF_{date_string}_{time_string}.log"
    # Ensure directory exists
    os.makedirs(os.path.dirname(tof_log_filename), exist_ok=True)
    try:
        tof_log = open(tof_log_filename, "a", buffering=1)
        tof_log.write("# timestamp,S1_mm,S2_mm,S3_mm,S4_mm\n")
    except Exception as e:
        print(f"{RED}Failed to open ToF log file {tof_log_filename}: {e}{RESET}")
        tof_log = None

    # History of last 5 readings (current + 4 previous) for 4 sensors; values are ints or None
    history = [[None, None, None, None] for _ in range(5)]

    def read_sensor(slot):
        """Safely read a single sensor.

        Returns (value_int_or_None, updated_slot). If a sensor fails mid-run,
        it is marked unavailable and will return None from then on.
        """
        if not slot["available"] or slot["sensor"] is None:
            return None, slot
        try:
            value = slot["sensor"].get_distance()
            try:
                value_int = int(value)
            except (TypeError, ValueError):
                value_int = None
            if value_int is not None and value_int < 0:
                value_int = 0
            slot["last_value_mm"] = value_int
            return value_int, slot
        except Exception as e:
            print(f"{RED}Sensor {slot['index']} read failed, marking as unavailable: {e}{RESET}")
            slot["available"] = False
            slot["sensor"] = None
            slot["last_value_mm"] = None
            try:
                GPIO.output(slot["power_pin"], GPIO.LOW)
            except Exception:
                pass
            return None, slot

    try:
        while True:  # Loop until 'q' is pressed
            ret, frame = cap.read()  # Read frame from camera
            if not ret:  # If frame is not read correctly
                print(f"{RED}Error: Could not read frame from camera.{RESET}")
                disconnect = True
                while disconnect:
                    print(f"{RED}Warning: Frame not read correctly. Attempting to reconnect...{RESET}")
                    # cap.release()  # Release the camera
                    # cv2.destroyAllWindows()  # Close all windows
                    time.sleep(1)  # Wait before reconnecting
                    cap, cam_index = reconnect_camera()  # Try to reconnect
                    if cap is None:  # If reconnection failed
                        continue  # Retry
                    else:
                        disconnect = False  # Reconnection successful
                        print(f"{GREEN}Reconnected to camera index {cam_index}{RESET}")
                        ret, frame = cap.read()  # Read frame from camera
                        break  # Exit the reconnect loop
                # continue
            # cv2.imshow('Video Feed', frame)  # Display the frame in the window
            # add a 5px black border around the frame

            out.write(frame)  # write the frame to the output file

            """ Nothing after this point gets saved to the local video file """

            bordered_frame = cv2.copyMakeBorder(frame, 1, 32, 30, 30, cv2.BORDER_CONSTANT, value=[0, 0, 0])

            # get the distance from each sensor (ints or None)
            current_values = [None, None, None, None]
            for i in range(4):
                if i < len(sensors):
                    value, sensors[i] = read_sensor(sensors[i])
                    current_values[i] = value
                else:
                    current_values[i] = None

            # update history: shift older readings and insert current at front
            history[4] = history[3]
            history[3] = history[2]
            history[2] = history[1]
            history[1] = history[0]
            history[0] = current_values

            # average the current reading with the previous readings to smooth out the data
            smoothed_values = []
            for sensor_idx in range(4):
                samples = [h[sensor_idx] for h in history if h[sensor_idx] is not None]
                if len(samples) == 0:
                    smoothed_values.append(None)
                else:
                    smoothed_values.append(sum(samples) // len(samples))

            # Convert smoothed values to display strings, using '---' for unavailable sensors
            display_values = []
            for val in smoothed_values:
                if val is None:
                    display_values.append("---")
                else:
                    display_values.append(str(val))

            L0, L1, L2, L3 = display_values

            timestamp = time.strftime("%Y.%m.%d %H:%M:%S", time.localtime())

            # Log ToF readings if log file is available
            if tof_log is not None:
                try:
                    tof_log.write(f"{timestamp},{L0},{L1},{L2},{L3}\n")
                except Exception as e:
                    print(f"{RED}Failed to write ToF log entry: {e}{RESET}")
                    tof_log = None

            # print the distances at the bottom of the video feed
            cv2.putText(bordered_frame, f"S1: {L0}mm S2: {L1}mm S3: {L2}mm S4: {L3}mm", (10, frame.shape[0] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            # print the timestamp at the top left of the video feed
            cv2.putText(bordered_frame, f"{timestamp}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

            cv2.imshow('Video Feed', bordered_frame)  # Display the frame with border in the window

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on 'q' key press
                break
    finally:
        cap.release()  # Release the camera
        if tof_log is not None:
            try:
                tof_log.close()
            except Exception:
                pass



if __name__ == '__main__':  # Run the
    # program
    if len(sys.argv) > 1:  # Check for command line argument
        cam_index = int(sys.argv[1])  # Get camera index from command line
        # record_video(cam_index=cam_index)  # Record video from a usb camera
    else:  # Default
        # cam_index = 0
        # cam_index = print_cams()  # Print available cameras
        for camera_info in enumerate_cameras():
            print(f'Camera {camera_info.index}: {camera_info.name}')
            if camera_info.name == "AFN_Cap video: AFN_Cap video":
                # cap = cv2.VideoCapture(camera_info.index)
                cam_index = camera_info.index
                print(f"{GREEN}Using camera index {cam_index}{RESET}")

        try:
            record_video(cam_index=cam_index)  # Record video from a usb camera
        except Exception as e:
            if f"name 'cam_index' is not defined" in str(e):
                print(f"{RED}Error: No suitable camera found. Exiting program.{RESET}")
            else:
                print(f"{RED}An error occurred: {e}{RESET}")