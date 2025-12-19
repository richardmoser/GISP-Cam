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
import sys, time
import I2C_VL6180X_Functions
from ST_VL6180X import VL6180X
# import tkinter as tk

# sensor power pin setup
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
sensor0_power_pin = 17
sensor1_power_pin = 27
sensor2_power_pin = 22
sensor3_power_pin = 23

# Set all sensor power pins as outputs
GPIO.setup(sensor0_power_pin, GPIO.OUT)
GPIO.setup(sensor1_power_pin, GPIO.OUT)
GPIO.setup(sensor2_power_pin, GPIO.OUT)
GPIO.setup(sensor3_power_pin, GPIO.OUT)
# Turn off all sensors initially
GPIO.output(sensor0_power_pin, GPIO.LOW)
GPIO.output(sensor1_power_pin, GPIO.LOW)
GPIO.output(sensor2_power_pin, GPIO.LOW)
GPIO.output(sensor3_power_pin, GPIO.LOW)
time.sleep(0.1)  # Wait for sensors to power down


# get active I2C bus
i2c_bus = I2C_VL6180X_Functions.get_i2c_bus()
print(f"I2C bus: {i2c_bus}")

# Power on each sensor one at a time and set their I2C addresses
# Sensor 0
GPIO.output(sensor0_power_pin, GPIO.HIGH)
time.sleep(0.1)  # Wait for sensor to power up

# #Initialize and report Sensor 0
# sensor0_i2cid = 0x10
# sensor0 = VL6180X(sensor0_i2cid)
# sensor0.get_identification()
# if sensor0.idModel != 0xB4:
#     print("Not Valid Sensor, Id reported as ",hex(sensor0.idModel))
# else:
#     print("Valid Sensor, ID reported as ",hex(sensor0.idModel))
# sensor0.default_settings()
# # Finish Initialize Sensor 0
# # ---------------------------------
# #Initialize and report Sensor 1
# sensor1_i2cid = 0x11
# sensor1 = VL6180X(sensor1_i2cid)
# sensor1.get_identification()
# if sensor1.idModel != 0xB4:
#     print("Not Valid Sensor, Id reported as ",hex(sensor1.idModel))
# else:
#     print("Valid Sensor, ID reported as ",hex(sensor1.idModel))
# sensor1.default_settings()
# #Finish Initialize Sensor 1
# #---------------------------------
# #Initialize and report Sensor 2
# sensor2_i2cid = 0x12
# sensor2 = VL6180X(sensor2_i2cid)
# sensor2.get_identification()
# if sensor2.idModel != 0xB4:
#     print("Not Valid Sensor, Id reported as ",hex(sensor2.idModel))
# else:
#     print("Valid Sensor, ID reported as ",hex(sensor2.idModel))
# sensor2.default_settings()
# #Finish Initialize Sensor 2
# #---------------------------------
# #Initialize and report Sensor 3
# sensor3_i2cid = 0x13
# sensor3 = VL6180X(sensor3_i2cid)
# sensor3.get_identification()
# if sensor3.idModel != 0xB4:
#     print("Not Valid Sensor, Id reported as ",hex(sensor3.idModel))
# else:
#     print("Valid Sensor, ID reported as ",hex(sensor3.idModel))
# sensor3.default_settings()
# #Finish Initialize Sensor 3
# #---------------------------------

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

def update_sensor_address(current_address, new_address):
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
    while True:  # Loop until 'q' is pressed
        ret, frame = cap.read()  # Read frame from camera
        if not ret:  # If frame is not read correctly
            print(f"{RED}Warning: Frame not read correctly. Attempting to reconnect...{RESET}")
            cap.release()  # Release the camera
            cv2.destroyAllWindows()  # Close all windows
            time.sleep(1)  # Wait before reconnecting
            cap, cam_index = reconnect_camera()  # Try to reconnect
            if cap is None:  # If reconnection failed
                continue  # Retry
            continue  # Continue to next iteration after reconnection
        # cv2.imshow('Video Feed', frame)  # Display the frame in the window
        # add a 5px black border around the frame

        out.write(frame)  # write the frame to the output file
        """ Nothing after this point gets saved to the local video file """
        bordered_frame = cv2.copyMakeBorder(frame, 5, 30, 50, 50, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        # in the bottom border, add the current date and time in white text and the screen resolution
        # timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        # resolution = f"{frame.shape[1]}x{frame.shape[0]}"
        # cv2.putText(bordered_frame, f"{timestamp} | {resolution}", (10, frame.shape[0] + 25),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # test_text = "Test Video Capture, sensor input goes here"
        # cv2.putText(bordered_frame, test_text, (60, frame.shape[0] + 25),
                    # cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        L0 = str(sensor0.get_distance())
        # time.sleep(Range_Convergtime)
        L1 = str(sensor1.get_distance())
        # time.sleep(Range_Convergtime)
        L2 = str(sensor2.get_distance())
        # time.sleep(Range_Convergtime)
        L3 = str(sensor3.get_distance())
        # time.sleep(Range_Convergtime)

        # print the distances on the frame
        cv2.putText(bordered_frame, f"S0: {L0}mm S1: {L1}mm S2: {L2}mm S3: {L3}mm", (10, frame.shape[0] + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Video Feed', bordered_frame)  # Display the frame with border in the window

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on 'q' key press
            break
    cap.release()  # Release the camera



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

        record_video(cam_index=cam_index)  # Record video from a usb camera