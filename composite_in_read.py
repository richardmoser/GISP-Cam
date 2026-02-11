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
dir = "captures/"
# set the directory to {the date}_Captures
dir = time.strftime("../%Y%m%d", time.localtime()) + "_Captures/"


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

#display the video feed from the camera in a window
def record_video(cam_index=0):
    # the window should be resizable and the video feed should adjust to the window size
    cap = cv2.VideoCapture(cam_index)  # Capture video from camera
    if not cap.isOpened():  # Check if camera is opened
        print(f"{RED}Error: Could not open camera with index {cam_index}{RESET}")
        return
    cv2.namedWindow('Video Feed', cv2.WINDOW_NORMAL)  # Create a resizable window
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
        bordered_frame = cv2.copyMakeBorder(frame, 5, 30, 5, 5, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        # in the bottom border, add the current date and time in white text and the screen resolution
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        resolution = f"{frame.shape[1]}x{frame.shape[0]}"
        cv2.putText(bordered_frame, f"{timestamp} | {resolution}", (10, frame.shape[0] + 25),
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
        # cam_index = 202
        record_video(cam_index=cam_index)  # Record video from a usb camera