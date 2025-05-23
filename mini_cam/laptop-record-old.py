"""
Author: Richard Moser
Date: 15Apr25
Purpose: Record video from a usb camera
"""

import cv2  # OpenCV
import time  # Time
import sys
import os  # OS
import tkinter as tk

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

# set the directory to save the file in
# dir = "captures/"
# set the directory to {the date}_Captures
dir = time.strftime("%Y%m%d", time.localtime()) + "_Captures/"

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
            else:  # Camera is not available
                print(f"Camera {i}: {RED}Not Available{RESET}")  # Print camera

        # wait a moment before checking again
        # time.sleep(0.25)  # Wait
        time.sleep(2)
        # clear the console
        # os.system('cls' if os.name == 'nt' else 'clear')  # Clear console
        # os.system('cls || clear')

    return good_cams  # Return list of cameras

def record_video(cam_index):  # Record video from a usb camera
    cap = cv2.VideoCapture(cam_index)  # 0 is the default camera
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec
    # set the filename to the date and time
    date_string = time.strftime("%Y%m%d", time.localtime())
    time_string = time.strftime("%H%M%S", time.localtime())

    # check for a directory to save the file and create it if it doesn't exist"
    if not os.path.exists(dir):
        os.makedirs(dir)
    # set the filename to the date and time
    filename = f"{dir}capture_{date_string}_{time_string}.avi"

    out = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))  # Output file

    # start_time = time.time()  # Record for 10 seconds
    while(cap.isOpened()):  # Record video
        ret, frame = cap.read()  # Read frame
        if ret == True:  # Write frame
            # draw the timestamp in minutes and seconds on the frame
            font = cv2.FONT_HERSHEY_SIMPLEX
            # time_string = time.strftime("%M:%S", time.localtime(time.time()))
            # set time_string to the 24 hour time
            time_string = time.strftime("%H:%M:%S", time.localtime(time.time()))

            # draw to the lower left corner of the frame
            cv2.putText(frame, time_string, (10, 470), font, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

            """ anything you want to save to the file goes above here """
            out.write(frame)  # Save the frame to output file
            """ anything you want to show on the screen but not save to the file goes below here """

            # draw "Press 'q' to quit" on the frame
            cv2.putText(frame, 'Press q to quit', (10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow('frame', frame)  # Display frame
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                break  # Quit
        else:  # Quit
            break  # Quit

    cap.release()  # Release camera
    out.release()  # Release output file
    cv2.destroyAllWindows()  # Close all windows

if __name__ == '__main__':  # Run the
    # program
    if len(sys.argv) > 1:  # Check for command line argument
        cam_index = int(sys.argv[1])  # Get camera index from command line
        record_video(cam_index=cam_index)  # Record video from a usb camera
    else:  # Default
        cam_index = 0
        # cam_index = print_cams()  # Print available cameras


    record_video(cam_index)  # Record video from a usb camera
