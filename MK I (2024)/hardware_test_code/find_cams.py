"""
Author: Richard Moser
Date Modified: 03Dec24
Purpose: print all available cameras to the console
"""

import sys
import os
import time
os.environ["OPENCV_LOG_LEVEL"]="FATAL"
import cv2

# Get the number of possible cameras
num_cams = 5
for i in range(num_cams):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera {i} is available")
    cap.release()
    cv2.destroyAllWindows()



