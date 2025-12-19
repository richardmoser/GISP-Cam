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

# from cv2.videoio_registry import getBackendName
# from cv2_enumerate_cameras import supported_backends
# from cv2_enumerate_cameras import enumerate_cameras
# os.environ["XDG_SESSION_TYPE"] = "xcb"  # may need to comment this out on certain systems
#
# for backend in supported_backends:
#     print(getBackendName(backend))

""" ToF sensor setup """
import sys, time
# import I2C_VL6180X_Functions
from ST_VL6180X import VL6180X
# import tkinter as tk

# sensor power pin setup
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
sensor0_power_pin = 17
sensor1_power_pin = 27
sensor2_power_pin = 22
sensor3_power_pin = 23
sensor_pins = [sensor0_power_pin, sensor1_power_pin, sensor2_power_pin, sensor3_power_pin]

for pin in sensor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Turn off all sensors initially
time.sleep(0.1)  # Wait for sensors to power down

print("I2C device scan:")
# run i2cdetect -y 1
import subprocess
# p = subprocess.Popen(['i2cdetect', '-y', '1'], stdout=subprocess.PIPE)

# print the output line by line
# for line in p.stdout:
#     print(line.decode('utf-8').strip())

# make a list of detected addresses to ignore when powering on sensors
detected_addresses = []
# p = subprocess.Popen(['i2cdetect', '-y', '1'], stdout=subprocess.PIPE)
# for line in p.stdout:
#     line_str = line.decode('utf-8').strip()
#     if len(line_str) > 2 and line_str[0:2].isdigit() and line_str[2] == ':':
#         parts = line_str.split()
#         for part in parts[1:]:
#             if part != '--':
#                 detected_addresses.append(int(part, 16))
# print("Detected I2C addresses before powering on sensors:", [hex(addr) for addr in detected_addresses])

# Initialize sensors one by one
sensors = []
sensor_pins = [sensor0_power_pin, sensor1_power_pin, sensor2_power_pin, sensor3_power_pin]
sensor_addresses = [0x10, 0x11, 0x12, 0x13]  # New I2C addresses for sensors


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


for i, pin in enumerate(sensor_pins):
    print("==============================")
    print(f"Powering on sensor {i}...")
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)  # Wait for sensor to power up

    # Scan for new devices
    p = subprocess.Popen(['i2cdetect', '-y', '1'], stdout=subprocess.PIPE)
    current_addresses = []
    for line in p.stdout:
        line_str = line.decode('utf-8').strip()
        if len(line_str) > 2 and line_str[0:2].isdigit() and line_str[2] == ':':
            parts = line_str.split()
            for part in parts[1:]:
                if part != '--':
                    current_addresses.append(int(part, 16))
    new_addresses = [addr for addr in current_addresses if addr not in detected_addresses]
    if len(new_addresses) == 0:
        print(f"No new I2C device found for sensor {i}")
        GPIO.output(pin, GPIO.LOW)  # Power off the sensor
        continue
    current_address = new_addresses[0]
    print(f"Found new I2C device at address {hex(current_address)} for sensor {i}")
    success = update_sensor_address(current_address, sensor_addresses[i])
    if success:
        detected_addresses.append(sensor_addresses[i])
        sensor = VL6180X(sensor_addresses[i])
        sensor.default_settings()
        sensors.append(sensor)
    else:
        GPIO.output(pin, GPIO.LOW)  # Power off the sensor
print("==============================")



