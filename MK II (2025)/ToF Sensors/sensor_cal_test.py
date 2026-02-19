#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2018 Tony DiCola for Adafruit Industries
# SPDX-License-Identifier: MIT

# Demo of calibrating the part to part range offset per Application Note 4545
# for the VL6180X sensor

import time
import board
import busio
import adafruit_vl6180x
import RPi.GPIO as GPIO
import subprocess
import adafruit_dht
import sys
import select

skip = False
calibrated = False

# Create a DHT22 sensor instance, connected to pin D4
dht22 = adafruit_dht.DHT22(board.D5)

RED = "\033[31m"
RESET = "\033[0m"

cal_dist = 30

# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

sensor_addresses = [0x10, 0x11, 0x12, 0x13]  # New I2C addresses for sensors
ignored_addresses = []  # Add any addresses to ignore here


GPIO.setmode(GPIO.BCM)
led_pin = 0  # GPIO pin connected to the LED
sensor0_power_pin = 17
sensor1_power_pin = 27
sensor2_power_pin = 22
sensor3_power_pin = 10
sensor_pins = [sensor0_power_pin, sensor1_power_pin, sensor2_power_pin, sensor3_power_pin]

for pin in sensor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Turn off all sensors initially

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
        sensor = adafruit_vl6180x.VL6180X(i2c, address=target_address)
        sensor.offset = 0  # Clear any existing offset
        slot["sensor"] = sensor
        slot["available"] = True
        print(f"Sensor {index} initialized at address {hex(target_address)}")
    except Exception as e:
        print(f"{RED}Failed to initialize sensor {index} at address {hex(target_address)}: {e}{RESET}")
        try:
            GPIO.output(power_pin, GPIO.LOW)  # Power off the sensor
        except Exception:
            pass

    return slot

def update_sensor_address(current_address, target_address):
    """Update the I2C address of a sensor from current_address to target_address."""
    try:
        sensor = adafruit_vl6180x.VL6180X(i2c, address=current_address)
        sensor.address = target_address
        print(f"Updated sensor address from {hex(current_address)} to {hex(target_address)}")
        return True
    except Exception as e:
        print(f"{RED}Failed to update sensor address from {hex(current_address)} to {hex(target_address)}: {e}{RESET}")
        return False

# Initialize sensors one by one using fault-tolerant initialization
sensors = [
    init_single_sensor(i, sensor_pins[i], sensor_addresses[i])
    for i in range(len(sensor_pins))
]

print("==============================")
for slot in sensors:
    status = "available" if slot["available"] else "unavailable"
    print(f"sensor {slot['index']}: {status} at address {hex(slot['address'])}")

#Prompt the user to press the space key to calibrate the sensors within 2 seconds, otherwise read the calibration offsets from the file
print("Press the space key to calibrate the sensors within 2 seconds, otherwise read the calibration offsets from the file")
# start_time = time.time()
i, o, e = select.select([sys.stdin], [], [], 2)
if i:
    input_str = sys.stdin.readline().strip()
    if input_str == "":
        skip = True
        calibration_offset1 = 0
        calibration_offset2 = 0
        calibration_offset3 = 0
        calibration_offset4 = 0
        print("Skipping calibration and using 0 mm offsets for all sensors.")
    else:
        print(f"Calibration Requested. Place the sensors at {cal_dist} mm from the target and press Enter to continue.")
        temp = input()
        calibrated = False
else:
    calibrated = True

print("==============================\n")


if not skip:
    if calibrated:
        print("Reading calibration offsets from file...")
        try:
            with open("calibration_offsets.txt", "r") as f:
                print("Calibration offsets file found. Reading offsets...")
                lines = f.readlines()
                calibration_offset1 = int(lines[0].split(":")[1].strip().split()[0])
                calibration_offset2 = int(lines[1].split(":")[1].strip().split()[0])
                calibration_offset3 = int(lines[2].split(":")[1].strip().split()[0])
                calibration_offset4 = int(lines[3].split(":")[1].strip().split()[0])
                print("Calibration offsets read from file:")
        except FileNotFoundError:
            print("Calibration offsets file not found. Please calibrate the sensors by pressing the space key within 2 seconds.")


    if not calibrated:
        meas1 = []
        meas2 = []
        meas3 = []
        meas4 = []
        for msmt in range(10):
            if sensors[0]["available"]:
                meas1.append(sensors[0]["sensor"].range)
            if sensors[1]["available"]:
                meas2.append(sensors[1]["sensor"].range)
            if sensors[2]["available"]:
                meas3.append(sensors[2]["sensor"].range)
            if sensors[3]["available"]:
                meas4.append(sensors[3]["sensor"].range)
            time.sleep(0.5)
        average_msmt1 = sum(meas1) / 10
        average_msmt2 = sum(meas2) / 10
        average_msmt3 = sum(meas3) / 10
        average_msmt4 = sum(meas4) / 10

        print(f"Sensor 1 ave: {average_msmt1:.2f} mm")
        print(f"Sensor 2 ave: {average_msmt2:.2f} mm")
        print(f"Sensor 3 ave: {average_msmt3:.2f} mm")
        print(f"Sensor 4 ave: {average_msmt4:.2f} mm")

        # Calculate the offset required:
        calibration_offset1 = int(cal_dist - average_msmt1)
        calibration_offset2 = int(cal_dist - average_msmt2)
        calibration_offset3 = int(cal_dist - average_msmt3)
        calibration_offset4 = int(cal_dist - average_msmt4)

print(f"Sensor 1 calibration offset: {calibration_offset1} mm")
print(f"Sensor 2 calibration offset: {calibration_offset2} mm")
print(f"Sensor 3 calibration offset: {calibration_offset3} mm")
print(f"Sensor 4 calibration offset: {calibration_offset4} mm")

# Apply offset
if sensors[0]["available"]:
    sensors[0]["sensor"].offset = calibration_offset1
if sensors[1]["available"]:
    sensors[1]["sensor"].offset = calibration_offset2
if sensors[2]["available"]:
    sensors[2]["sensor"].offset = calibration_offset3
if sensors[3]["available"]:
    sensors[3]["sensor"].offset = calibration_offset4

# write the offsets to a file for later use
if not skip:
    with open("calibration_offsets.txt", "w") as f:
        f.write(f"Sensor 1 offset: {calibration_offset1} mm\n")
        f.write(f"Sensor 2 offset: {calibration_offset2} mm\n")
        f.write(f"Sensor 3 offset: {calibration_offset3} mm\n")
        f.write(f"Sensor 4 offset: {calibration_offset4} mm\n")



# continuously print the range, temperature, and humidity to the console and write them to a file for later analysis
file_timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
while True:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    if sensors[0]["available"]:
        range1 = sensors[0]["sensor"].range
    if sensors[1]["available"]:
        range2 = sensors[1]["sensor"].range
    if sensors[2]["available"]:
        range3 = sensors[2]["sensor"].range
    if sensors[3]["available"]:
        range4 = sensors[3]["sensor"].range

    try:
        temperature = dht22.temperature
        humidity = dht22.humidity
    except RuntimeError as error:
        temperature = "read error"
        humidity = "read error"

    print(f"Sensor 1: {range1} mm, Sensor 2: {range2} mm, Sensor 3: {range3} mm, Sensor 4: {range4} mm, Temp: {temperature}°C, Humidity: {humidity}%")

    # append the data to a file for later analysis
    with open(f"readings/{file_timestamp}_data_log.txt", "a") as f:
        f.write(f"{timestamp}, {range1}, {range2}, {range3}, {range4}, {temperature}, {humidity}\n")

    time.sleep(0.5)