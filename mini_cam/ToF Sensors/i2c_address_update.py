#!/usr/bin/env python3

# set a new i2c address for the attached VL6180X sensor

import time
import I2C_VL6180X_Functions
from ST_VL6180X import VL6180X

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

# Example usage
current_i2c_address = 0x28  # Current address of the sensor
# new_i2c_address = 0x11
new_i2c_address = input("Enter new I2C address in hex (e.g., 0x11): ")
new_i2c_address = int(new_i2c_address, 16)

update_sensor_address(current_i2c_address, new_i2c_address)