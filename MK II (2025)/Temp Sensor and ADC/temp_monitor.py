#!/usr/bin/env python3

# read from the DHT22 sensor and print the temperature and humidity to the console

import time
import board
import adafruit_dht

# Create a DHT22 sensor instance, connected to pin D4
dht22 = adafruit_dht.DHT22(board.D5)

while True:
    try:
        # Read the temperature and humidity from the sensor
        temperature = dht22.temperature
        humidity = dht22.humidity

        # convert the temperature to Fahrenheit
        temperature_f = temperature * 9 / 5 + 32

        # Print the temperature and humidity to the console
        print(f"Temperature: {temperature:.1f}°C, {temperature_f:.1f}°F, Humidity: {humidity:.1f}%")
    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])

    time.sleep(2.0)  # Wait a few seconds before reading again