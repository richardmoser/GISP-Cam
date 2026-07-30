#!/usr/bin/env python3

# use PMW to control the brightness of the LED
import RPi.GPIO as GPIO
import time

led_pin = 18  # GPIO pin connected to the LED
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

pwm = GPIO.PWM(led_pin, 1000)  # Set frequency to 1 kHz
pwm.start(0)  # Start with LED off

try:
    while True:
        for duty_cycle in range(0, 101, 5):  # Increase brightness
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
        for duty_cycle in range(100, -1, -5):  # Decrease brightness
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()