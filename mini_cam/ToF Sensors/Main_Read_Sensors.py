#!/usr/bin/env python3

# Quick and Dirty Sensor Read for multiple sensors
# Assumes i2c addresses have been change prior to run

import sys, time
import I2C_VL6180X_Functions
from ST_VL6180X import VL6180X
# import tkinter as tk


#Initialize and report Sensor 0
sensor0_i2cid = 0x10
sensor0 = VL6180X(sensor0_i2cid)
sensor0.get_identification()
if sensor0.idModel != 0xB4:
    print("Not Valid Sensor, Id reported as ",hex(sensor0.idModel))
else:
    print("Valid Sensor, ID reported as ",hex(sensor0.idModel))
sensor0.default_settings()
# Finish Initialize Sensor 0
# ---------------------------------
#Initialize and report Sensor 1
sensor1_i2cid = 0x11
sensor1 = VL6180X(sensor1_i2cid)
sensor1.get_identification()
if sensor1.idModel != 0xB4:
    print("Not Valid Sensor, Id reported as ",hex(sensor1.idModel))
else:
    print("Valid Sensor, ID reported as ",hex(sensor1.idModel))
sensor1.default_settings()
#Finish Initialize Sensor 1
#---------------------------------
#Initialize and report Sensor 2
sensor2_i2cid = 0x12
sensor2 = VL6180X(sensor2_i2cid)
sensor2.get_identification()
if sensor2.idModel != 0xB4:
    print("Not Valid Sensor, Id reported as ",hex(sensor2.idModel))
else:
    print("Valid Sensor, ID reported as ",hex(sensor2.idModel))
sensor2.default_settings()
#Finish Initialize Sensor 2
#---------------------------------
#Initialize and report Sensor 3
sensor3_i2cid = 0x13
sensor3 = VL6180X(sensor3_i2cid)
sensor3.get_identification()
if sensor3.idModel != 0xB4:
    print("Not Valid Sensor, Id reported as ",hex(sensor3.idModel))
else:
    print("Valid Sensor, ID reported as ",hex(sensor3.idModel))
sensor3.default_settings()
#Finish Initialize Sensor 3
#---------------------------------

#Time allotted to each sensor to make a reading in sec
Range_Convergtime = 0.02

#Main Body of Program

results = 0
L0 = "0"
L1 = "0"
L2 = "0"
L3 = "0"
def Results_Label(label):
    def result():
        global results
        global L0
        global l1
        global L2
        global L3
        results +=1
        L0 = str(sensor0.get_distance())
        time.sleep(Range_Convergtime)
        L1 = str(sensor1.get_distance())
        time.sleep(Range_Convergtime)
        L2 = str(sensor2.get_distance())
        time.sleep(Range_Convergtime)
        L3 = str(sensor3.get_distance())
        time.sleep(Range_Convergtime)
        label.config(text="sensor 0 :"+L0+"mm\n"+"sensor 1 :"+L1+"mm\n"+"sensor 2 :"+L2+"mm\n"+"sensor 3 :"+L3+"mm\n"+"Readings :"+str(results))
        
        label.after(10,result)
    result()


while True:
    L0 = str(sensor0.get_distance())
    time.sleep(Range_Convergtime)
    L1 = str(sensor1.get_distance())
    time.sleep(Range_Convergtime)
    L2 = str(sensor2.get_distance())
    time.sleep(Range_Convergtime)
    L3 = str(sensor3.get_distance())
    time.sleep(Range_Convergtime)
    print("sensor 0 :"+L0+"mm\t"+"sensor 1 :"+L1+"mm\t"+"sensor 2 :"+L2+"mm\t"+"sensor 3 :"+L3+"mm")