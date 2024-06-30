#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter
import time
import os
import pprint
import pygame
import serial
import numpy as np
from scipy.interpolate import griddata

class SPMLookupTable:
    def __init__(self, data):
        self.data = data
        self.points = data[:, 1:3]  # x and y coordinates (assuming they're in columns 1 and 2)
        self.values_m1 = data[:, 3]  # m1 values
        self.values_m2 = data[:, 4]  # m2 values
        self.values_m3 = data[:, 5]  # m3 values

        # Store the min and max values for x and y
        self.x_min, self.x_max = np.min(self.points[:, 0]), np.max(self.points[:, 0])
        self.y_min, self.y_max = np.min(self.points[:, 1]), np.max(self.points[:, 1])

    def lookup(self, x, y):
        # Check if the point is within the data range
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            print(f"Warning: Input point ({x}, {y}) is outside the data range.")
            return np.array([np.nan, np.nan, np.nan])

        point = np.array([[x, y]])
        m1 = griddata(self.points, self.values_m1, point, method='linear')[0]
        m2 = griddata(self.points, self.values_m2, point, method='linear')[0]
        m3 = griddata(self.points, self.values_m3, point, method='linear')[0]
        return np.array([m1, m2, m3])
    
# Load data
data = np.loadtxt('data.csv', delimiter=',', skiprows=1)
# print("Data shape:", data.shape)
# print("First few rows:")
# print(data[:5])  # Print first 5 rows to verify column order

# Create lookup table
lut = SPMLookupTable(data)

SPM = False
last_update_time = 0
update_interval = 0.5  # Minimum interval between updates in seconds


from serial.tools import list_ports
# hwinfo --short    --> hwinfo can also be used to list devices
# lsusb             --> short info
# usb-devices       --> shows alls device info
# sudo dmesg        --> check connectifity

def find_serial_device(device_signature):
    """Return the device path based on vender & product ID.
    
    The device is something like (like COM4, /dev/ttyUSB0 or /dev/cu.usbserial-1430)
    """
    candidates = list(list_ports.grep(device_signature))
    if not candidates:
        raise ValueError(f'No device with signature {device_signature} found')
    if len(candidates) > 1:
        raise ValueError(f'More than one device with signature {device_signature} found')
    return candidates[0].device

if SPM:
    try:
        print(find_serial_device('0483:5740'))
        SPM_port = find_serial_device('0483:5740')
        print('found SPM port...')
    except:
        print('No Device Found With Given ID...')
        exit()

    try:
        # serial_SPM = serial.Serial('COM3', 115200)
        # serial_SPM = serial.Serial('/dev/ttyACM0', 115200)
        serial_SPM = serial.Serial(SPM_port, 115200)
        serial_SPM.close()
        serial_SPM.open()
    except serial.serialutil.SerialException:
        print("No device connected...")
        connected = False
        exit()

    time.sleep(2)

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def add(value):
    value = value + .5
    return value
    # if value < 40:
    #     return value
    # else:
    #     return 40

def sub(value):
    value = value -.5
    return value
    # if value > 0:
    #     return value
    # else:
    #     return 0

class Motor:
    """
    Manages motor positions.
    """
    num_of_motors = 0
    scale = 1.0

    def __init__(self, name, value):
        self.name = name
        self.value = value
        Motor.num_of_motors += 1
        # pass


    def return_value(self):
        """returns the motor value"""
        return self.value

    def apply_scale(self):
        self.value = int(self.value * self.scale)

motor_a = Motor("a",0)
motor_b = Motor("b",0)
motor_c = Motor("c",0)

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""

        pygame.init()
        pygame.joystick.init()
        print("number of joystick:",pygame.joystick.get_count())
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        """Listen for events to happen"""

        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value, 2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.

                #os.system('clear')
                
                #pprint.pprint(self.button_data)
                #pprint.pprint(self.axis_data)
                #pprint.pprint(self.hat_data)
                global start_moving
                global calibration
                global new_value_x_left

                global last_update_time
                current_time = time.time()

                value_x_left = self.controller.get_axis(0)/2
                value_y_left = self.controller.get_axis(1)/2

                if current_time - last_update_time < update_interval:
                    continue
                else:
                    last_update_time = current_time
                    x, y = value_x_left, value_y_left
                    result = lut.lookup(x, y)
                    print(f"For x={x}, y={y}: result={result}")
                    m1, m2, m3 = result
                    print(f"m1={m1:.4f}, m2={m2:.4f}, m3={m3:.4f}")

                # if self.button_data[1] == True:
                #     if calibration == True:
                #         new_value_x = add(new_value_x)
                #         my_drive.axis1.controller.input_pos = new_value_x
                #         my_drive.axis0.controller.input_pos = new_value_x
                #         print("button pressed")
                # if self.button_data[3] == True:
                #     if calibration == True:
                #         new_value_x = sub(new_value_x)
                #         my_drive.axis1.controller.input_pos = new_value_x
                #         my_drive.axis0.controller.input_pos = new_value_x
                #     print("button pressed")
                # if self.button_data[2] == True:
                #     start_moving = True
                #     calibration = False
                #     print("button pressed")

if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()