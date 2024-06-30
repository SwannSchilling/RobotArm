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

left = True
right = True
old_value_x_left = 0
new_value_x_left = 0
old_value_y_left = 0
new_value_y_left = 0
old_value_x_right = 0
new_value_x_right = 0
old_value_y_right = 0
new_value_y_right = 0
# start_moving = False
start_moving = True
calibration = True

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any(serial_number="2088399B4D4D")
odrv1 = odrive.find_any(serial_number="2068399D4D4D")

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

odrv0.axis0.controller.config.vel_limit = 100
odrv0.axis1.controller.config.vel_limit = 100
odrv1.axis0.controller.config.vel_limit = 100
odrv1.axis1.controller.config.vel_limit = 100
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis1.controller.config.vel_gain = 0.02
odrv1.axis0.controller.config.vel_gain = 0.02
odrv1.axis1.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.pos_gain = 2
odrv0.axis1.controller.config.pos_gain = 2
odrv1.axis0.controller.config.pos_gain = 2
odrv1.axis1.controller.config.pos_gain = 2
odrv0.axis0.controller.config.input_filter_bandwidth = 0.1
odrv0.axis1.controller.config.input_filter_bandwidth = 0.1
odrv1.axis0.controller.config.input_filter_bandwidth = 0.1
odrv1.axis1.controller.config.input_filter_bandwidth = 0.1

# Calibrate motor and wait for it to finish
print("starting calibration...")
# odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

odrv0.axis0.requested_state = INPUT_MODE_POS_FILTER
odrv0.axis1.requested_state = INPUT_MODE_POS_FILTER
odrv1.axis0.requested_state = INPUT_MODE_POS_FILTER
odrv1.axis1.requested_state = INPUT_MODE_POS_FILTER

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv0.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv1.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv1.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
start_liveplotter(lambda:[odrv1.axis0.encoder.pos_estimate, odrv1.axis0.controller.pos_setpoint,odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])

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

                os.system('clear')
                #pprint.pprint(self.button_data)
                #pprint.pprint(self.axis_data)
                #pprint.pprint(self.hat_data)
                global start_moving
                global calibration
                global new_value_x_left

                if 0 in self.axis_data:
                    if start_moving == True:
                        global old_value_x_left
                        value = (self.axis_data[0])
                        #value = int(scale(value, (-1.0, +1.0), (0.0, 40.0)))
                        value = int(scale(value, (-1.0, +1.0), (-10.0, 10.0)))
                        if old_value_x_left != value:
                            print("changed")
                            old_value_x_left = value
                            odrv0.axis0.controller.input_pos = value

                if 1 in self.axis_data:
                    if start_moving == True:
                        global old_value_y_left
                        value = (self.axis_data[1])
                        #value = new_value_x + int(scale(value, (-1.0, +1.0), (0.0, 40.0)))
                        value = new_value_x_left + int(scale(value, (-1.0, +1.0), (-10.0, 10.0)))
                        if old_value_y_left != value:
                            print("changed")
                            old_value_y_left = value
                            odrv0.axis1.controller.input_pos = value

                if 3 in self.axis_data:
                    if start_moving == True:
                        global old_value_x_right
                        value = (self.axis_data[3])
                        value = int(scale(value, (-1.0, +1.0), (-10.0, 10.0)))
                        if old_value_x_right != value:
                            print("changed")
                            old_value_x_right = value
                            odrv1.axis1.controller.input_pos = -1 * value

                if 4 in self.axis_data:
                    if start_moving == True:
                        global old_value_y_right
                        value = (self.axis_data[4])
                        value = new_value_x_right + int(scale(value, (-1.0, +1.0), (-10.0, 10.0)))
                        if old_value_y_right != value:
                            print("changed")
                            old_value_y_right = value
                            odrv1.axis0.controller.input_pos = -1 * value

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