#!/usr/bin/python3
import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter
import time
import serial
from time import sleep

odrive_0 = False
odrive_1 = True
axis_0_0 = False
axis_0_1 = False
axis_1_0 = False
axis_1_1 = True

vel_limit = 50
vel_gain = 0.01
pos_gain = 1
input_filter_bandwidth = 0.1

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
if not odrive_0 and not odrive_1:
    print("no odrive set in parameter!")
    exit()
if odrive_0 and not axis_0_0 and not axis_0_1:
    print("no axis selected on odrive0")
    exit()
if odrive_1 and not axis_1_0 and not axis_1_1:
    print("no axis seleceted on odrive1")
    exit()

if odrive_0:
    print("odrive_0 found")
    odrv0 = odrive.find_any(serial_number="2088399B4D4D")
    if axis_0_0:
        odrv0.axis0.controller.config.vel_limit = vel_limit
        odrv0.axis0.controller.config.vel_gain = vel_gain
        odrv0.axis0.controller.config.pos_gain = pos_gain
        odrv0.axis0.controller.config.input_filter_bandwidth = input_filter_bandwidth
    if axis_0_1:
        odrv0.axis1.controller.config.vel_limit = vel_limit
        odrv0.axis1.controller.config.vel_gain = vel_gain
        odrv0.axis1.controller.config.pos_gain = pos_gain
        odrv0.axis1.controller.config.input_filter_bandwidth = input_filter_bandwidth

if odrive_1:
    print("odrive_1 found")
    odrv1 = odrive.find_any(serial_number="2068399D4D4D")
    if axis_1_0:
        odrv1.axis0.controller.config.vel_limit = vel_limit
        odrv1.axis0.controller.config.vel_gain = vel_gain
        odrv1.axis0.controller.config.pos_gain = pos_gain
        odrv1.axis0.controller.config.input_filter_bandwidth = input_filter_bandwidth
    if axis_1_1:
        odrv1.axis1.controller.config.vel_limit = vel_limit
        odrv1.axis1.controller.config.vel_gain = vel_gain
        odrv1.axis1.controller.config.pos_gain = pos_gain
        odrv1.axis1.controller.config.input_filter_bandwidth = input_filter_bandwidth

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

# Calibrate motor and wait for it to finish
print("starting calibration...")
# odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

if odrive_0:
    if axis_0_0:
        odrv0.axis0.requested_state = INPUT_MODE_POS_FILTER
        while odrv0.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    if axis_0_1:
        odrv0.axis1.requested_state = INPUT_MODE_POS_FILTER
        while odrv0.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

if odrive_1:
    if axis_1_0:
        odrv1.axis0.requested_state = INPUT_MODE_POS_FILTER
        while odrv1.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    if axis_1_1:
        while odrv1.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
#start_liveplotter(lambda:[odrv1.axis0.encoder.pos_estimate, odrv1.axis0.controller.pos_setpoint,odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])

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

odrv_dict = {}
if odrive_0 :
    odrv_dict.update({0: odrv0.axis0.controller, 1: odrv0.axis1.controller})
if odrive_1 :
    odrv_dict.update({2: odrv1.axis0.controller, 3: odrv1.axis1.controller})

# odrv_dict = {0: odrv0.axis0.controller, 1: odrv0.axis1.controller, 
#              2: odrv1.axis0.controller, 3: odrv1.axis1.controller}

while True:
    value = int(input("enter position: "))
    axis = int(input("enter axis: "))

    if axis in odrv_dict:
        odrv_dict[axis].input_pos = value
    else:
        print("Invalid axis. Please enter a value between 0 and 3.")
        break

# while True:
#     value = input("Enter position: ")
#     value = int(value)
#     axis = input("Enter axis: ")
#     axis = int(axis)
#     if axis == 0:
#         odrv0.axis0.controller.input_pos = value
#     elif axis == 1:
#         odrv0.axis1.controller.input_pos = value
#     elif axis == 2:
#         odrv1.axis0.controller.input_pos = value
#     elif axis == 3:
#         odrv1.axis1.controller.input_pos = value
#     else:
#         print("Something went wrong")
#         break


