#!/usr/bin/python3
import odrive
from odrive.enums import *
import odrive.utils
from odrive.utils import start_liveplotter
import time
import serial
from time import sleep


print(odrive.__version__)
odrive_0 = True
odrive_1 = True
# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")

if odrive_0:
    print("odrive_0 found")
    odrv0 = odrive.find_any(serial_number="2088399B4D4D")

    odrv0.axis0.controller.config.vel_limit = 100
    odrv0.axis1.controller.config.vel_limit = 100
    odrv0.axis0.controller.config.vel_gain = 0.02
    odrv0.axis1.controller.config.vel_gain = 0.02
    odrv0.axis0.controller.config.pos_gain = 2
    odrv0.axis1.controller.config.pos_gain = 2
    odrv0.axis0.controller.config.input_filter_bandwidth = 0.1
    odrv0.axis1.controller.config.input_filter_bandwidth = 0.1

if odrive_1:
    print("odrive_1 found")
    odrv1 = odrive.find_any(serial_number="2068399D4D4D")
    hw_major = odrv1.hw_version_major
    hw_minor = odrv1.hw_version_minor
    print(str(hw_major)+"."+str(hw_minor))
    fw_major = odrv1.fw_version_major
    fw_minor = odrv1.fw_version_minor
    fw_revision = odrv1.fw_version_revision
    print(str(fw_major)+"."+str(fw_minor)+"."+str(fw_revision  ))
    odrv1.axis0.controller.config.vel_limit = 100
    odrv1.axis1.controller.config.vel_limit = 50
    odrv1.axis0.controller.config.vel_gain = 0.02
    odrv1.axis1.controller.config.vel_gain = 0.01
    odrv1.axis0.controller.config.pos_gain = 2
    odrv1.axis1.controller.config.pos_gain = 1
    odrv1.axis0.controller.config.input_filter_bandwidth = 0.1
    odrv1.axis1.controller.config.input_filter_bandwidth = 0.1


    # odrv1.axis0.motor.config.current_lim = 5   # Example current limit in Amps
    # odrv1.axis1.motor.config.current_lim = 5  # Example current limit in Amps
    
    # odrv1.axis0.motor.config.calibration_current = 5
    # odrv1.axis1.motor.config.calibration_current = 5
        

    # odrv0.axis0.controller.config.current_lim = 30  # Example current limit in Amps
    # odrv0.axis1.controller.config.current_lim = 30  # Example current limit in Amps

    # odrv1.axis0.controller.config.current_lim = 30  # Example current limit in Amps
    # odrv1.axis1.controller.config.current_lim = 30  # Example current limit in Amps

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

# Calibrate motor and wait for it to finish
print("starting calibration...")
# odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

if odrive_0:
    odrv0.axis0.requested_state = INPUT_MODE_POS_FILTER
    odrv0.axis1.requested_state = INPUT_MODE_POS_FILTER
    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

if odrive_1:
    odrv1.axis0.requested_state = INPUT_MODE_POS_FILTER
    odrv1.axis1.requested_state = INPUT_MODE_POS_FILTER
    while odrv1.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    while odrv1.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    errors_odrv0 = odrive.utils.dump_errors(odrv0, True)
    errors_odrv1 = odrive.utils.dump_errors(odrv1, True)
    odrv0.clear_errors()
    odrv1.clear_errors()
    #odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

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
    odrv1.axis1.controller.input_pos = value
    errors_odrv1 = odrive.utils.dump_errors(odrv1, True)
    odrv1.clear_errors()
#    axis = int(input("enter axis: "))
#
#    if axis in odrv_dict:
#        odrv_dict[axis].input_pos = value
#    else:
#        print("Invalid axis. Please enter a value between 0 and 3.")
#        break

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


