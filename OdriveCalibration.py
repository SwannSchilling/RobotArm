#!/usr/bin/python3
import serial
import serial.tools.list_ports
import odrive
import odrive.utils
from odrive.enums import *
from odrive.utils import start_liveplotter
import time 

vel_limit = 100
vel_gain = 0.02
pos_gain = 3
input_filter_bandwidth = 0.2

current_lim = 10
calibration_current = 10

print("finding an odrive...")

# Find a connected ODrive (this will block until you connect one)
odrv0 = odrive.find_any(serial_number="2088399B4D4D")
odrv1 = odrive.find_any(serial_number="2068399D4D4D")

odrv0.clear_errors()
odrv1.clear_errors()

odrv0.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
odrv0.axis1.controller.config.vel_integrator_gain = 0.3333333432674408

odrv1.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
odrv1.axis1.controller.config.vel_integrator_gain = 0.3333333432674408

odrv0.axis0.controller.config.vel_gain = vel_gain
odrv0.axis1.controller.config.vel_gain = vel_gain
odrv1.axis0.controller.config.vel_gain = vel_gain
odrv1.axis1.controller.config.vel_gain = vel_gain

odrv0.axis0.controller.config.vel_limit = vel_limit
odrv0.axis1.controller.config.vel_limit = vel_limit

odrv1.axis0.controller.config.vel_limit = vel_limit
odrv1.axis1.controller.config.vel_limit = vel_limit

odrv0.axis0.controller.config.pos_gain = pos_gain
odrv0.axis1.controller.config.pos_gain = pos_gain

odrv1.axis0.controller.config.pos_gain = pos_gain
odrv1.axis1.controller.config.pos_gain = pos_gain

odrv0.axis0.controller.config.input_filter_bandwidth = input_filter_bandwidth
odrv0.axis1.controller.config.input_filter_bandwidth = input_filter_bandwidth

odrv1.axis0.controller.config.input_filter_bandwidth = input_filter_bandwidth
odrv1.axis1.controller.config.input_filter_bandwidth = input_filter_bandwidth

odrv1.axis0.motor.config.current_lim = current_lim
odrv1.axis1.motor.config.current_lim = current_lim
odrv1.axis0.motor.config.calibration_current = calibration_current
odrv1.axis1.motor.config.calibration_current = calibration_current

print("starting calibration...")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv0.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv1.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv1.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print("checking errors and locking motor calibration...")

# =========================
# ODRV0 AXIS0
# =========================
if (odrv0.axis0.error == 0 and
    odrv0.axis0.motor.error == 0 and
    odrv0.axis0.encoder.error == 0):

    odrv0.axis0.motor.config.pre_calibrated = True
    print("odrv0.axis0 calibration locked.")
else:
    print("odrv0.axis0 ERROR — not saving.")


# =========================
# ODRV0 AXIS1
# =========================
if (odrv0.axis1.error == 0 and
    odrv0.axis1.motor.error == 0 and
    odrv0.axis1.encoder.error == 0):

    odrv0.axis1.motor.config.pre_calibrated = True
    print("odrv0.axis1 calibration locked.")
else:
    print("odrv0.axis1 ERROR — not saving.")


# =========================
# ODRV1 AXIS0
# =========================
if (odrv1.axis0.error == 0 and
    odrv1.axis0.motor.error == 0 and
    odrv1.axis0.encoder.error == 0):

    odrv1.axis0.motor.config.pre_calibrated = True
    print("odrv1.axis0 calibration locked.")
else:
    print("odrv1.axis0 ERROR — not saving.")


# =========================
# ODRV1 AXIS1
# =========================
if (odrv1.axis1.error == 0 and
    odrv1.axis1.motor.error == 0 and
    odrv1.axis1.encoder.error == 0):

    odrv1.axis1.motor.config.pre_calibrated = True
    print("odrv1.axis1 calibration locked.")
else:
    print("odrv1.axis1 ERROR — not saving.")


# ---- SAVE CONFIGURATION (ONLY ONCE PER BOARD) ----
odrv0.save_configuration()
odrv1.save_configuration()

print("configuration saved.")


# ---- FINAL ERROR DUMP ----
odrive.utils.dump_errors(odrv0, True)
odrive.utils.dump_errors(odrv1, True)

odrv0.clear_errors()
odrv1.clear_errors()


print("--- Axis 0 (odrv0) ---")
print(f"Motor Type: {odrv0.axis0.motor.config.motor_type}")
print(f"Encoder Type: {odrv0.axis0.encoder.config.mode}")
print(f"CPR: {odrv0.axis0.encoder.config.cpr}")
print(f"Use Index: {odrv0.axis0.encoder.config.use_index}")
print(f"Pre-Calibrated (Motor): {odrv0.axis0.motor.config.pre_calibrated}")
print(f"Pre-Calibrated (Encoder): {odrv0.axis0.encoder.config.pre_calibrated}")
