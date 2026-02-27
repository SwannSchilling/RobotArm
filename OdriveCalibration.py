#!/usr/bin/python3
import odrive
import odrive.utils
from odrive.enums import *
import time
import sys


SERIAL_ODRV0 = "2088399B4D4D"
SERIAL_ODRV1 = "2068399D4D4D"


def connect_odrive(serial):
    print(f"Connecting to ODrive {serial} ...")
    return odrive.find_any(serial_number=serial)


def wait_for_idle(axis):
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)


def print_axis_status(name, axis):
    print(f"\n--- {name} ---")
    print(f"Axis Error: {axis.error}")
    print(f"Motor Error: {axis.motor.error}")
    print(f"Encoder Error: {axis.encoder.error}")
    print(f"Motor Pre-Calibrated: {axis.motor.config.pre_calibrated}")
    print(f"Encoder Pre-Calibrated: {axis.encoder.config.pre_calibrated}")
    print(f"Encoder CPR: {axis.encoder.config.cpr}")
    print(f"Use Index: {axis.encoder.config.use_index}")
    print(f"Motor Type: {axis.motor.config.motor_type}")


def print_full_status(odrv, name):
    print(f"\n==============================")
    print(f"STATUS FOR {name}")
    print(f"==============================")
    print_axis_status(f"{name}.axis0", odrv.axis0)
    print_axis_status(f"{name}.axis1", odrv.axis1)


def calibrate_and_lock(odrv, name):
    print(f"\nStarting FULL calibration on {name}...")

    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    wait_for_idle(odrv.axis0)
    wait_for_idle(odrv.axis1)

    print("Checking errors...")

    # Axis 0
    if (odrv.axis0.error == 0 and
        odrv.axis0.motor.error == 0 and
        odrv.axis0.encoder.error == 0):

        odrv.axis0.motor.config.pre_calibrated = True
        print(f"{name}.axis0 locked.")
    else:
        print(f"{name}.axis0 ERROR — not locking.")

    # Axis 1
    if (odrv.axis1.error == 0 and
        odrv.axis1.motor.error == 0 and
        odrv.axis1.encoder.error == 0):

        odrv.axis1.motor.config.pre_calibrated = True
        print(f"{name}.axis1 locked.")
    else:
        print(f"{name}.axis1 ERROR — not locking.")

    print("Saving configuration (board will reboot)...")

    try:
        odrv.save_configuration()
    except:
        print("Reboot detected (expected).")

    print("Waiting for reboot...")
    time.sleep(5)

    print("Reconnecting...")
    return connect_odrive(name)


# =========================
# MAIN
# =========================

print("Connecting to both ODrives...")
odrv0 = connect_odrive(SERIAL_ODRV0)
odrv1 = connect_odrive(SERIAL_ODRV1)

odrv0.clear_errors()
odrv1.clear_errors()

while True:
    print("\n========== MENU ==========")
    print("1 - Show status")
    print("2 - Calibrate & lock odrv0")
    print("3 - Calibrate & lock odrv1")
    print("4 - Exit")
    print("==========================")

    choice = input("Select option: ")

    if choice == "1":
        print_full_status(odrv0, SERIAL_ODRV0)
        print_full_status(odrv1, SERIAL_ODRV1)

    elif choice == "2":
        odrv0 = calibrate_and_lock(odrv0, SERIAL_ODRV0)

    elif choice == "3":
        odrv1 = calibrate_and_lock(odrv1, SERIAL_ODRV1)

    elif choice == "4":
        print("Exiting.")
        sys.exit(0)

    else:
        print("Invalid selection.")