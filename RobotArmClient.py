#!/usr/bin/python3
import serial
import serial.tools.list_ports
import odrive
import odrive.utils
from odrive.enums import *
from odrive.utils import start_liveplotter
import logging
import json 
import threading
import time
import requests
import sys
import math 
import socket
import numpy as np
from WaveshareServoController import WaveshareServoController

# Top of file — shared between control loop and Flask
latest_obs = {
    'base': 0.0, 'lower_hinge': 0.0, 'upper_hinge': 0.0, 'end_effector': 0.0,
    'upper_ring': 0.0, 'middle_ring': 0.0, 'lower_ring': 0.0, 'gripper': 0.0
}
latest_act = {
    'base': 0.0, 'lower_hinge': 0.0, 'upper_hinge': 0.0, 'end_effector': 0.0,
    'upper_ring': 0.0, 'middle_ring': 0.0, 'lower_ring': 0.0, 'gripper': 0.0
}
obs_act_lock = threading.Lock()

# logging.getLogger('werkzeug').setLevel(logging.ERROR)

ODrive = True  # Set to False if not using ODrive
# Storm32 (only if SPM is True)
SPM = False  # Set to False if not using Storm32
# Arduino Nano (only if Gripper is True)
Gripper = False  # Set to False if not using Gripper
Waveshare = True  # Set to False if not using Waveshare
SPM_Waveshare = True  # Set to False if not using SPM_Waveshare
OpenCM = True  # Set to False if not using OpenCM
# Temperature threshold in Celsius to set 🔥 WARNING: Overheat!
TEMP_THRESHOLD = 50.0

# Initialize last positions and timeouts
last_odrive_positions = [float('-inf')] * 4
last_position_change_time = [0] * 4  # Track the last time each motor's position changed
idle_timeout = 10.0  # Time in seconds after which to idle the motor if position hasn't changed
idle_threshold = 0.01  # Define a threshold for position change to avoid floating-point issues

start_moving = True

# --- FIX: Initialize global variables to prevent crashes ---
posOffset = 0 
stored_positions = [0,0,0]
serial_Pico = None 
idle_flag = False
position_changed_flag = False
gripper_open = 180
gripper_closed = 40
current_gripper_val = gripper_open # Initialize state memory

GRIPPER_OPEN = 180
GRIPPER_CLOSED = 40
MIN_DELTA = 2          # Ignore tiny changes (servo jitter guard)
SERIAL_RATE = 0.02     # 50 Hz max send rate
last_serial_time = 0
current_gripper_val = -1

if Waveshare == True:
    controller = WaveshareServoController(
    servo_ids=[1, 2, 3],
    angle_range=(-180, 180),    # Larger joint range
    reduction_ratio=20.0,
    position_range=(100, 4000)  # Use full servo range
    )
else:
    print("Not connecting to the Waveshare this time...")

position = ''

# Device finder function
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# Find ports
# --- UPDATED: Using the IDs found in your dmesg logs (Adafruit/CircuitPython) ---
pico_port = find_device(0x239A, 0x80F4)
storm32_port = find_device(0x0483, 0x5740)
nano_port = find_device(0x1A86, 0x7523)
waveshare_servo_port = find_device(0x1A86, 0x55D3)

# OpenCM 9.04
ROBOTIS_VID = 0xFFF1
ROBOTIS_PID = 0xFF48
BAUD_RATE = 115200
OpenCM_port = find_device(ROBOTIS_VID, ROBOTIS_PID)

if OpenCM_port:
    print(f"✅ Found OpenCM on {OpenCM_port}")
    try:
        serial_OpenCM = serial.Serial(OpenCM_port, 115200, timeout=1)
        print("✅ OpenCM serial port opened.")
    except serial.SerialException as e:
        print(f"❌ Failed to open OpenCM serial port: {e}")
        # Abort if OpenCM is critical
        sys.exit(1)
else:
    print("❌ OpenCM not found (Check Power Supply!).")
    # We exit here because your script relies on reading the OpenCM for safety
    sys.exit(1)

# --- FIX 2: Discared the Pico to save poweron the Raspberry Pi's USB ports ---
# if pico_port:
#     print(f"✅ Found Pico on {pico_port}")
#     try:
#         serial_Pico = serial.Serial(pico_port, 115200, timeout=1)
#         print("✅ Pico serial port opened.")
#     except serial.SerialException as e:
#         print(f"❌ Failed to open Pico serial port: {e}")
#         # Abort if Pico is critical
#         sys.exit(1)
# else:
#     print("❌ Pico not found (Check Power Supply!).")
#     # We exit here because your script relies on reading the Pico for safety
#     sys.exit(1) 

if SPM:
    if storm32_port:
        print(f"✅ Found Storm32 BGC on {storm32_port}")
        try:
            serial_SPM = serial.Serial(storm32_port, 115200, timeout=1)
            print("✅ Storm32 serial port opened.")
        except serial.SerialException as e:
            print(f"❌ Failed to open Storm32 serial port: {e}")
    else:
        print("❌ Storm32 BGC not found.")
else:
    print("❌ Not connecting to the Storm32 BGC this time...")

if Gripper:
    if nano_port:
        print(f"✅ Found Arduino Nano on {nano_port}")
        try:
            serial_Gripper = serial.Serial(nano_port, 115200, timeout=1)
            print("✅ Arduino Nano serial port opened.")
        except serial.SerialException as e:
            print(f"❌ Failed to open Arduino Nano serial port: {e}")
    else:
        print("❌ Arduino Nano not found.")
else:
    print("❌ Not connecting to the Gripper this time...")

# --- CRITICAL FIX: DO NOT OPEN WAVESHARE SERIAL MANUALLY ---
# The 'controller' object above already opened it. Opening it again breaks it.
if waveshare_servo_port:
    print(f"✅ Found Waveshare Servo Adapter on {waveshare_servo_port} (Managed by Controller)")
else:
    print("❌ Waveshare Servo Adapter not found.")


# Initialize variables      
counter_num = 0

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

axis_0 = 0
axis_1 = 0
axis_2 = 0
axis_3 = 0

vel_limit = 100
vel_gain = 0.02
pos_gain = 3
input_filter_bandwidth = 0.2

current_lim = 10
calibration_current = 10

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def map_gripper_to_servo(raw_val):
    # Clamp Unity input range first (never trust upstream)
    raw_val = max(0, min(255, raw_val))

    # Linear mapping: 0–255 → GRIPPER_CLOSED–GRIPPER_OPEN
    servo = GRIPPER_CLOSED + (
        (raw_val / 255.0) * (GRIPPER_OPEN - GRIPPER_CLOSED)
    )

    # Redundant hardware clamp (hard safety layer)
    servo = max(GRIPPER_CLOSED, min(GRIPPER_OPEN, servo))

    return int(servo)

if ODrive == True:
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

    print("starting encoder offset calibration...")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    odrv1.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    odrv1.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

    while odrv0.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    while odrv1.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    while odrv1.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    errors_odrv0 = odrive.utils.dump_errors(odrv0, True)
    errors_odrv1 = odrive.utils.dump_errors(odrv1, True)
    odrv0.clear_errors() 
    odrv1.clear_errors()

    print("--- Axis 0 ---")
    print(f"Motor Type: {odrv0.axis0.motor.config.motor_type}")
    print(f"Encoder Type: {odrv0.axis0.encoder.config.mode}")
    print(f"CPR: {odrv0.axis0.encoder.config.cpr}")
    print(f"Use Index: {odrv0.axis0.encoder.config.use_index}")
    print(f"Pre-Calibrated (Motor): {odrv0.axis0.motor.config.pre_calibrated}")
    print(f"Pre-Calibrated (Encoder): {odrv0.axis0.encoder.config.pre_calibrated}")

    def liveplot():
        start_liveplotter(lambda: [
            odrv0.axis1.motor.current_control.Iq_measured,  # Current drawn by axis1
            odrv0.axis1.encoder.pos_estimate,              # Position estimate of axis1
            odrv0.axis1.controller.pos_setpoint            # Position setpoint of axis1
        ])

else:
    print("❌ Not connecting to the Odrive this time...")

time.sleep(2)  # Wait for the Pico to reset

print("\nStarting the robot arm!")
print("Reading temperature for safety shutdown... 🔥🔥🔥\n")

running_event = threading.Event()
running_event.set()

def read_serial():
    if serial_Pico is None:
        return
        
    while running_event.is_set():
        try:
            line = serial_Pico.readline().decode("utf-8").strip()
            if line:
                try:
                    temp_c, _ = map(float, line.split(","))
                    print(f"Temp: {temp_c:.2f} °C", end='\r')
                    if temp_c > TEMP_THRESHOLD:
                        print("  🔥 WARNING: Overheat!")
                        shutdown_motors()
                        running_event.clear()
                        break  # Exit cleanly after overheat
                except ValueError:
                    # Ignore partial lines
                    pass
        except Exception as e:
            print(f"Serial read error: {e}")
            break

def poll_flask():
    global posOffset
    global current_gripper_val
    global gripper_closed
    global gripper_open 
    SERVO_INVERSIONS = {1: -1, 2: -1, 3: -1}  # Servo 1,2,3 inverted
    # Define centers based on physical mounting positions
    UPPER_RING_OFFSET  = 30.0
    MIDDLE_RING_OFFSET = 60.0
    LOWER_RING_OFFSET  = 0.0
    RING_SCALE         = 5.0
    RING_MAX_DEG       = 45.0  # your configured servo range

    def compute_ring_cmd(raw_input, offset, inversion):
        cmd = RING_SCALE * (raw_input + offset)
        cmd = max(-RING_MAX_DEG, min(RING_MAX_DEG, cmd))  # clamp to safe range
        return cmd * inversion
    
    motorPositions = [0.0] * 8  # initialized once per thread run
    print(f"Initialized motorPositions with length: {len(motorPositions)}")
    current_time = time.time()

    while running_event.is_set():
        try:
            url = 'http://127.0.0.1:5000/return_positions'
            response = requests.get(url, timeout=0.5)
            # print(f"Response body: {response.text}")

            # clean string
            data = response.text.strip('"')  # remove quotes if present
            data = data.replace(",", ".")
            split_positions = data.split('&')

            if len(split_positions) != 8:
                continue

            # parse safely
            motorPositions = [float(pos) for pos in split_positions]

            Base_Rotation = motorPositions[3]
            LowerHinge_Rotation = motorPositions[4]
            UpperHinge_Rotation = motorPositions[5]
            EndEffector_Rotation = motorPositions[6]

            # Normalize the positions
            # Updated for new gearboxes
            Base_Rotation_norm = round((-Base_Rotation / 360) * 50, 3)  # Invert direction
            LowerHinge_Rotation_norm = round((-LowerHinge_Rotation / 360) * 50, 3)  # Invert direction
            UpperHinge_Rotation_norm = round((UpperHinge_Rotation / 360) * 40, 3)
            EndEffector_Rotation_norm = round((EndEffector_Rotation / 360) * 40, 3)

            odrive_states = {}

            if ODrive:
                global start_moving, last_position_change_time, idle_flag, position_changed_flag
                if start_moving:
                    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    # Suppress default logging of HTTP requests
                    log = logging.getLogger('werkzeug')
                    log.setLevel(logging.ERROR)
                    start_moving = False
                    idle_flag = False
                    position_changed_flag = False

                global axis_0, axis_1, axis_2, axis_3
                
                new_positions = [Base_Rotation_norm, LowerHinge_Rotation_norm, UpperHinge_Rotation_norm, EndEffector_Rotation_norm]
                position_changed = False

                # Track the previous states of each axis
                previous_states = {
                    'axis1': odrv1.axis1.current_state,
                    'axis0': odrv1.axis0.current_state,
                    'axis3': odrv0.axis1.current_state,
                    'axis2': odrv0.axis0.current_state
                }

                for i, (new_pos, last_pos) in enumerate(zip(new_positions, last_odrive_positions)):
                    if abs(new_pos - last_pos) > idle_threshold:
                        position_changed = True
                        last_odrive_positions[i] = new_pos

                if position_changed:
                    if not position_changed_flag:
                        print("At least one motor position has changed")
                        position_changed_flag = True
                        idle_flag = False
                    last_position_change_time = current_time

                    # Set all motors to CLOSED_LOOP_CONTROL if they are currently in IDLE state
                    if previous_states['axis1'] == AXIS_STATE_IDLE:
                        odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv1.axis1.controller.input_pos = new_positions[0] + axis_0

                    if previous_states['axis0'] == AXIS_STATE_IDLE:
                        odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv1.axis0.controller.input_pos = new_positions[1] + axis_1

                    if previous_states['axis3'] == AXIS_STATE_IDLE:
                        odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv0.axis1.controller.input_pos = new_positions[2] + axis_2

                    if previous_states['axis2'] == AXIS_STATE_IDLE:
                        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv0.axis0.controller.input_pos = new_positions[3] + axis_3

                    # For all 4 ODrive axes:
                    obs_base        = odrv1.axis1.encoder.pos_estimate    # turns
                    obs_lower       = odrv1.axis0.encoder.pos_estimate
                    obs_upper       = odrv0.axis1.encoder.pos_estimate
                    obs_ee          = odrv0.axis0.encoder.pos_estimate

                    act_base        = odrv1.axis1.controller.pos_setpoint  # turns (same unit)
                    act_lower       = odrv1.axis0.controller.pos_setpoint
                    act_upper       = odrv0.axis1.controller.pos_setpoint
                    act_ee          = odrv0.axis0.controller.pos_setpoint

                    # Convert both to degrees in the same coordinate space
                    def odrive_to_deg_base(turns):
                        return -(turns / 50) * 360   # inverted, 50:1

                    def odrive_to_deg_lower(turns):
                        return -(turns / 50) * 360   # inverted, 50:1

                    def odrive_to_deg_upper(turns):
                        return  (turns / 40) * 360   # not inverted, 40:1

                    def odrive_to_deg_ee(turns):
                        return  (turns / 40) * 360   # not inverted, 40:1

                    print(f"Base:        obs={odrive_to_deg_base(obs_base):+.3f} | act={odrive_to_deg_base(act_base):+.3f} | err={odrive_to_deg_base(act_base - obs_base):+.3f}")
                    print(f"LowerHinge:  obs={odrive_to_deg_lower(obs_lower):+.3f} | act={odrive_to_deg_lower(act_lower):+.3f} | err={odrive_to_deg_lower(act_lower - obs_lower):+.3f}")
                    print(f"UpperHinge:  obs={odrive_to_deg_upper(obs_upper):+.3f} | act={odrive_to_deg_upper(act_upper):+.3f} | err={odrive_to_deg_upper(act_upper - obs_upper):+.3f}")
                    print(f"EndEffector: obs={odrive_to_deg_ee(obs_ee):+.3f} | act={odrive_to_deg_ee(act_ee):+.3f} | err={odrive_to_deg_ee(act_ee - obs_ee):+.3f}")
                    
                elif current_time - last_position_change_time > idle_timeout:
                    if not idle_flag:
                        print("No motor positions have changed for timeout duration")
                        idle_flag = True
                        position_changed_flag = False
                    # Set all motors to IDLE
                    odrv1.axis1.requested_state = AXIS_STATE_IDLE
                    odrv1.axis0.requested_state = AXIS_STATE_IDLE
                    odrv0.axis1.requested_state = AXIS_STATE_IDLE
                    odrv0.axis0.requested_state = AXIS_STATE_IDLE

                # Get the states of the ODrive axes
                odrive_states['axis1'] = odrv1.axis1.current_state
                odrive_states['axis0'] = odrv1.axis0.current_state
                odrive_states['axis3'] = odrv0.axis1.current_state
                odrive_states['axis2'] = odrv0.axis0.current_state

            if SPM_Waveshare:
                upper_cmd  = compute_ring_cmd(float(motorPositions[0]), UPPER_RING_OFFSET,  -1)
                middle_cmd = compute_ring_cmd(float(motorPositions[1]), MIDDLE_RING_OFFSET, -1)
                lower_cmd  = compute_ring_cmd(float(motorPositions[2]), LOWER_RING_OFFSET,  -1)

                controller.set_multiple_target_angles({
                    1: upper_cmd,
                    2: middle_cmd,
                    3: lower_cmd
                })

            global MIN_DELTA, SERIAL_RATE, last_serial_time, current_gripper_val

            GRIPPER_OPEN = 180
            GRIPPER_CLOSED = 40

            if OpenCM:
                try:
                    raw_val = int(motorPositions[7])

                    # Dataset → servo mapping
                    servo_val = map_gripper_to_servo(raw_val)

                    # -------------------------------------------------
                    # SAFETY CLAMP (HARD LIMITS)
                    # -------------------------------------------------
                    servo_val = int(np.clip(
                        servo_val,
                        GRIPPER_CLOSED,
                        GRIPPER_OPEN
                    ))

                    now = time.time()

                    # -------------------------------------------------
                    # Rate limit + jitter guard
                    # -------------------------------------------------
                    if (
                        abs(servo_val - current_gripper_val) >= MIN_DELTA and
                        now - last_serial_time >= SERIAL_RATE
                    ):
                        current_gripper_val = servo_val
                        serial_OpenCM.write(f"{current_gripper_val}\n".encode())
                        last_serial_time = now

                        # Optional debug
                        print(f"Sent to OpenCM: {current_gripper_val}")
                        print(f"Gripper raw:{raw_val} mapped:{servo_val}")
                except ValueError:
                    print("Invalid gripper value received")

                # Non-blocking feedback read
                if serial_OpenCM.in_waiting > 0:
                    try:
                        response = serial_OpenCM.readline().decode().strip()
                        print(f"OpenCM Feedback: {response}")
                    except Exception as e:
                        print(f"Serial Read Error: {e}")

            # if Gripper == True:
            #     if not serial_Gripper.is_open:
            #         serial_Gripper.open()
                
            #     Gripper_State = int(motorPositions[7])
            #     if Gripper_State == 2:
            #         serial_Gripper.write(b'a')
            #     elif Gripper_State == 1:
            #         serial_Gripper.write(b'b')
            #     elif Gripper_State == 0:
            #         #serial_Gripper.write(b'0')
            #         serial_Gripper.close()

            # ODrive observations (encoder)
            obs_base  = -(odrv1.axis1.encoder.pos_estimate / 50) * 360
            obs_lower = -(odrv1.axis0.encoder.pos_estimate / 50) * 360
            obs_upper =  (odrv0.axis1.encoder.pos_estimate / 40) * 360
            obs_ee    =  (odrv0.axis0.encoder.pos_estimate / 40) * 360

            # ODrive actions (setpoint)
            act_base  = -(odrv1.axis1.controller.pos_setpoint / 50) * 360
            act_lower = -(odrv1.axis0.controller.pos_setpoint / 50) * 360
            act_upper =  (odrv0.axis1.controller.pos_setpoint / 40) * 360
            act_ee    =  (odrv0.axis0.controller.pos_setpoint / 40) * 360

            # Ring servos
            cached = controller.get_cached_angles()
            upper_cmd  = compute_ring_cmd(float(motorPositions[0]), UPPER_RING_OFFSET,  SERVO_INVERSIONS[1])
            middle_cmd = compute_ring_cmd(float(motorPositions[1]), MIDDLE_RING_OFFSET, SERVO_INVERSIONS[2])
            lower_cmd  = compute_ring_cmd(float(motorPositions[2]), LOWER_RING_OFFSET,  SERVO_INVERSIONS[3])

            controller.set_multiple_target_angles({1: upper_cmd, 2: middle_cmd, 3: lower_cmd})

            # Atomically update shared state
            with obs_act_lock:
                latest_obs.update({
                    'base': obs_base, 'lower_hinge': obs_lower,
                    'upper_hinge': obs_upper, 'end_effector': obs_ee,
                    'upper_ring': cached.get(1, 0.0),
                    'middle_ring': cached.get(2, 0.0),
                    'lower_ring':  cached.get(3, 0.0),
                    'gripper': 0.0  # add your gripper source here
                })
                latest_act.update({
                    'base': act_base, 'lower_hinge': act_lower,
                    'upper_hinge': act_upper, 'end_effector': act_ee,
                    'upper_ring': upper_cmd, 'middle_ring': middle_cmd,
                    'lower_ring': lower_cmd,
                    'gripper': 0.0  # add your gripper source here
                })

        except requests.exceptions.Timeout:
            pass # ignore timeouts
        except Exception as e:
            print(f"Error: {e}")

def shutdown_motors():
    print("Shutting down motors...")
    try:
        odrv1.axis1.requested_state = AXIS_STATE_IDLE
        odrv1.axis0.requested_state = AXIS_STATE_IDLE
        odrv0.axis1.requested_state = AXIS_STATE_IDLE
        odrv0.axis0.requested_state = AXIS_STATE_IDLE
    except Exception as e:
        print(f"Error during motor shutdown: {e}")

# Create and start threads
serial_thread = threading.Thread(target=read_serial, name="SerialReader")
flask_thread = threading.Thread(target=poll_flask, name="FlaskPoller")

# Don't use daemon threads - we want controlled shutdown
serial_thread.start()
flask_thread.start()

try:
    # Wait for either thread to finish or event to be cleared
    while running_event.is_set() and (serial_thread.is_alive() or flask_thread.is_alive()):
        time.sleep(0.1)
    
    # If we get here due to overheat, wait a moment for both threads to finish cleanly
    if not running_event.is_set():
        print("Waiting for threads to finish...")
        serial_thread.join(timeout=1.0)
        flask_thread.join(timeout=1.0)
        
except KeyboardInterrupt:
    print("\nUser interruption. Stopping...")
    running_event.clear()
    shutdown_motors()

# Final cleanup - wait for threads to finish
serial_thread.join(timeout=2.0)
flask_thread.join(timeout=2.0)

print("System stopped.")
sys.exit()