#!/usr/bin/python3
import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter
import time
from flask import Flask, json ,request, jsonify
import serial
import numpy as np
import math
from time import sleep
import threading
import logging
import odrive.utils
import lewansoul_lx16a
from lewansoul_lx16a import ServoController
import serial.tools.list_ports
import platform

def list_serial_ports_with_details():
    ports = serial.tools.list_ports.comports()

    for port in ports:
        print(f"Device: {port.device}")
        print(f"Description: {port.description}")
        print(f"Hardware ID: {port.hwid}")
        print(f"Location: {port.location}")
        print(f"Manufacturer: {port.manufacturer}")
        print(f"Product: {port.product}")
        print(f"Interface: {port.interface}")

        if platform.system() == "Windows":
            try:
                import wmi
                c = wmi.WMI()
                for usb in c.Win32_USBControllerDevice():
                    if usb.Dependent.DeviceID == port.hwid:
                        print(f"USB Device ID: {usb.Dependent.DeviceID}")
                        print(f"USB Antecedent: {usb.Antecedent.DeviceID}")
            except ImportError:
                print("wmi library not installed. Install with pip install WMI.")

        print("-" * 40)

# list_serial_ports_with_details()
# exit()

last_update_time = 0
update_interval = 0.1  # Minimum interval between updates in seconds

ODrive = False
SPM_Gripper = False
Gripper = False
SPM = False
LX16a = False
Endeffector = False # for the end effector turn on SPM or LX16a

# Declare global variables for servo controllers
collect_data = True
collect_position_data = ""

# Initialize last positions and timeouts
last_odrive_positions = [float('-inf')] * 4
last_position_change_time = [0] * 4  # Track the last time each motor's position changed
idle_timeout = 10.0  # Time in seconds after which to idle the motor if position hasn't changed
idle_threshold = 0.01  # Define a threshold for position change to avoid floating-point issues

start_moving = True

def close_all_serial_ports():
    # List all available serial ports
    ports = serial.tools.list_ports.comports()

    for port in ports:
        try:
            # Attempt to open and then close the port
            with serial.Serial(port.device, 9600, timeout=1) as ser:
                print(f'Closing port: {port.device}')
        except serial.SerialException as e:
            print(f'Could not close port {port.device}: {e}')

# Call the function to close all serial ports
close_all_serial_ports()

# The serial ports are being looked up, its a bit messy but it works
# It needs to be cleaned up in the future

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

if SPM_Gripper == True:
    try:
        print(find_serial_device('0483:5740'))
        SPM_port = find_serial_device('0483:5740')
        print('found SPM port...')
        print(find_serial_device('1a86:7523'))
        GRIPPER_port = find_serial_device('1a86:7523')
        print('found GRIPPER port...')
    except:
        print('No Device Found With Given ID...')
        exit()

""" VID = 483
PID = 5740

device_list = list_ports.comports()
for device in device_list:
        print(device)
        if (device.vid != None or device.pid != None):
            if ('{:04X}'.format(device.vid) == VID and
                '{:04X}'.format(device.pid) == PID):
                port = device.device
                print(port)
                breakodrv0.clear_errors() 
            port = None """

if SPM_Gripper == True:
    try:
        # serial_SPM = serial.Serial('COM3', 115200)
        # serial_SPM = serial.Serial('/dev/ttyACM0', 115200)
        serial_SPM = serial.Serial(SPM_port, 115200)
        serial_SPM.close()
        serial_SPM.open()

        #serial_Gripper = serial.Serial('COM21', 115200)
        serial_Gripper = serial.Serial(GRIPPER_port, 115200)
        serial_Gripper.close()
        serial_Gripper.open()
    except serial.serialutil.SerialException:
        print("No device connected...")
        connected = False
        exit()

def find_serial_device_by_vid_pid(vid_pid):
    vid, pid = vid_pid.split(':')
    print(f'Searching for device with VID: {vid} and PID: {pid}')

    for port in serial.tools.list_ports.comports():
        # print(f'Found port: {port.device}, VID: {port.vid}, PID: {port.pid}')
        if port.vid is not None and port.pid is not None:
            if f'{port.vid:04x}' == vid and f'{port.pid:04x}' == pid:
                return port.device

    return None

if LX16a:
    try:
        LX16a_port = find_serial_device('1A86:7523')
        if LX16a_port:
            print(f'Found SPM port: {LX16a_port}')
        else:
            print('No Device Found With Given ID.')
            exit()
    except Exception as e:
        print(f'Error finding serial device: {e}')
        exit()

if Endeffector:
    try:
        controller = lewansoul_lx16a.ServoController(serial.Serial(LX16a_port, 115200, timeout=1))
        UpperRing_LX16a = controller.servo(3)
        print(f'UpperRing_LX16a initialized with ID: {UpperRing_LX16a.servo_id}')
        MiddleRing_LX16a = controller.servo(2)
        print(f'MiddleRing_LX16a initialized with ID: {MiddleRing_LX16a.servo_id}')
        LowerRing_LX16a = controller.servo(1)
        print(f'LowerRing_LX16a initialized with ID: {LowerRing_LX16a.servo_id}')
        print('Servos initialized successfully.')
        UpperRing_LX16a.move(500)
        MiddleRing_LX16a.move(500)
        LowerRing_LX16a.move(500)
        print('Servos initialized successfully.')

    except serial.SerialException as e:
        print(f'Failed to initialize servos: {e}')
        exit()

# motorPositions = 'c1\r\n'

counter_num = 0
stored_positions = [0,0,0]

app = Flask(__name__)

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

def clamp_rotation(rotation_degrees, min_degrees=-120, max_degrees=120):
    """
    Clamp rotation values to the specified range.
    
    Args:
        rotation_degrees (float): Rotation value in degrees (-180 to 180)
        min_degrees (float): Minimum allowed degrees (default: -120)
        max_degrees (float): Maximum allowed degrees (default: 120)
        
    Returns:
        float: Clamped rotation value in degrees
    """
    return max(min_degrees, min(rotation_degrees, max_degrees))

def degrees_to_servo_position(degrees, min_degrees=-120, max_degrees=120, min_pos=0, max_pos=1000):
    """
    Convert clamped rotation degrees to servo position.
    
    Args:
        degrees (float): Rotation value in degrees
        min_degrees (float): Minimum rotation in degrees (default: -120)
        max_degrees (float): Maximum rotation in degrees (default: 120)
        min_pos (int): Minimum servo position (default: 0)
        max_pos (int): Maximum servo position (default: 1000)
        
    Returns:
        int: Servo position value
    """
    # First clamp the degrees to ensure it's within range
    clamped_degrees = clamp_rotation(degrees, min_degrees, max_degrees)
    
    # Map the clamped degrees to servo position
    normalized_position = (clamped_degrees - min_degrees) / (max_degrees - min_degrees)
    servo_position = min_pos + normalized_position * (max_pos - min_pos)
    
    # Return as integer since servo positions are typically whole numbers
    return int(servo_position)

if ODrive == True:
    print("finding an odrive...")
    # Find an ODrive that is connected on the serial port /dev/ttyUSB0
    # my_drive = odrive.find_any("serial:/dev/ttyUSB3")

    # Find a connected ODrive (this will block until you connect one)
    odrv0 = odrive.find_any(serial_number="2088399B4D4D")
    odrv1 = odrive.find_any(serial_number="2068399D4D4D")

    odrv0.clear_errors()
    odrv1.clear_errors()

    odrv0.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
    odrv0.axis1.controller.config.vel_integrator_gain = 0.3333333432674408

    odrv1.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
    odrv1.axis1.controller.config.vel_integrator_gain = 0.3333333432674408
    # odrv0.axis0.controller.config.vel_integrator_gain = 0
    # odrv0.axis1.controller.config.vel_integrator_gain = 0
    # odrv1.axis0.controller.config.vel_integrator_gain = 0
    # odrv1.axis1.controller.config.vel_integrator_gain = 0

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

    #odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    errors_odrv0 = odrive.utils.dump_errors(odrv0, True)
    errors_odrv1 = odrive.utils.dump_errors(odrv1, True)
    odrv0.clear_errors() 
    odrv1.clear_errors()

    #start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
    #start_liveplotter(lambda:[odrv1.axis0.encoder.pos_estimate, odrv1.axis0.controller.pos_setpoint,odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
    #start_liveplotter(lambda: [odrv0.axis1.encoder.pos_estimate, odrv0.axis1.controller.pos_setpoint])

    # Function to start the live plotter
    def liveplot():
        start_liveplotter(lambda: [
            odrv0.axis1.motor.current_control.Iq_measured,  # Current drawn by axis1
            odrv0.axis1.encoder.pos_estimate,              # Position estimate of axis1
            odrv0.axis1.controller.pos_setpoint            # Position setpoint of axis1
        ])

    # Start the live plotter
    # liveplot()

else:
    print("not connecting to the Odrive this time...")

def counter(positions):
    global serial_SPM
    global counter_num
    global stored_positions
    # stored_positions = np.array(stored_positions)
    # positions = np.array(positions)
    # stored_positions = np.add(stored_positions,positions)
    stored_positions[0] = stored_positions[0] + float(positions[0])
    stored_positions[1] = stored_positions[1] + float(positions[1])
    stored_positions[2] = stored_positions[2] + float(positions[2])
    counter_num += 1
    print (counter_num)
    print(stored_positions)
    if counter_num ==5:
        print("reset")
        # stored_positions = stored_positions / 5
        stored_positions[0] = stored_positions[0] /5
        stored_positions[1] = stored_positions[1] /5
        stored_positions[2] = stored_positions[2] /5
        print(stored_positions)
        counter_num = 0
        UpperRing = 5 * (float(stored_positions[0]) + 30)
        MiddleRing = 5 * (float(stored_positions[1]) + 60)
        LowerRing = 5 * (float(stored_positions[2]))
        UpperRing = round((math.radians(UpperRing)), 10)
        MiddleRing = round((math.radians(MiddleRing)), 10)
        LowerRing = round((math.radians(LowerRing)), 10)
        UpperRing_Rotation = str('a' + str(UpperRing) + '\r\n')
        print(UpperRing_Rotation)
        serial_SPM.write(UpperRing_Rotation.encode())
        MiddleRing_Rotation = str('b' + str(MiddleRing) + '\r\n')
        print(MiddleRing_Rotation)
        serial_SPM.write(MiddleRing_Rotation.encode())
        LowerRing_Rotation = str('c' + str(LowerRing) + '\r\n')
        print(LowerRing_Rotation)
        serial_SPM.write(LowerRing_Rotation.encode())
        sleep(.2)
        stored_positions = [0, 0, 0]


@app.route('/set_positions_a/<position_a>', methods=['GET','POST'])
def set_positions_a(position_a):
    motor_a.value = position_a
    # print(motor_a.value)
    pos = (math.radians(5*float(position_a)))
    motorPositions = str('a' + str(pos) + '\r\n')
    serial.write(motorPositions.encode())
    return json.dumps(motor_a.value)

@app.route('/set_positions_b/<position_b>', methods=['GET','POST'])
def set_positions_b(position_b):
    motor_b.value = position_b
    # print(motor_b.value)
    pos = (math.radians(5*float(position_b)))
    motorPositions = str('b' + str(pos) + '\r\n')
    serial.write(motorPositions.encode())
    return json.dumps(motor_b.value)

@app.route('/set_positions_c/<position_c>', methods=['GET','POST'])
def set_positions_c(position_c):
    motor_c.value = position_c
    # print(motor_c.value)
    pos = (math.radians(5*float(position_c)))
    motorPositions = str('c' + str(pos) + '\r\n')
    serial.write(motorPositions.encode())
    return json.dumps(motor_c.value)

@app.route('/return_positions', methods=['GET','POST'])
def return_positions():
    motorPositions = collect_position_data
    print(motorPositions)
    return json.dumps(motorPositions)

@app.route('/set_positions/<position>', methods=['GET','POST'])
def set_positions(position):
    if collect_data:
        global collect_position_data
        collect_position_data = position
        return json.dumps(position)
    
    global last_update_time, last_odrive_positions, last_position_change_time
    current_time = time.time()
    
    if current_time - last_update_time < update_interval:
        return jsonify({"status": "rate_limit_exceeded"}), 429

    last_update_time = current_time
    
    try:
        global serial_SPM
        # global motorPositions
        # print(position)
        motorPositions = str(position).replace(",",".")
        motorPositions = motorPositions.split('&')

        Base_Rotation = float(motorPositions[3])
        LowerHinge_Rotation = float(motorPositions[4])
        UpperHinge_Rotation = float(motorPositions[5])
        EndEffector_Rotation = float(motorPositions[6])

        # Base_Rotation = round((Base_Rotation/360)*40,3)
        # LowerHinge_Rotation = round((LowerHinge_Rotation/360)*40,3)
        # UpperHinge_Rotation = round((UpperHinge_Rotation/360)*40,3)
        # EndEffector_Rotation =round((EndEffector_Rotation/360)*40,3)

        # if ODrive 
        #     global axis_0, axis_1, axis_2, axis_3
        #     odrv1.axis1.controller.input_pos = Base_Rotation        + axis_0
        #     odrv1.axis0.controller.input_pos = LowerHinge_Rotation  + axis_1
        #     odrv0.axis1.controller.input_pos = UpperHinge_Rotation  + axis_2
        #     odrv0.axis0.controller.input_pos = EndEffector_Rotation + axis_3

        #     # motorPositions[3] = Base_Rotation
        #     # motorPositions[4] = LowerHinge_Rotation
        #     # motorPositions[5] = UpperHinge_Rotation
        #     # motorPositions[5] = EndEffector_Rotation
        #     # counter(motorPositions)

        # Normalize the positions
        # Base_Rotation_norm = round((Base_Rotation / 360) * 40, 3)
        # LowerHinge_Rotation_norm = round((LowerHinge_Rotation / 360) * 40, 3)
        # UpperHinge_Rotation_norm = round((UpperHinge_Rotation / 360) * 40, 3)
        # EndEffector_Rotation_norm = round((EndEffector_Rotation / 360) * 40, 3)

        # Normalize the positions
        # Updated for new gearboxes
        Base_Rotation_norm = round((-Base_Rotation / 360) * 50, 3)  # Invert direction and update gear ratio to match the new gearbox
        LowerHinge_Rotation_norm = round((-LowerHinge_Rotation / 360) * 50, 3)  # Invert direction and update gear ratio to match the new gearbox
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


        if Endeffector == True:
            if SPM == True:
                UpperRing = 5*(float(motorPositions[0])+30)
                MiddleRing = 5*(float(motorPositions[1])+60)
                LowerRing = 5*(float(motorPositions[2]))
                UpperRing = round((math.radians(UpperRing)),10)
                MiddleRing = round((math.radians(MiddleRing)), 10)
                LowerRing = round((math.radians(LowerRing)), 10)
                UpperRing_Rotation = str('a' + str(UpperRing) + '\r\n')
                # print(UpperRing_Rotation)
                serial_SPM.write(UpperRing_Rotation.encode())
                MiddleRing_Rotation = str('b' + str(MiddleRing) + '\r\n')
                # print(MiddleRing_Rotation)
                serial_SPM.write(MiddleRing_Rotation.encode())
                LowerRing_Rotation = str('c' + str(LowerRing) + '\r\n')
                # print(LowerRing_Rotation)
                serial_SPM.write(LowerRing_Rotation.encode())

            if LX16a == True:
                # Map angles to the servo range and calculate corresponding positions
                UpperRing_angle = float(motorPositions[0]) + 30  # Adjust base offset
                MiddleRing_angle = float(motorPositions[1]) + 60 # Adjust base offset
                LowerRing_angle = float(motorPositions[2])       # No offset

                # Convert angles to servo position values (0 to 1000)
                UpperRing = degrees_to_servo_position(-UpperRing_angle * 4)  # Inverted
                MiddleRing = degrees_to_servo_position(-MiddleRing_angle * 4)  # Inverted
                LowerRing = degrees_to_servo_position(-LowerRing_angle * 4)  # Inverted

                UpperRing_LX16a.move(UpperRing)
                MiddleRing_LX16a.move(MiddleRing)
                LowerRing_LX16a.move(LowerRing)
                # UpperRing_LX16a.move_prepare(UpperRing)
                # MiddleRing_LX16a.move_prepare(MiddleRing)
                # LowerRing_LX16a.move_prepare(LowerRing)
                # controller.move_start()

        if Gripper == True:
            if not serial_Gripper.is_open:
                serial_Gripper.open()
            
            Gripper_State = int(motorPositions[7])
            if Gripper_State == 2:
                serial_Gripper.write(b'a')
            elif Gripper_State == 1:
                serial_Gripper.write(b'b')
            elif Gripper_State == 0:
                #serial_Gripper.write(b'0')
                serial_Gripper.close()
    
    except NameError:
        print("Error")
        exit()

    # y = {'f(x)': [2, 4, 20, 8], 'g(x)': [1, 5, 30, 5]}
    # x = [1, 2, 3, 4]
    # graph = pd.DataFrame(y, x)
    # graph.plot(kind='line', grid=True, title='my graph', ylabel='servoposition', xlabel='time', xlim=(1, 4))

    # Return motor positions and their states
    return jsonify({
        "motor_positions": motorPositions,
        "odrive_states": odrive_states
    })

@app.route('/check_errors', methods=['GET'])
def check_errors():
    try:
        # Assuming you have access to odrivetool's dump_errors function
        import odrive.utils
        
        errors_odrv0 = odrive.utils.dump_errors(odrv0, True)
        errors_odrv1 = odrive.utils.dump_errors(odrv1, True)

        return jsonify({
            "odrv0_errors": errors_odrv0,
            "odrv1_errors": errors_odrv1
        }), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/clear_errors', methods=['GET'])
def clear_errors():
    try:
        odrv0.clear_errors()
        odrv1.clear_errors()
        return jsonify({"status": "errors_cleared"}), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/get_positions', methods=['GET'])
def get_positions():
    motorPositions = [motor_a.value,motor_b.value,motor_c.value]
    return json.dumps(motorPositions)

@app.route('/')
def index():
    ip_address = request.remote_addr
    return "Requester IP: " + ip_address

@app.route('/axis_0/<ax_0>', methods=['GET','POST'])
def set_axis_0(ax_0):
    global axis_0
    axis_0 = axis_0 + int(ax_0)  
    return "axis_0 set to:"+ str(axis_0)

@app.route('/axis_1/<ax_1>', methods=['GET','POST'])
def set_axis_1(ax_1):
    global axis_1
    axis_1 = axis_1 + int(ax_1)
    return "axis_1 set to:"+ str(axis_1)

@app.route('/axis_2/<ax_2>', methods=['GET','POST'])
def set_axis_2(ax_2):
    global axis_2
    axis_2 = axis_2 + int(ax_2)
    return "axis_2 set to:"+ str(axis_2)

@app.route('/axis_3/<ax_3>', methods=['GET','POST'])
def set_axis_3(ax_3):
    global axis_3
    axis_3 = axis_3 + int(ax_3)
    return "axis_3 set to:"+ str(axis_3)

if __name__ == '__main__':
    # app.run(debug=True, port=80, host='0.0.0.0')
    # app.debug = True
    app.run(debug=False, port=5000, host="0.0.0.0")
    # app.run(debug=True, port=8080,host="192.168.2.114")
    # app.run()

    # flask run -h 0.0.0.0 to get flask working remotely
    # cd flask_server
    # venv\Scripts\activate
    # python app.py

    # it only works in terminal and it has to be flask run --host=0.0.0.0
    # or on Linux
    # sudo flask run --host=0.0.0.0