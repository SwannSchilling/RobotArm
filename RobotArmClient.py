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
from WaveshareServoController import WaveshareServoController

ODrive = False  # Set to False if not using ODrive
# Storm32 (only if SPM is True)
SPM = False  # Set to False if not using Storm32
# Arduino Nano (only if Gripper is True)
Gripper = False  # Set to False if not using Gripper
# Temperature threshold in Celsius to set 🔥 WARNING: Overheat!
TEMP_THRESHOLD = 30.0

# Initialize last positions and timeouts
last_odrive_positions = [float('-inf')] * 4
last_position_change_time = [0] * 4  # Track the last time each motor's position changed
idle_timeout = 10.0  # Time in seconds after which to idle the motor if position hasn't changed
idle_threshold = 0.01  # Define a threshold for position change to avoid floating-point issues

start_moving = True
position = ''

# Initialize with your angle range
controller = WaveshareServoController(
    servo_ids=[1, 2, 3],
    angle_range=(-45, 45),
    position_range=(500, 3500)  # Safe range
)

# def get_ip_address():
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         # doesn't have to be reachable, just used to get local IP
#         s.connect(("8.8.8.8", 80))
#         ip = s.getsockname()[0]
#     except Exception:
#         ip = "127.0.0.1"
#     finally:
#         s.close()

#     print(f"\n💻 Flask server should be accessible at http://{ip}:5000 \n")
#     return ip

# get_ip_address()

# Device finder function
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# Find ports
pico_port = find_device(0x239A, 0x80F4)
storm32_port = find_device(0x0483, 0x5740)
nano_port = find_device(0x1A86, 0x7523)
waveshare_servo_port = find_device(0x1A86, 0x55D3)

if pico_port:
    print(f"✅ Found Pico on {pico_port}")
    try:
        serial_Pico = serial.Serial(pico_port, 115200, timeout=1)
        print("✅ Pico serial port opened.")
    except serial.SerialException as e:
        print(f"❌ Failed to open Pico serial port: {e}")
else:
    print("❌ Pico not found.")

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

if waveshare_servo_port:
    print(f"✅ Found Waveshare Servo Adapter on {waveshare_servo_port}")
    try:
        serial_Waveshare = serial.Serial(waveshare_servo_port, 115200, timeout=1)
        print("✅ Waveshare Servo serial port opened.")
    except serial.SerialException as e:
        print(f"❌ Failed to open Waveshare Servo serial port: {e}")
else:
    print("❌ Waveshare Servo Adapter not found.")

# Initialize variables      
counter_num = 0
stored_positions = [0,0,0]

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
    print("❌ Not connecting to the Odrive this time...")

time.sleep(2)  # Wait for the Pico to reset

print("\nStarting the robot arm!")
print("Reading temperature for safety shutdown... 🔥🔥🔥\n")

running_event = threading.Event()
running_event.set()

# TEMP_THRESHOLD = 30

def read_serial():
    while running_event.is_set():
        try:
            line = serial_Pico.readline().decode("utf-8").strip()
            if line:
                try:
                    temp_c, _ = map(float, line.split(","))
                    print(f"Temp: {temp_c:.2f} °C", end='')
                    if temp_c > TEMP_THRESHOLD:
                        print("  🔥 WARNING: Overheat!")
                        shutdown_motors()
                        running_event.clear()
                        break  # Exit cleanly after overheat
                    else:
                        print()
                except ValueError:
                    print("⚠️ Malformed line:", line)
        except Exception as e:
            print(f"Serial read error: {e}")
            break

def poll_flask():
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
                print(f"Warning: received {len(split_positions)} positions, expected 8")
                continue

            # parse safely
            motorPositions = [float(pos) for pos in split_positions]

            Base_Rotation = motorPositions[3]
            LowerHinge_Rotation = motorPositions[4]
            UpperHinge_Rotation = motorPositions[5]
            EndEffector_Rotation = motorPositions[6]

            # print(f"Base: {Base_Rotation}, Lower: {LowerHinge_Rotation}, Upper: {UpperHinge_Rotation}, End: {EndEffector_Rotation}")

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


            if SPM == True:
                UpperRing = 5*(float(motorPositions[0])+30)
                MiddleRing = 5*(float(motorPositions[1])+60)
                LowerRing = 5*(float(motorPositions[2]))
                # UpperRing = 25*(float(motorPositions[2]))
                # MiddleRing = 25*(float(motorPositions[1]))
                # LowerRing = 25*(float(motorPositions[0]))
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
        
        except requests.exceptions.Timeout:
            print("Request timeout.")
        except Exception as e:
            print(f"Error: {e}")

        # except NameError:
        #     print("Error")
        #     exit()
        # # This will return immediately if the event is cleared
        # if not running_event.wait(timeout=0.05):
        #     break

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

exit()

# print("\nStarting the robot arm!")
# print("Reading temperature for safety shutdown... 🔥🔥🔥\n")

# running_event = threading.Event()
# running_event.set()

# TEMP_THRESHOLD = 30

# def read_serial():
#     while running_event.is_set():
#         try:
#             line = serial_Pico.readline().decode("utf-8").strip()
#             if line:
#                 try:
#                     temp_c, _ = map(float, line.split(","))
#                     print(f"Temp: {temp_c:.2f} °C", end='')
#                     if temp_c > TEMP_THRESHOLD:
#                         print("  🔥 WARNING: Overheat!")
#                         shutdown_motors()
#                         running_event.clear()
#                         break  # Exit cleanly after overheat
#                     else:
#                         print()
#                 except ValueError:
#                     print("⚠️ Malformed line:", line)
#         except Exception as e:
#             print(f"Serial read error: {e}")
#             break

# def poll_flask():
#     while running_event.is_set():
#         try:
#             url = 'http://127.0.0.1:5000/return_positions'
#             response = requests.get(url, timeout=0.5)
#             print(f'Response body: {response.text}')
#         except requests.exceptions.Timeout:
#             print("Request timeout.")
#         except Exception as e:
#             print(f"Error contacting server: {e}")
        
#         # This will return immediately if the event is cleared
#         if not running_event.wait(timeout=0.05):
#             break

# def shutdown_motors():
#     print("Shutting down motors...")
#     try:
#         odrv1.axis1.requested_state = AXIS_STATE_IDLE
#         odrv1.axis0.requested_state = AXIS_STATE_IDLE
#         odrv0.axis1.requested_state = AXIS_STATE_IDLE
#         odrv0.axis0.requested_state = AXIS_STATE_IDLE
#     except Exception as e:
#         print(f"Error during motor shutdown: {e}")

# # Create and start threads
# serial_thread = threading.Thread(target=read_serial, name="SerialReader")
# flask_thread = threading.Thread(target=poll_flask, name="FlaskPoller")

# # Don't use daemon threads - we want controlled shutdown
# serial_thread.start()
# flask_thread.start()

# try:
#     # Wait for either thread to finish or event to be cleared
#     while running_event.is_set() and (serial_thread.is_alive() or flask_thread.is_alive()):
#         time.sleep(0.1)
    
#     # If we get here due to overheat, wait a moment for both threads to finish cleanly
#     if not running_event.is_set():
#         print("Waiting for threads to finish...")
#         serial_thread.join(timeout=1.0)
#         flask_thread.join(timeout=1.0)
        
# except KeyboardInterrupt:
#     print("\nUser interruption. Stopping...")
#     running_event.clear()
#     shutdown_motors()

# # Final cleanup - wait for threads to finish
# serial_thread.join(timeout=2.0)
# flask_thread.join(timeout=2.0)

# print("System stopped.")
# sys.exit()

# exit()
# print("\nStarting the robot arm!")
# print("Reading temperature for safety shutdown... 🔥🔥🔥\n")

# running_event = threading.Event()
# running_event.set()

# def read_serial():
#     while running_event.is_set():
#         line = serial_Pico.readline().decode("utf-8").strip()
#         if line:
#             try:
#                 temp_c, _ = map(float, line.split(","))
#                 print(f"Temp: {temp_c:.2f} °C", end='')
#                 if temp_c > TEMP_THRESHOLD:
#                     print("  🔥 WARNING: Overheat!")
#                     shutdown_motors()
#                     running_event.clear()
#                 else:
#                     print()
#             except ValueError:
#                 print("⚠️ Malformed line:", line)

# def poll_flask():
#     while running_event.is_set():
#         try:
#             url = f'http://127.0.0.1:5000/return_positions'
#             response = requests.get(url, timeout=0.5)
#             print(f'Response body: {response.text}')
#         except requests.exceptions.Timeout:
#             print("Request timeout.")
#         except Exception as e:
#             print(f"Error contacting server: {e}")
        
#         # Use running_event.wait() with timeout instead of sleep loop
#         # This will return immediately if the event is cleared
#         if not running_event.wait(timeout=0.05):  # 50ms timeout
#             break  # Event was cleared, exit immediately

# def shutdown_motors():
#     print("Shutting down motors...")
#     odrv1.axis1.requested_state = AXIS_STATE_IDLE
#     odrv1.axis0.requested_state = AXIS_STATE_IDLE
#     odrv0.axis1.requested_state = AXIS_STATE_IDLE
#     odrv0.axis0.requested_state = AXIS_STATE_IDLE

# serial_thread = threading.Thread(target=read_serial)
# serial_thread.daemon = True # Set as daemon

# flask_thread = threading.Thread(target=poll_flask)
# flask_thread.daemon = True # Set as daemon

# serial_thread.start()
# flask_thread.start()

# try:
#     # Your main loop no longer needs to check the event itself.
#     # It just needs to keep the program alive.
#     # The .join() with a timeout is a robust way to do this.
#     while serial_thread.is_alive() and flask_thread.is_alive():
#         serial_thread.join(timeout=0.1)
#         flask_thread.join(timeout=0.1)
# except KeyboardInterrupt:
#     print("\nUser interruption. Stopping...")
#     running_event.clear()

# # With daemon threads, the .join() calls are no longer strictly necessary
# # for shutdown, but it's good practice to wait briefly for them.
# # The main point is that when this 'try...except' block ends, the program WILL exit.

# print("System stopped.")
# sys.exit()
