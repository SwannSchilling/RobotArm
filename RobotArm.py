#!/usr/bin/python3
import odrive
from odrive.enums import *
from odrive.utils import start_liveplotter
import time
from flask import Flask, json ,request
import serial
import numpy as np
import math
from time import sleep

#import pandas as pd

ODrive = False
# ODrive = True
#SPM = False
SPM = True

Gripper = True

try:
    serial_SPM = serial.Serial('COM3', 115200)
    # serial = serial.Serial('/dev/ttyACM0', 115200)
    serial_SPM.close()
    serial_SPM.open()
    serial_Gripper = serial.Serial('COM21', 115200)
    serial_Gripper.close()
    serial_Gripper.open()
except serial.serialutil.SerialException:
    print("No device connected...")
    connected = False

time.sleep(2)

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

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

if ODrive == True:
    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    odrv0 = odrive.find_any(serial_number="2088399B4D4D")
    odrv1 = odrive.find_any(serial_number="2068399D4D4D")

    # Find an ODrive that is connected on the serial port /dev/ttyUSB0
    #my_drive = odrive.find_any("serial:/dev/ttyUSB0")

    odrv0.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
    odrv0.axis1.controller.config.vel_integrator_gain = 0.3333333432674408
    odrv1.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
    odrv1.axis1.controller.config.vel_integrator_gain = 0.3333333432674408
    # odrv0.axis0.controller.config.vel_integrator_gain = 0
    # odrv0.axis1.controller.config.vel_integrator_gain = 0
    # odrv1.axis0.controller.config.vel_integrator_gain = 0
    # odrv1.axis1.controller.config.vel_integrator_gain = 0
    odrv0.axis0.controller.config.vel_gain = 0.01
    odrv0.axis1.controller.config.vel_gain = 0.01
    odrv1.axis0.controller.config.vel_gain = 0.01
    odrv1.axis1.controller.config.vel_gain = 0.01

    odrv0.axis0.controller.config.vel_limit = 100
    odrv0.axis1.controller.config.vel_limit = 100
    odrv1.axis0.controller.config.vel_limit = 100
    odrv1.axis1.controller.config.vel_limit = 100
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
    #start_liveplotter(lambda:[odrv1.axis0.encoder.pos_estimate, odrv1.axis0.controller.pos_setpoint,odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
    #start_liveplotter(lambda: [odrv0.axis1.encoder.pos_estimate, odrv0.axis1.controller.pos_setpoint])
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

@app.route('/set_positions/<position>', methods=['GET','POST'])
def set_positions(position):
    try:
        global serial_SPM
        # global motorPositions
        print(position)
        motorPositions = str(position).replace(",",".")
        motorPositions = motorPositions.split('&')

        Base_Rotation = float(motorPositions[3])
        LowerHinge_Rotation = float(motorPositions[4])
        UpperHinge_Rotation = float(motorPositions[5])
        EndEffector_Rotation = float(motorPositions[6])

        Base_Rotation = round((Base_Rotation/360)*40,3)
        LowerHinge_Rotation = round((LowerHinge_Rotation/360)*40,3)
        UpperHinge_Rotation = round((UpperHinge_Rotation/360)*40,3)
        EndEffector_Rotation =round((EndEffector_Rotation/360)*40,3)

        if ODrive == True:
            odrv1.axis1.controller.input_pos = Base_Rotation
            odrv1.axis0.controller.input_pos = LowerHinge_Rotation
            odrv0.axis1.controller.input_pos = UpperHinge_Rotation
            odrv0.axis0.controller.input_pos = EndEffector_Rotation

            # motorPositions[3] = Base_Rotation
            # motorPositions[4] = LowerHinge_Rotation
            # motorPositions[5] = UpperHinge_Rotation
            # motorPositions[5] = EndEffector_Rotation
            # counter(motorPositions)

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
            print(UpperRing_Rotation)
            serial_SPM.write(UpperRing_Rotation.encode())
            MiddleRing_Rotation = str('b' + str(MiddleRing) + '\r\n')
            print(MiddleRing_Rotation)
            serial_SPM.write(MiddleRing_Rotation.encode())
            LowerRing_Rotation = str('c' + str(LowerRing) + '\r\n')
            print(LowerRing_Rotation)
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
    
    except NameError:
        print("Error")
        exit()

    # y = {'f(x)': [2, 4, 20, 8], 'g(x)': [1, 5, 30, 5]}
    # x = [1, 2, 3, 4]
    # graph = pd.DataFrame(y, x)
    # graph.plot(kind='line', grid=True, title='my graph', ylabel='servoposition', xlabel='time', xlim=(1, 4))

    return json.dumps(motorPositions)

@app.route('/get_positions', methods=['GET'])
def get_positions():
    motorPositions = [motor_a.value,motor_b.value,motor_c.value]
    return json.dumps(motorPositions)

@app.route('/')
def index():
    ip_address = request.remote_address
    return "Requester IP: " + ip_address

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