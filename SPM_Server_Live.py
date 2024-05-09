#!/usr/bin/python3
from flask import Flask, json ,request
import serial
import numpy as np
import math
from time import sleep
#import pandas as pd

try:
    serial = serial.Serial('COM3', 115200)
    # serial = serial.Serial('/dev/ttyACM0', 115200)
    serial.close()
    serial.open()
except serial.serialutil.SerialException:
    print("No device connected...")
    connected = False

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


def counter(positions):
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
        serial.write(UpperRing_Rotation.encode())
        MiddleRing_Rotation = str('b' + str(MiddleRing) + '\r\n')
        print(MiddleRing_Rotation)
        serial.write(MiddleRing_Rotation.encode())
        LowerRing_Rotation = str('c' + str(LowerRing) + '\r\n')
        print(LowerRing_Rotation)
        serial.write(LowerRing_Rotation.encode())
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
    # global motorPositions
    print(position)
    motorPositions = str(position).replace(",",".")
    motorPositions = motorPositions.split('&')

    # counter(motorPositions)

    # UpperRing = 5*(float(motorPositions[0])+30)
    # MiddleRing = 5*(float(motorPositions[1])+60)
    # LowerRing = 5*(float(motorPositions[2]))
    # UpperRing = round((math.radians(UpperRing)),10)
    # MiddleRing = round((math.radians(MiddleRing)), 10)
    # LowerRing = round((math.radians(LowerRing)), 10)
    # UpperRing_Rotation = str('a' + str(UpperRing) + '\r\n')
    # print(UpperRing_Rotation)
    # serial.write(UpperRing_Rotation.encode())
    # MiddleRing_Rotation = str('b' + str(MiddleRing) + '\r\n')
    # print(MiddleRing_Rotation)
    # serial.write(MiddleRing_Rotation.encode())
    # LowerRing_Rotation = str('c' + str(LowerRing) + '\r\n')
    # print(LowerRing_Rotation)
    # serial.write(LowerRing_Rotation.encode())

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

# if __name__ == '__main__':
    # app.run(debug=True, port=80, host='0.0.0.0')
    # app.debug = True
    # app.run(debug=False, port=5000, host="0.0.0.0")
    # app.run(debug=True, port=8080,host="192.168.2.114")
    # app.run()

    # flask run -h 0.0.0.0 to get flask working remotely


