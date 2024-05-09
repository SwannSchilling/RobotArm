from flask import Flask, json ,request
import serial
import pandas as pd
import math

serial = serial.Serial('COM3', 115200)
# serial = serial.Serial('/dev/ttyACM0', 115200)
serial.close()
serial.open()

# motorPositions = 'c1\r\n'

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
    print(motor_c.value)
    pos = (math.radians(5*float(position_c)))
    motorPositions = str('c' + str(pos) + '\r\n')
    serial.write(motorPositions.encode())
    return json.dumps(motor_c.value)

@app.route('/set_positions/<position>', methods=['GET','POST'])
def set_positions(position):
    global motorPositions
    # print(position)
    motorPositions = str(position).replace(",",".")
    motorPositions = motorPositions.split('&')
    # motorPositions[0].replace(",",".")
    UpperRing = 5*(float(motorPositions[0])+30)
    MiddleRing = 5*(float(motorPositions[1])+60)
    LowerRing = 5*(float(motorPositions[2]))
    UpperRing = round((math.radians(UpperRing)),10)
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

    # y = {'f(x)': [2, 4, 20, 8], 'g(x)': [1, 5, 30, 5]}
    # x = [1, 2, 3, 4]
    # graph = pd.DataFrame(y, x)
    # graph.plot(kind='line', grid=True, title='my graph', ylabel='servoposition', xlabel='time', xlim=(1, 4))

    return json.dumps(motorPositions)

@app.route('/get_positions', methods=['GET'])
def get_positions():
    global motorPositions
    del motorPositions[-4:]
    UpperRing = 5 * (float(motorPositions[0]) + 30)
    MiddleRing = 5 * (float(motorPositions[1]) + 60)
    LowerRing = 5 * (float(motorPositions[2]))
    UpperRing = round((math.radians(UpperRing)), 10)
    MiddleRing = round((math.radians(MiddleRing)), 10)
    LowerRing = round((math.radians(LowerRing)), 10)
    motorPositions = [UpperRing,MiddleRing,LowerRing]
    #motorPositions = [motor_a.value,motor_b.value,motor_c.value]
    return json.dumps(motorPositions)

@app.route('/')
def index():
  return 'SPM Index Page'

if __name__ == '__main__':
    # app.debug = True
    app.run(host="0.0.0.0")
    # app.run()

