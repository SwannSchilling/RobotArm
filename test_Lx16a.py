# https://github.com/maximkulkin/lewansoul-lx16a/tree/master
import serial
import lewansoul_lx16a
import time 
from serial.tools import list_ports  # Part of pyserial library

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
        print("-" * 40)

# list_serial_ports_with_details()

SERIAL_PORT = 'COM35'

# Ensure serial port is closed before opening
serial_port = serial.Serial(SERIAL_PORT, 115200, timeout=1)
serial_port.close()
time.sleep(1)  # Short delay before reopening

# Reopen the serial port
serial_port = serial.Serial(SERIAL_PORT, 115200, timeout=1)

controller = lewansoul_lx16a.ServoController(serial_port)

# print("connected")
# for i in range (0,15):
#     try:
#         print(controller.get_voltage(i))
#     except:
#         print(f"trying to connect to servo: {i}")
#         pass

# set_servo_id = 3
# controller.set_servo_id(2, set_servo_id)
# servo_id = controller.get_servo_id()
# Print the servo ID
# print(f"The ID of the connected servo is: {servo_id}")

# control servos directly
# controller.move(1, 0) # move servo ID=1 to position 100
# exit()

# or through proxy objects
servo1 = controller.servo(1)
servo2 = controller.servo(2)
servo3 = controller.servo(3)
servo4 = controller.servo(4)
servo5 = controller.servo(5)
servo6 = controller.servo(6)
servo7 = controller.servo(7)
servo8 = controller.servo(8)
servo9 = controller.servo(9)
servo10 = controller.servo(10)
servo11 = controller.servo(11)
servo12 = controller.servo(12)
servo13 = controller.servo(13)
servo14 = controller.servo(14)
servo15 = controller.servo(15)

# UpperRing = controller.servo(3)
# MiddleRing = controller.servo(2)
# LowerRing = controller.servo(1)

# UpperRing.move(500)
# MiddleRing.move(500)
# LowerRing.move(500)
pos1 = 500 # example position 0-1000 corresponds to 0°-240°
pos2 = 500 # example position 0-1000 corresponds to 0°-240°
pos3 = 500 # example position 0-1000 corresponds to 0°-240°
pos3 = 500 # example position 0-1000 corresponds to 0°-240°
pos4 = 500 # example position 0-1000 corresponds to 0°-240°
pos5 = 500 # example position 0-1000 corresponds to 0°-240°
pos6 = 500 # example position 0-1000 corresponds to 0°-240°
pos7 = 500 # example position 0-1000 corresponds to 0°-240°
pos8 = 500 # example position 0-1000 corresponds to 0°-240°
pos9 = 500 # example position 0-1000 corresponds to 0°-240°
pos10 = 500 # example position 0-1000 corresponds to 0°-240°
pos11 = 500 # example position 0-1000 corresponds to 0°-240°
pos12 = 500 # example position 0-1000 corresponds to 0°-240°
pos13 = 500 # example position 0-1000 corresponds to 0°-240°
pos14 = 500 # example position 0-1000 corresponds to 0°-240°
pos15 = 500 # example position 0-1000 corresponds to 0°-240°

#servo1.move_prepare(pos1)
#servo2.move_prepare(pos2)
#servo3.move_prepare(pos3)
servo4.move_prepare(pos4)
#servo5.move_prepare(pos5)
servo6.move_prepare(pos6)
#servo7.move_prepare(pos7)
servo8.move_prepare(pos8)
#servo9.move_prepare(pos9)
servo10.move_prepare(pos10)
#servo11.move_prepare(pos11)
servo12.move_prepare(pos12)
#servo13.move_prepare(pos13)
servo14.move_prepare(pos14)
#servo15.move_prepare(pos15)

# UpperRing.move_prepare(pos1)
# MiddleRing.move_prepare(pos2)
# LowerRing.move_prepare(pos3)
controller.move_start()

# servo1.move(500)
# servo2.move(500)
# servo3.move(500)

exit()

# synchronous move of multiple servos
servo1.move_prepare(300)
servo2.move_prepare(600)
servo3.move_prepare(900)	
controller.move_start()

# time.sleep(3)

# # Set the servo to motor mode (continuous rotation)
# servo_id = 2  # Replace with your servo's ID
# controller.set_motor_mode(servo_id, speed=1000)  # Rotate clockwise at speed 500
# def run_loop_for_seconds(duration):
#     start_time = time.time()  # Get the current time
#     end_time = start_time + duration  # Calculate the end time
#     counter = 0
#     while time.time() < end_time:
#         # Your code here
#         print("Servoposition...")
#         postion = controller.get_position(servo_id)
#         print(postion)
#         if postion >= 1000:
#             print("Exceed target position by...")
#             counter += 1
#             print(counter)
#         time.sleep(.01)  # Sleep for 1 second to avoid busy-waiting

#     print("Time's up!")

# # Example usage: Run the loop for 10 seconds
# run_loop_for_seconds(5)

# # Stop the servo
# controller.set_motor_mode(servo_id, speed=0)  # Stop rotation

# # Switch back to servo mode (position control)
# controller.set_servo_mode(servo_id)