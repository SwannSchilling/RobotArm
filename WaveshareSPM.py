import threading
import time
from scservo_sdk import *   # SCServo SDK
import serial.tools.list_ports

# -----------------------------
# Control table addresses
# -----------------------------
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56

# -----------------------------
# Protocol / defaults
# -----------------------------
PROTOCOL_END               = 0
BAUDRATE                   = 1000000
SCS_MINIMUM_POSITION_VALUE = 100
SCS_MAXIMUM_POSITION_VALUE = 4000
SCS_MOVING_SPEED           = 0   # 0 = max
SCS_MOVING_ACC             = 0   # 0 = max

# List of servo IDs
servo_ids = [1, 2, 3]

# -----------------------------
# Find Waveshare Servo Adapter
# -----------------------------
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# -----------------------------
# Initialize servos
# -----------------------------
def initialize_servo():
    DEVICENAME = find_device(0x1A86, 0x55D3)
    if DEVICENAME is None:
        print("❌ Waveshare Servo Adapter not found.")
        quit()
    print(f"✅ Found Waveshare Servo Adapter on {DEVICENAME}")

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_END)

    if not portHandler.openPort():
        print("❌ Failed to open port")
        quit()
    if not portHandler.setBaudRate(BAUDRATE):
        print("❌ Failed to set baudrate")
        quit()

    # Enable torque for all servos
    for servo_id in servo_ids:
        packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_SCS_TORQUE_ENABLE, 1)
    print("✅ Torque enabled for all servos")

    return portHandler, packetHandler

# -----------------------------
# Non-blocking servo move
# -----------------------------
def set_servo_position_nowait(portHandler, packetHandler, servo_id, target_position,
                              speed=SCS_MOVING_SPEED, acc=SCS_MOVING_ACC):
    target_position = max(SCS_MINIMUM_POSITION_VALUE,
                          min(SCS_MAXIMUM_POSITION_VALUE, target_position))
    packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_SCS_GOAL_ACC, acc)
    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_SCS_GOAL_SPEED, speed)
    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_SCS_GOAL_POSITION, target_position)

# -----------------------------
# Read servo position
# -----------------------------
def read_servo_position(portHandler, packetHandler, servo_id):
    pos, comm_result, err = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_SCS_PRESENT_POSITION)
    if comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(comm_result))
        return None
    elif err != 0:
        print(packetHandler.getRxPacketError(err))
        return None
    return pos

# -----------------------------
# Realtime joystick loop
# -----------------------------
current_targets = {servo_id: 1500 for servo_id in servo_ids}  # initial positions

def servo_joystick_loop_multi(portHandler, packetHandler, update_hz=50):
    dt = 1.0 / update_hz
    while True:
        for servo_id, target in current_targets.items():
            set_servo_position_nowait(portHandler, packetHandler, servo_id, target)
        time.sleep(dt)

# -----------------------------
# Main
# -----------------------------
portHandler, packetHandler = initialize_servo()

# Start joystick thread
threading.Thread(target=servo_joystick_loop_multi, args=(portHandler, packetHandler), daemon=True).start()

# Example: update targets dynamically
time.sleep(1)
current_targets[1] = 1000
current_targets[2] = 3000
current_targets[3] = 2000

time.sleep(1)
current_targets[1] = 2000
current_targets[2] = 1000
current_targets[3] = 3000