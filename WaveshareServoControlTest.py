# main.py
from WaveshareServoController import WaveshareServoController
import json
import time
# Method 1: Simple usage
servo_ids = [1, 2, 3]

controller = WaveshareServoController(servo_ids)

# # Move servos smoothly via background thread
# controller.set_target_position(1, 2000)
# controller.set_multiple_targets({2: 2000, 3: 2000})

# # Clean shutdown
# controller.close()

# # Method 2: Context manager (recommended)
# with WaveshareServoController([1, 2, 3]) as controller:
#     controller.set_target_position(1, 2000)
#     # Automatic cleanup on exit

# # Method 3: Manual control without background thread
# controller = WaveshareServoController([1, 2, 3], auto_start_thread=False)
# controller.set_servo_position(1, 2000, blocking=True)

# print(controller.read_all_angles())
# print(controller.read_all_positions())

# print(controller.read_servo_angle(1))

# print(json.dumps(controller.get_cached_positions()))

controller.stop_realtime_control()
time.sleep(2)

def fresh_read():
    with controller._bus_lock:
        return controller._sync_read_positions()

print("Initial:")
print(fresh_read())

print("\nMoving ONLY servo 1 to +20 deg...")
controller.set_servo_angle(1, 20, blocking=False)
time.sleep(2)
print(fresh_read())  # live read, not cache

print("\nMoving ONLY servo 2 to +20 deg...")
controller.set_servo_angle(1, 0, blocking=False)
time.sleep(1)
controller.set_servo_angle(2, 20, blocking=False)
time.sleep(2)
print(fresh_read())

print("\nMoving ONLY servo 3 to +20 deg...")
controller.set_servo_angle(2, 0, blocking=False)
time.sleep(1)
controller.set_servo_angle(3, 20, blocking=False)
time.sleep(2)
print(fresh_read())

