# main.py
from WaveshareServoController import WaveshareServoController

# Method 1: Simple usage
servo_ids = [1, 2, 3]

controller = WaveshareServoController(servo_ids)

# Move servos smoothly via background thread
controller.set_target_position(1, 2000)
controller.set_multiple_targets({2: 2000, 3: 2000})

# Clean shutdown
controller.close()

# Method 2: Context manager (recommended)
with WaveshareServoController([1, 2, 3]) as controller:
    controller.set_target_position(1, 2000)
    # Automatic cleanup on exit

# Method 3: Manual control without background thread
controller = WaveshareServoController([1, 2, 3], auto_start_thread=False)
controller.set_servo_position(1, 2000, blocking=True)