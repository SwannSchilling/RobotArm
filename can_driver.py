import time
from can_CubeMarsArduino import RobotArm
import time

arm = RobotArm("can0")
try:
    arm.set_position(0.0, speed_deg_per_sec=30.0, repeat=100)
    time.sleep(3)
    arm.set_position(180.0, speed_deg_per_sec=30.0, repeat=100)
    time.sleep(3)
    arm.set_position(0.0, speed_deg_per_sec=30.0, repeat=100)
    time.sleep(3)
    arm.set_position(90.0, speed_deg_per_sec=30.0, repeat=100)
    time.sleep(3)
    
    telem = arm.read_telemetry()
    if telem:
        print(f"🎯 Position: {telem['position']:.1f}°")
finally:
    arm.close()  # Always clean up