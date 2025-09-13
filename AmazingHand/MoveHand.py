# Python Script:
#!/usr/bin/env python3
"""
Auto-generated servo position script
Generated at: 26.8.2025, 21:46:32
"""

import time
import serial

class ServoController:
    def __init__(self, serial_port="COM7", baudrate=115200):
        self.serial = serial.Serial(serial_port, baudrate, timeout=0.5)
        time.sleep(2)  # Wait for Arduino reset

    def close(self):
        if self.serial.is_open:
            self.serial.close()
 
    def write_servo(self, servo_id, angle):
        angle = int(max(0, min(180, angle)))
        cmd = f"SERVO,{servo_id},{angle}\n"
        self.serial.write(cmd.encode())
        time.sleep(0.05)
        
    def set_pose(self, positions, prev_positions=None):
        for servo_id, angle in enumerate(positions):
            self.write_servo(servo_id, angle)
        
        if prev_positions:
            max_delta = max(abs(a - b) for a, b in zip(positions, prev_positions))
            # assume ~0.2s per 60° of movement
            wait_time = max_delta / 60 * 0.2 + 0.2
        else:
            wait_time = 0.5
        
        time.sleep(wait_time)
    def play_sequence(self, poses, timeout=2):
        print("Playing servo sequence...")
        for i, pose in enumerate(poses):
            print(f"Setting pose {i+1}: {pose}")
            self.set_pose(pose)
            time.sleep(timeout)  # Hold pose for 2 seconds
            
        print("Sequence complete!")

    def run_sequence(self):
        """Play all generated poses in sequence"""
        idle = [90, 90, 90, 90, 90, 90, 90, 90]
        closed = [180, 0, 180, 0, 180, 0, 180, 0] 
        one = [60, 140, 180, 0, 180, 0, 90, 90]
        two = [60, 140, 65, 130, 180, 0, 90, 90]  
        three = [45, 120, 45, 90, 45, 90, 90, 90]
        middle = [180, 0, 62, 120, 180, 0, 90, 90]  
        wide_open = [100, 180, 45, 97, 35, 60, 50, 140] 
        victory = [100, 180, 45, 95, 180, 0, 180, 0] 
        metal = [60, 140, 180, 0, 65, 90, 180, 0] 
        ok = [170, 40, 45, 90, 45, 90, 115, 140]

        # poses = [closed, one, two, three, closed, two, victory, closed, middle, closed, metal, closed, ok, closed, wide_open, closed]
        poses = [wide_open, closed]
        timeout = 1
        self.play_sequence(poses, timeout)

hand_controller = ServoController()
try:
    hand_controller.run_sequence()
finally:
    hand_controller.close()


