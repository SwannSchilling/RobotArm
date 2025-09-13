#!/usr/bin/env python3
"""
Servo controller with Flask polling - DEBUG VERSION
Runs on Raspberry Pi, auto-detects Arduino Uno
"""

import time
import serial
import serial.tools.list_ports
import requests


class ServoController:
    def __init__(self, vid=0x1A86, pid=0x7523, baudrate=115200):
        """Auto-detect Arduino Nano by VID/PID and open serial port"""
        self.port = self.find_device(vid, pid)
        if not self.port:
            raise IOError(f"❌ Arduino Nano not found (VID={hex(vid)}, PID={hex(pid)})")

        print(f"✅ Found Nano on {self.port}")
        try:
            self.serial = serial.Serial(self.port, baudrate, timeout=1)
            print("✅ Nano serial port opened.")
        except serial.SerialException as e:
            raise IOError(f"❌ Failed to open Nano serial port: {e}")

        time.sleep(2)  # Wait for Arduino reset

        # Define poses
        self.poses = {
            "idle": [90, 90, 90, 90, 90, 90, 90, 90],
            "closed": [180, 0, 180, 0, 180, 0, 180, 0],
            "one": [60, 140, 180, 0, 180, 0, 90, 90],
            "two": [60, 140, 65, 130, 180, 0, 90, 90],
            "three": [45, 120, 45, 90, 45, 90, 90, 90],
            "middle": [180, 0, 62, 120, 180, 0, 90, 90],
            "wide_open": [100, 180, 45, 97, 35, 60, 50, 140],
            "victory": [100, 180, 45, 95, 180, 0, 180, 0],
            "metal": [60, 140, 180, 0, 65, 90, 180, 0],
            "ok": [170, 40, 45, 90, 45, 90, 115, 140],
        }

    @staticmethod
    def find_device(vid, pid):
        """Search system serial ports for device matching VID/PID"""
        for port in serial.tools.list_ports.comports():
            if port.vid == vid and port.pid == pid:
                return port.device
        return None

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("🔌 Serial port closed.")

    def write_servo(self, servo_id, angle):
        angle = int(max(0, min(180, angle)))
        cmd = f"SERVO,{servo_id},{angle}\n"
        print(f"📤 Sending: {cmd.strip()}")  # DEBUG: Show what we're sending
        self.serial.write(cmd.encode())
        
        # DEBUG: Try to read any response from Arduino
        time.sleep(0.02)  # Reduced from 0.1s to 20ms
        if self.serial.in_waiting > 0:
            response = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
            print(f"📥 Arduino response: {repr(response)}")
        
        time.sleep(0.01)  # Reduced from 50ms to 10ms between servo commands

    def set_pose_fast(self, positions, prev_positions=None):
        """Fast pose setting - send all commands quickly"""
        print(f"🎯 Setting pose: {positions}")
        
        # Send all servo commands rapidly
        for servo_id, angle in enumerate(positions):
            angle = int(max(0, min(180, angle)))
            cmd = f"SERVO,{servo_id},{angle}\n"
            self.serial.write(cmd.encode())
            time.sleep(0.005)  # Just 5ms between commands
        
        # Calculate wait time based on largest movement
        if prev_positions:
            max_delta = max(abs(a - b) for a, b in zip(positions, prev_positions))
            wait_time = max_delta / 120 + 0.1  # Faster calculation
        else:
            wait_time = 0.3  # Reduced default wait
        
        print(f"⏱️ Waiting {wait_time:.2f}s for movement completion")
        time.sleep(wait_time)

    def test_single_servo(self, servo_id=0):
        """Test a single servo by moving it back and forth"""
        print(f"🧪 Testing servo {servo_id}")
        for angle in [0, 90, 180, 90]:
            print(f"Moving servo {servo_id} to {angle}°")
            self.write_servo(servo_id, angle)
            time.sleep(1)

    def poll_endpoint(self, url="http://127.0.0.1:5000/current_pose", interval=0.05):
        """Continuously poll Flask for new pose commands"""
        last_pose = None
        empty_count = 0
        consecutive_same = 0
        
        try:
            while True:
                try:
                    r = requests.get(url, timeout=0.5)
                    if r.status_code == 200:
                        pose_name = r.text.strip()
                        
                        if pose_name:
                            empty_count = 0  # Reset empty counter
                            
                            if pose_name == last_pose:
                                consecutive_same += 1
                                # Only log every 20 polls (1 second) to avoid spam
                                if consecutive_same % 20 == 0:
                                    print(f"📡 Holding pose: {pose_name}")
                            else:
                                consecutive_same = 0
                                print(f"📡 New pose received: '{pose_name}'")
                                
                                if pose_name in self.poses:
                                    print(f"🎯 Executing pose: {pose_name}")
                                    prev_pose = self.poses.get(last_pose) if last_pose else None
                                    self.set_pose_fast(self.poses[pose_name], prev_pose)  # Use fast version
                                    last_pose = pose_name
                                else:
                                    print(f"❌ Unknown pose: '{pose_name}'")
                        else:
                            if last_pose is not None:  # Only print when transitioning to empty
                                print("📡 No active pose - servos idle")
                                last_pose = None
                            empty_count += 1
                                
                except requests.RequestException as e:
                    print(f"⚠️ Request error: {e}")

                time.sleep(interval)  # Poll every 50ms for responsiveness
        except KeyboardInterrupt:
            print("🛑 Stopped polling.")


if __name__ == "__main__":
    hand_controller = ServoController()
    try:
        # First test a single servo
        print("🧪 Testing single servo first...")
        hand_controller.test_single_servo(0)
        
        print("🚀 Starting endpoint polling...")
        hand_controller.poll_endpoint()
    finally:
        hand_controller.close()