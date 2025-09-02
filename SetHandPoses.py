#!/usr/bin/env python3
"""
Servo controller with Flask polling
Runs on Raspberry Pi, auto-detects Arduino Uno
"""

import time
import serial
import serial.tools.list_ports
import requests


class ServoController:
    def __init__(self, vid=0x2341, pid=0x0043, baudrate=115200):
        """Auto-detect Arduino Uno by VID/PID and open serial port"""
        self.port = self.find_device(vid, pid)
        if not self.port:
            raise IOError(f"❌ Arduino Uno not found (VID={hex(vid)}, PID={hex(pid)})")

        print(f"✅ Found Uno on {self.port}")
        try:
            self.serial = serial.Serial(self.port, baudrate, timeout=1)
            print("✅ Uno serial port opened.")
        except serial.SerialException as e:
            raise IOError(f"❌ Failed to open Uno serial port: {e}")

        time.sleep(2)  # Wait for Arduino reset

        # Define poses
        self.poses = {
            "idle": [90, 90, 90, 90, 90, 90, 90, 90],
            "closed": [180, 0, 180, 0, 180, 0, 180, 0],
            "one": [60, 140, 180, 0, 180, 0, 90, 90],
            "two": [60, 140, 65, 130, 180, 0, 90, 90],
            "three": [45, 120, 45, 90, 45, 90, 90, 90],
            "middle": [180, 0, 62, 120, 180, 0, 90, 90],
            "wide_open": [100, 180, 45, 97, 35, 60, 50, 150],
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
        self.serial.write(cmd.encode())
        time.sleep(0.05)

    def set_pose(self, positions, prev_positions=None):
        for servo_id, angle in enumerate(positions):
            self.write_servo(servo_id, angle)

        if prev_positions:
            max_delta = max(abs(a - b) for a, b in zip(positions, prev_positions))
            wait_time = max_delta / 60 * 0.2 + 0.2
        else:
            wait_time = 0.5

        time.sleep(wait_time)

    def poll_endpoint(self, url="http://127.0.0.1:5000/poses", interval=0.02):
        """Continuously poll Flask for new pose commands"""
        last_pose = None
        try:
            while True:
                try:
                    r = requests.get(url, timeout=0.2)
                    if r.status_code == 200:
                        pose_name = r.text.strip()
                        if pose_name and pose_name in self.poses:
                            if pose_name != last_pose:
                                print(f"🎯 New pose: {pose_name}")
                                self.set_pose(self.poses[pose_name])
                                last_pose = pose_name
                except requests.RequestException as e:
                    print(f"⚠️ Request error: {e}")

                time.sleep(interval)  # Poll every 20 ms
        except KeyboardInterrupt:
            print("🛑 Stopped polling.")


if __name__ == "__main__":
    hand_controller = ServoController()
    try:
        hand_controller.poll_endpoint()
    finally:
        hand_controller.close()
