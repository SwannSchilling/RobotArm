#!/usr/bin/env python3
"""
Servo and Motor controller with Flask polling - INTEGRATED VERSION
Runs on Raspberry Pi, auto-detects Arduino Nano
Polls both servo poses and motor position commands
"""

import time
import serial
import serial.tools.list_ports
import requests


class ServoMotorController:
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

        # Motor position tracking
        self.g_pos = 0
        print(f"🔧 Initial motor position: {self.g_pos}")

        # Define poses
        self.poses = {
            "L_UP":     [60, 140, 180, 0, 65, 90, 180, 0],       # metal
            "L_DOWN":   [60, 140, 180, 0, 180, 0, 90, 90],       # one
            "L_LEFT":   [60, 140, 65, 130, 180, 0, 90, 90],      # two
            "L_RIGHT":  [45, 120, 45, 90, 45, 90, 90, 90],       # three
            "R_UP":     [100, 180, 45, 97, 35, 60, 50, 140],     # wide_open
            "R_DOWN":   [180, 0, 180, 0, 180, 0, 180, 0],        # closed
            "R_LEFT":   [180, 0, 62, 120, 180, 0, 90, 90],       # middle
            "R_RIGHT":  [100, 180, 45, 95, 180, 0, 180, 0],      # victory
            "L_PRESS":  [90, 90, 90, 90, 90, 90, 90, 90],        # idle
            "R_PRESS":  [170, 40, 45, 90, 45, 90, 115, 140],     # ok
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
        print(f"📤 Sending servo: {cmd.strip()}")
        self.serial.write(cmd.encode())
        
        # DEBUG: Try to read any response from Arduino
        time.sleep(0.02)  # Reduced from 0.1s to 20ms
        if self.serial.in_waiting > 0:
            response = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
            print(f"📥 Arduino response: {repr(response)}")
        
        time.sleep(0.01)  # Reduced from 50ms to 10ms between servo commands

    def send_motor_command(self, position):
        """Send motor position command (e.g., g10, g-5, g0)"""
        cmd = f"g{position}\n"
        print(f"🔧 Sending motor: {cmd.strip()}")
        self.serial.write(cmd.encode())
        time.sleep(0.02)  # Small delay for command processing
        
        # DEBUG: Try to read any response from Arduino
        if self.serial.in_waiting > 0:
            response = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
            print(f"📥 Arduino response: {repr(response)}")

    def update_motor_position(self, command):
        """Update motor position based on Flask command"""
        old_pos = self.g_pos
        
        if command == 1:
            self.g_pos -= 45
            print(f"🔧 Motor UP: {old_pos} -> {self.g_pos}")
        elif command == 2:
            self.g_pos += 45
            print(f"🔧 Motor DOWN: {old_pos} -> {self.g_pos}")
        elif command == 0:
            # Position unchanged, but still send current position
            print(f"🔧 Motor HOLD: {self.g_pos}")
        else:
            print(f"❌ Unknown motor command: {command}")
            return False
        
        # Send the motor position command
        self.send_motor_command(self.g_pos)
        return True

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

    def poll_motor_command(self, url="http://127.0.0.1:5000/motor_command"):
        """Poll Flask endpoint for motor position command"""
        try:
            r = requests.get(url, timeout=0.5)
            if r.status_code == 200:
                command = int(r.text.strip())
                return command
            else:
                print(f"⚠️ Motor endpoint returned status: {r.status_code}")
                return None
        except requests.RequestException as e:
            print(f"⚠️ Motor endpoint error: {e}")
            return None
        except ValueError:
            print(f"⚠️ Invalid motor command response: {r.text}")
            return None

    def test_single_servo(self, servo_id=0):
        """Test a single servo by moving it back and forth"""
        print(f"🧪 Testing servo {servo_id}")
        for angle in [0, 90, 180, 90]:
            print(f"Moving servo {servo_id} to {angle}°")
            self.write_servo(servo_id, angle)
            time.sleep(1)

    def poll_endpoints(self, 
                      pose_url="http://127.0.0.1:5000/current_pose", 
                      motor_url="http://127.0.0.1:5000/motor_command", 
                      interval=0.05):
        """Continuously poll Flask for both pose and motor commands"""
        last_pose = None
        last_motor_command = None
        empty_count = 0
        consecutive_same_pose = 0
        consecutive_same_motor = 0
        
        try:
            while True:
                # Poll servo pose endpoint
                try:
                    r = requests.get(pose_url, timeout=0.5)
                    if r.status_code == 200:
                        pose_name = r.text.strip()
                        
                        if pose_name:
                            empty_count = 0  # Reset empty counter
                            
                            if pose_name == last_pose:
                                consecutive_same_pose += 1
                                # Only log every 20 polls (1 second) to avoid spam
                                if consecutive_same_pose % 20 == 0:
                                    print(f"📡 Holding pose: {pose_name}")
                            else:
                                consecutive_same_pose = 0
                                print(f"📡 New pose received: '{pose_name}'")
                                
                                if pose_name in self.poses:
                                    print(f"🎯 Executing pose: {pose_name}")
                                    prev_pose = self.poses.get(last_pose) if last_pose else None
                                    self.set_pose_fast(self.poses[pose_name], prev_pose)
                                    last_pose = pose_name
                                else:
                                    print(f"❌ Unknown pose: '{pose_name}'")
                        else:
                            if last_pose is not None:  # Only print when transitioning to empty
                                print("📡 No active pose - servos idle")
                                last_pose = None
                            empty_count += 1
                                
                except requests.RequestException as e:
                    print(f"⚠️ Pose endpoint error: {e}")

                # Poll motor command endpoint
                try:
                    motor_command = self.poll_motor_command(motor_url)
                    if motor_command is not None:
                        if motor_command == last_motor_command:
                            consecutive_same_motor += 1
                            # Only log every 40 polls (2 seconds) for motor holds to reduce spam
                            if consecutive_same_motor % 40 == 0 and motor_command == 0:
                                print(f"🔧 Motor holding position: {self.g_pos}")
                        else:
                            consecutive_same_motor = 0
                            self.update_motor_position(motor_command)
                            last_motor_command = motor_command
                except Exception as e:
                    print(f"⚠️ Motor polling error: {e}")

                time.sleep(interval)  # Poll every 50ms for responsiveness
                
        except KeyboardInterrupt:
            print("🛑 Stopped polling.")

    def manual_test(self):
        """Manual testing mode for both servos and motor"""
        print("\n🧪 Manual Test Mode")
        print("Commands:")
        print("- pose <name>: Set servo pose")
        print("- motor <pos>: Set motor position")
        print("- m+ / m-: Increment/decrement motor")
        print("- quit: Exit")
        print(f"Available poses: {', '.join(self.poses.keys())}")
        
        while True:
            try:
                user_input = input("Enter command: ").strip().lower()
                
                if user_input == 'quit':
                    break
                elif user_input.startswith('pose '):
                    pose_name = user_input[5:].strip()
                    if pose_name in self.poses:
                        print(f"🎯 Setting pose: {pose_name}")
                        self.set_pose_fast(self.poses[pose_name])
                    else:
                        print(f"❌ Unknown pose: {pose_name}")
                elif user_input.startswith('motor '):
                    try:
                        position = int(user_input[6:].strip())
                        self.g_pos = position
                        self.send_motor_command(self.g_pos)
                        print(f"🔧 Motor position set to: {self.g_pos}")
                    except ValueError:
                        print("❌ Invalid motor position")
                elif user_input == 'm+':
                    self.g_pos += 1
                    self.send_motor_command(self.g_pos)
                    print(f"🔧 Motor position: {self.g_pos}")
                elif user_input == 'm-':
                    self.g_pos -= 1
                    self.send_motor_command(self.g_pos)
                    print(f"🔧 Motor position: {self.g_pos}")
                else:
                    print("❌ Unknown command")
                    
            except KeyboardInterrupt:
                print("\n🛑 Exiting manual test mode...")
                break


if __name__ == "__main__":
    hand_controller = ServoMotorController()
    try:
        # Choose mode
        print("\n🚀 Select mode:")
        print("1. Automatic polling (default)")
        print("2. Manual testing")
        print("3. Test single servo first")
        
        choice = input("Enter choice (1/2/3): ").strip()
        
        if choice == "2":
            hand_controller.manual_test()
        elif choice == "3":
            print("🧪 Testing single servo first...")
            hand_controller.test_single_servo(0)
            print("🚀 Starting endpoint polling...")
            hand_controller.poll_endpoints()
        else:
            print("🚀 Starting endpoint polling...")
            hand_controller.poll_endpoints()
            
    finally:
        hand_controller.close()
