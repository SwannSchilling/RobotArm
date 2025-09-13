#!/usr/bin/env python3
"""
Auto-generated servo position script with G command support
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
    
    def send_g_command(self, value):
        """Send a G command with specified value (e.g., g45, g0, g-45)"""
        cmd = f"g{value}\n"
        print(f"Sending G command: {cmd.strip()}")
        self.serial.write(cmd.encode())
        time.sleep(0.1)  # Small delay for command processing
        
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
            time.sleep(timeout)  # Hold pose for specified seconds
            
        print("Sequence complete!")
    
    def play_sequence_with_g_commands(self, poses, g_commands, timeout=1):
        """Play poses, then G commands, then poses again"""
        print("=== Phase 1: Initial poses ===")
        self.play_sequence(poses, timeout)
        
        print("\n=== Phase 2: G commands ===")
        for g_cmd in g_commands:
            self.send_g_command(g_cmd)
            time.sleep(0.5)  # Wait between G commands
        
        print("\n=== Phase 3: Final poses ===")
        self.play_sequence(poses, timeout)

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

        # Test poses
        poses = [wide_open, closed, victory, metal]
        
        # G commands to test
        g_commands = [45, 0, -45]
        
        timeout = 1
        self.play_sequence_with_g_commands(poses, g_commands, timeout)

    def interactive_mode(self):
        """Interactive mode for manual testing"""
        print("\n=== Interactive Mode ===")
        print("Commands:")
        print("- 'g<value>' to send G command (e.g., g45, g0, g-45)")
        print("- 'pose <name>' to set a pose (e.g., pose open, pose closed)")
        print("- 'quit' to exit")
        
        # Define poses for interactive use
        poses = {
            'idle': [90, 90, 90, 90, 90, 90, 90, 90],
            'closed': [180, 0, 180, 0, 180, 0, 180, 0],
            'open': [100, 180, 45, 97, 35, 60, 50, 140],
            'victory': [100, 180, 45, 95, 180, 0, 180, 0],
            'metal': [60, 140, 180, 0, 65, 90, 180, 0],
            'ok': [170, 40, 45, 90, 45, 90, 115, 140]
        }
        
        while True:
            try:
                cmd = input("\nEnter command: ").strip()
                
                if cmd.lower() == 'quit':
                    break
                elif cmd.startswith('g'):
                    # Extract value from g command
                    try:
                        value = cmd[1:]  # Remove 'g' prefix
                        self.send_g_command(value)
                    except ValueError:
                        print("Invalid G command format. Use: g<value> (e.g., g45)")
                elif cmd.startswith('pose '):
                    # Set a pose
                    pose_name = cmd[5:].strip().lower()
                    if pose_name in poses:
                        print(f"Setting pose: {pose_name}")
                        self.set_pose(poses[pose_name])
                    else:
                        print(f"Unknown pose: {pose_name}")
                        print(f"Available poses: {', '.join(poses.keys())}")
                else:
                    print("Unknown command. Use 'g<value>', 'pose <name>', or 'quit'")
                    
            except KeyboardInterrupt:
                print("\nExiting interactive mode...")
                break
            except Exception as e:
                print(f"Error: {e}")

# Main execution
if __name__ == "__main__":
    hand_controller = ServoController()
    try:
        # Run the automated sequence
        hand_controller.run_sequence()
        
        # Optional: Enter interactive mode for manual testing
        print("\nWould you like to enter interactive mode? (y/n)")
        if input().strip().lower() == 'y':
            hand_controller.interactive_mode()
            
    finally:
        hand_controller.close()