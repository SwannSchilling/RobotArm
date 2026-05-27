#!/usr/bin/env python3
"""
AK-Series MVP: Position-Velocity Loop (Mode 6) Demonstration
============================================================
This script demonstrates precise position control of the AK-Series
robotic actuator using the Position-Velocity Loop (Mode 6) CAN protocol.

KEY INSIGHT: The motor requires CONTINUOUS CAN command frames to keep moving.
Just like the bash loop: for i in {1..50}; do cansend can0 ...; sleep 0.05; done

Usage:
    python3 can_ak_series_mvp.py

Prerequisites:
    pip install python-can pyserial

CAN Bus Setup:
    ip link set can0 down type can bitrate 500000
    ip link set can0 up type can bitrate 500000
"""

import struct
import time
import sys
import can

# ============================================================================
# PROTOCOL CONSTANTS
# ============================================================================

# Control Mode IDs
MODE_DUTY_CYCLE = 0
MODE_CURRENT_LOOP = 1
MODE_CURRENT_BRAKE = 2
MODE_VELOCITY_LOOP = 3
MODE_POSITION_LOOP = 4
MODE_SET_ORIGIN = 5
MODE_POSITION_VELOCITY = 6  # Our primary mode for MVP
MODE_FORCE_CONTROL_MIT = 8
MODE_MOTOR_DISABLE = 15
MODE_FEEDBACK_CONFIG = 16

# Feedback Function IDs
FEEDBACK_START_FRAME = 0x2C
FEEDBACK_STATUS = 0x29
FEEDBACK_EXTENDED_POSITION = 0x2A


def make_can_id(mode_id, drive_id):
    """Construct CAN ID: (mode_id << 8) | drive_id"""
    return (mode_id << 8) | drive_id


def pack_position_velocity(pos_deg, speed_erpm, accel_erpm_s2):
    """
    Pack Mode 6 (Position-Velocity) payload.
    
    Protocol format (8 bytes, Big-Endian):
    - Bytes 0-3: Position (int32, scale: /10000.0 = degrees)
    - Bytes 4-5: Speed (int16, scale: *10 = ERPM)
    - Bytes 6-7: Acceleration (int16, scale: *10 = ERPM/s²)
    
    Returns 8-byte packed payload.
    """
    pos_raw = int(pos_deg * 10000.0)
    speed_raw = int(speed_erpm / 10)
    accel_raw = int(accel_erpm_s2 / 10)
    
    # Clamp acceleration to non-negative
    if accel_raw < 0:
        accel_raw = 0
    
    # Pack: int32 (position) + int16 (speed) + int16 (acceleration)
    payload = struct.pack('>ihh', pos_raw, speed_raw, accel_raw)
    return payload


def pack_position_loop(pos_deg):
    """
    Pack Mode 4 (Position Loop) payload.
    
    Protocol format (4 bytes, Big-Endian int32):
    - Scale: /10000.0 = degrees
    
    Returns 4-byte packed payload.
    """
    pos_raw = int(pos_deg * 10000.0)
    return struct.pack('>i', pos_raw)


def decode_feedback(data):
    """
    Decode feedback frame (ID 0x29) - 8 bytes.
    
    Protocol format:
    - Bytes 0-1: Position (int16, scale: *0.1 = degrees)
    - Bytes 2-3: Speed (int16, scale: *10.0 = ERPM)
    - Bytes 4-5: Current (int16, scale: *0.01 = Amps)
    - Byte 6: Temperature (int8 = °C)
    - Byte 7: Error code (uint8)
    """
    if len(data) < 8:
        return None
    
    pos_raw = (data[0] << 8) | data[1]
    spd_raw = (data[2] << 8) | data[3]
    cur_raw = (data[4] << 8) | data[5]
    
    # Convert to signed 16-bit
    if pos_raw > 32767:
        pos_raw -= 65536
    if spd_raw > 32767:
        spd_raw -= 65536
    if cur_raw > 32767:
        cur_raw -= 65536
    
    # Temperature (int8)
    temp = data[6]
    if temp > 127:
        temp -= 256
    
    return {
        'position': pos_raw * 0.1,      # degrees
        'speed': spd_raw * 10.0,        # ERPM
        'current': cur_raw * 0.01,     # Amps
        'temperature': temp,            # °C
        'error': data[7],                # error code
    }


ERROR_NAMES = {
    0: "None",
    1: "Over-temperature",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temperature",
    7: "Motor lock-up",
}


def error_name(code):
    return ERROR_NAMES.get(code, f"Unknown ({code})")


# ============================================================================
# CUBE MARS ACTUATOR CLASS
# ============================================================================

class AkSeriesActuator:
    """
    AK-Series Robotic Actuator Controller
    
    Communicates via CAN bus using the CubeMars protocol (V3.2.0).
    
    KEY INSIGHT: The motor requires CONTINUOUS CAN command frames to keep
    moving. Sending a single burst is not enough - you must keep broadcasting
    the command at ~20Hz while the motor is moving.
    
    Quick Start:
        arm = AkSeriesActuator("can0", drive_id=104)
        arm.set_origin()
        arm.move_to(180.0, speed=2000, accel=2000)
    """
    
    def __init__(self, channel="can0", drive_id=104, bitrate=500000):
        self.drive_id = drive_id
        self.channel = channel
        
        # Open CAN bus (interface parameter for python-can >= 4.2.0)
        try:
            self.bus = can.Bus(channel=channel, interface='socketcan', bitrate=bitrate)
        except TypeError:
            # Fallback for older python-can versions
            self.bus = can.Bus(channel=channel, bustype='socketcan', bitrate=bitrate)
        print(f"[OK] Connected to {channel} @ {bitrate//1000} kbps | Drive ID: {drive_id}")
    
    def send_control(self, mode_id, data_bytes, repeat=1, delay=0.02):
        """Send a control frame to the motor."""
        can_id = make_can_id(mode_id, self.drive_id)
        
        msg = can.Message(
            arbitration_id=can_id,
            data=list(data_bytes),
            is_extended_id=True,
            dlc=len(data_bytes)
        )
        
        for _ in range(repeat):
            self.bus.send(msg)
            time.sleep(delay)
        
        return can_id
    
    # ----------------------------------------------------------------
    # MODE 5: SET ORIGIN
    # ----------------------------------------------------------------
    def set_origin(self, permanent=False):
        """
        Set the current rotor position as the origin (zero position).
        
        Args:
            permanent: If True, save to Flash (survives power cycle)
                       If False, temporary (lost on power-off)
        """
        mode_byte = 1 if permanent else 0
        self.send_control(MODE_SET_ORIGIN, bytes([mode_byte]), repeat=1)
        time.sleep(0.3)
        kind = "permanent" if permanent else "temporary"
        print(f"[OK] Origin set ({kind})")
    
    # ----------------------------------------------------------------
    # MODE 6: POSITION-VELOCITY LOOP (CONTINUOUS BROADCAST)
    # ----------------------------------------------------------------
    def move_to(self, pos_deg, speed_erpm=2000, accel_erpm_s2=2000,
                send_hz=20, wait_for_arrival=True, tolerance_deg=2.0, timeout=10.0):
        """
        Move to a target position using Mode 6 (Position-Velocity Loop).
        
        THE MOTOR REQUIRES CONTINUOUS CAN COMMAND FRAMES TO KEEP MOVING.
        This method broadcasts commands at `send_hz` frequency while polling
        feedback. This mimics the bash loop behavior:
            for i in {1..50}; do cansend can0 00000668#...; sleep 0.05; done
        
        Args:
            pos_deg: Target position in degrees (-36000 to 36000)
            speed_erpm: Maximum speed in ERPM (default 2000)
            accel_erpm_s2: Acceleration in ERPM/s² (default 2000)
            send_hz: Frequency to resend CAN frames (default 20 Hz = every 50ms)
            wait_for_arrival: Block until position is reached
            tolerance_deg: Acceptable deviation from target
            timeout: Maximum wait time in seconds
            
        Returns:
            True if position reached within timeout, False otherwise
        """
        payload = pack_position_velocity(pos_deg, speed_erpm, accel_erpm_s2)
        can_id = make_can_id(MODE_POSITION_VELOCITY, self.drive_id)
        
        msg = can.Message(
            arbitration_id=can_id,
            data=list(payload),
            is_extended_id=True,
            dlc=len(payload)
        )
        
        print(f"[CMD] Move to {pos_deg:.1f}° | Speed: {speed_erpm} ERPM | Accel: {accel_erpm_s2} ERPM/s²")
        print(f"      (Continuous broadcast at {send_hz} Hz)")
        
        if not wait_for_arrival:
            # Just send a short burst
            frames = int(send_hz * 0.5)
            for _ in range(frames):
                self.bus.send(msg)
                time.sleep(1.0 / send_hz)
            return True
        
        # Continuous broadcast + feedback polling
        feedback_id = make_can_id(FEEDBACK_STATUS, self.drive_id)
        self.bus.set_filters([{'can_id': feedback_id, 'can_mask': 0x1FFFFFFF, 'extended': True}])
        
        start_time = time.time()
        last_print = time.time()
        send_interval = 1.0 / send_hz
        next_send_time = start_time
        frame_count = 0
        
        while time.time() - start_time < timeout:
            now = time.time()
            
            # Broadcast command at regular intervals
            if now >= next_send_time:
                self.bus.send(msg)
                frame_count += 1
                next_send_time = now + send_interval
            
            # Try to receive feedback (short timeout to keep the loop responsive)
            try:
                msg_recv = self.bus.recv(timeout=0.01)
                if msg_recv and len(msg_recv.data) >= 8:
                    feedback = decode_feedback(list(msg_recv.data))
                    
                    if feedback:
                        # Print status every 200ms
                        if now - last_print > 0.2:
                            err_str = f" [ERR:{error_name(feedback['error'])}]" if feedback['error'] else ""
                            print(f"  [FEED] {feedback['position']:7.1f}° | "
                                  f"{feedback['speed']:7.0f} ERPM | "
                                  f"{feedback['current']:5.2f}A | "
                                  f"{feedback['temperature']}°C{err_str}")
                            last_print = now
                        
                        # Check if reached target
                        if abs(feedback['position'] - pos_deg) <= tolerance_deg:
                            print(f"[OK] Reached {pos_deg:.1f}° (actual: {feedback['position']:.1f}°) after {frame_count} frames")
                            return True
            except can.CanError:
                pass
            
            # Small sleep to prevent busy-waiting
            time.sleep(0.005)
        
        print(f"[WARN] Timeout waiting for {pos_deg:.1f}° (sent {frame_count} frames)")
        return False
    
    # ----------------------------------------------------------------
    # MODE 4: SIMPLE POSITION LOOP (CONTINUOUS BROADCAST)
    # ----------------------------------------------------------------
    def set_position(self, pos_deg, send_hz=20, duration=0.5):
        """
        Move to position using Mode 4 (Position Loop - max speed/accel).
        
        Continuously broadcasts the command for `duration` seconds at `send_hz` Hz.
        
        Args:
            pos_deg: Target position in degrees
            send_hz: Broadcasting frequency (default 20 Hz)
            duration: How long to keep broadcasting (default 0.5s)
        """
        payload = pack_position_loop(pos_deg)
        can_id = make_can_id(MODE_POSITION_LOOP, self.drive_id)
        
        msg = can.Message(
            arbitration_id=can_id,
            data=list(payload),
            is_extended_id=True,
            dlc=len(payload)
        )
        
        frames_to_send = int(send_hz * duration)
        print(f"[CMD] Position: {pos_deg:.1f}° (Mode 4, {frames_to_send} frames over {duration}s)")
        
        for _ in range(frames_to_send):
            self.bus.send(msg)
            time.sleep(1.0 / send_hz)
    
    # ----------------------------------------------------------------
    # FEEDBACK READING
    # ----------------------------------------------------------------
    def read_feedback(self, timeout=0.1):
        """
        Read the latest feedback frame from the motor.
        
        Returns a dict with keys: position, speed, current, temperature, error
        """
        feedback_id = make_can_id(FEEDBACK_STATUS, self.drive_id)
        
        self.bus.set_filters([
            {'can_id': feedback_id, 'can_mask': 0x1FFFFFFF, 'extended': True}
        ])
        
        msg = self.bus.recv(timeout=timeout)
        if msg and len(msg.data) >= 8:
            return decode_feedback(list(msg.data))
        return None
    
    def get_position(self):
        """Quick read of current position in degrees."""
        fb = self.read_feedback(timeout=0.1)
        return fb['position'] if fb else 0.0

    # ----------------------------------------------------------------
    # MOTOR CONTROL
    # ----------------------------------------------------------------
    def disable_motor(self):
        """Disable motor output (Mode 15)."""
        self.send_control(MODE_MOTOR_DISABLE, bytes([0]*8), repeat=1)
        print("[OK] Motor disabled")
    
    def close(self):
        """Close the CAN bus connection."""
        self.bus.shutdown()
        print("[OK] CAN bus closed")


# ============================================================================
# MVP DEMONSTRATION
# ============================================================================

def print_separator(title=""):
    if title:
        print(f"\n{'='*60}")
        print(f" {title}")
        print(f"{'='*60}")
    else:
        print("\n" + "-"*40)


def main():
    print_separator("AK-Series MVP: Position-Velocity Loop Demo")
    print("This script demonstrates precise position control")
    print("using Mode 6 (Position-Velocity Loop).")
    print("KEY: Motor requires continuous CAN frames to keep moving.\n")
    
    # Configuration
    DRIVE_ID = 104  # 0x68
    CHANNEL = "can0"
    
    # Speed/acceleration settings
    SPEED_ERPM = 2000        # 2000 ERPM
    ACCEL_ERPM_S2 = 2000     # 2000 ERPM/s²
    
    # Tolerance for position arrival detection
    TOLERANCE_DEG = 2.0      # ±2 degrees
    
    # Create actuator instance
    arm = AkSeriesActuator(channel=CHANNEL, drive_id=DRIVE_ID)
    
    try:
        # =========================================================================
        # STEP 1: SET ORIGIN
        # =========================================================================
        print_separator("Step 1: Set Temporary Origin")
        arm.set_origin(permanent=False)
        time.sleep(0.5)
        
        # Read initial position
        fb = arm.read_feedback(timeout=0.5)
        if fb:
            print(f"Initial position: {fb['position']:.1f}°")
            print(f"Initial error code: {error_name(fb['error'])}")
        else:
            print("[WARN] No feedback received - check CAN wiring")
        
        # =========================================================================
        # STEP 2: DEMONSTRATION SEQUENCE
        # =========================================================================
        print_separator("Step 2: Position Control Demonstration")
        print("Continuous CAN broadcast while polling feedback...\n")
        
        # Define target positions
        targets = [180.0, 0.0, 90.0, 180.0]
        
        for i, target in enumerate(targets, 1):
            print(f"\n--- Move #{i}: Target = {target:.1f}° ---")
            
            # Move to target (continuous broadcast until arrival)
            success = arm.move_to(
                pos_deg=target,
                speed_erpm=SPEED_ERPM,
                accel_erpm_s2=ACCEL_ERPM_S2,
                send_hz=20,
                wait_for_arrival=True,
                tolerance_deg=TOLERANCE_DEG,
                timeout=10.0
            )
            
            # Final verification
            fb = arm.read_feedback(timeout=0.5)
            if fb:
                deviation = abs(fb['position'] - target)
                status = "PASS" if deviation <= TOLERANCE_DEG else "CLOSE"
                print(f"  Final: {fb['position']:.1f}° (deviation: {deviation:.1f}°) [{status}]")
            
            # Brief pause between moves
            time.sleep(0.5)
        
        # =========================================================================
        # STEP 3: QUICK OSCILLATION TEST
        # =========================================================================
        print_separator("Step 3: Quick Oscillation Test")
        print("Moving between 0° and 180° three times...\n")
        
        for cycle in range(3):
            print(f"Cycle {cycle+1}/3:")
            for deg in [0.0, 180.0]:
                arm.move_to(
                    pos_deg=deg,
                    speed_erpm=SPEED_ERPM,
                    accel_erpm_s2=ACCEL_ERPM_S2,
                    send_hz=20,
                    wait_for_arrival=True,
                    tolerance_deg=TOLERANCE_DEG,
                    timeout=10.0
                )
                time.sleep(0.2)
        
        # =========================================================================
        # CLEANUP
        # =========================================================================
        print_separator("Cleanup")
        arm.disable_motor()
        arm.close()
        
        print("\n[MVP] All tests completed!")
        
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user")
        try:
            arm.disable_motor()
            arm.close()
        except:
            pass
    except Exception as e:
        print(f"\n[ERROR] {e}")
        try:
            arm.disable_motor()
            arm.close()
        except:
            pass


if __name__ == "__main__":
    main()