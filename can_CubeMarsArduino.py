#!/usr/bin/env python3
"""
Python controller for MCP2515-based robot arm (matches Arduino firmware exactly).
Usage:
    arm = RobotArm("can0", controller_id=104)
    arm.set_position(180.0)      # Move to 180°
    telemetry = arm.read_telemetry()  # Get live feedback
"""

import can
import struct
import time
from typing import Optional, Dict

class RobotArm:
    # Command IDs (must match Arduino enum)
    CAN_PACKET_SET_DUTY = 0
    CAN_PACKET_SET_CURRENT = 1
    CAN_PACKET_SET_CURRENT_BRAKE = 2
    CAN_PACKET_SET_RPM = 3
    CAN_PACKET_SET_POS = 4
    CAN_PACKET_SET_ORIGIN_HERE = 5
    CAN_PACKET_SET_POS_SPD = 6
    CAN_PACKET_SET_MIT = 7  # Impedance control mode

    def __init__(self, channel: str = "can0", controller_id: int = 104, bitrate: int = 1000000):
        self.controller_id = controller_id
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.bitrate = bitrate
        print(f"🔌 RobotArm initialized on {channel} @ {bitrate//1000}k (ID: {controller_id})")

    def _make_eid(self, command_id: int) -> int:
        """Build Extended ID: controller_id | (command_id << 8)"""
        return self.controller_id | (command_id << 8)

    def _send_eid(self, command_id: int, data: bytes):
        """Send Extended frame with retry logic"""
        msg = can.Message(
            arbitration_id=self._make_eid(command_id),
            data=data,
            is_extended_id=True,  # Critical: matches Arduino's CAN_EFF_FLAG
            dlc=len(data)
        )
        self.bus.send(msg)
        time.sleep(0.02)  # Small delay to avoid bus flooding

    def set_position(self, pos_deg: float, speed_deg_per_sec: float = 50.0, rpa: float = 50.0, repeat: int = 1):
        """
        Move to target position with controlled speed.
        Matches Arduino's comm_can_set_pos_spd() exactly.
        
        Args:
            pos_deg: Target position in degrees
            speed_deg_per_sec: Movement speed (default 50°/s = SLOW)
            rpa: Ramp acceleration parameter (default 50.0)
            repeat: Number of times to send the frame (for reliability)
        """
        # Pack exactly like Arduino: big-endian, scaled values
        pos_raw = int(pos_deg * 10000.0)          # position * 10000 → int32
        spd_raw = int(speed_deg_per_sec * 10.0)   # speed * 10 → int16
        rpa_raw = int(rpa * 10.0)                 # RPA * 10 → int16
        
        # Build 8-byte payload: [pos:4][spd:2][rpa:2]
        payload = struct.pack('>i', pos_raw) + struct.pack('>hh', spd_raw, rpa_raw)
        
        command_id = self.CAN_PACKET_SET_POS_SPD  # = 6
        eid = self._make_eid(command_id)
        
        for _ in range(repeat):
            self._send_eid(command_id, payload)
        
        print(f"✅ Sent POS_SPD: {pos_deg}° @ {speed_deg_per_sec}°/s (ID: 0x{eid:03X})")

    def set_origin(self, mode: int = 0):
        """Set current position as origin. Matches comm_can_set_origin()."""
        self._send_eid(self.CAN_PACKET_SET_ORIGIN_HERE, bytes([mode]))
        print(f"✅ Sent origin command (mode={mode})")

    def read_telemetry(self, timeout: float = 0.1) -> Optional[Dict]:
        """Read latest telemetry frame (ID 0x2968). Matches motor_receive_servo()."""
        # Set filter for telemetry ID only (optional but efficient)
        self.bus.set_filters([{"can_id": 0x2968, "can_mask": 0x1FFFFFFF, "extended": True}])
        
        msg = self.bus.recv(timeout=timeout)
        if not msg or msg.arbitration_id != 0x2968 or len(msg.data) < 8:
            return None
            
        data = msg.data
        # Decode exactly like Arduino: big-endian ints, then scale
        pos_int = (data[0] << 8) | data[1]
        spd_int = (data[2] << 8) | data[3]
        cur_int = (data[4] << 8) | data[5]
        
        return {
            "position": pos_int * 0.1,      # °
            "speed": spd_int * 10.0,        # ? (RPM or deg/s)
            "current": cur_int * 0.01,      # A
            "temperature": data[6],         # °C (signed if needed)
            "error_code": data[7],          # 0 = OK
            "timestamp": msg.timestamp
        }

    def emergency_stop(self):
        """Send zero-current brake command (safe fallback)."""
        payload = struct.pack('>i', 0)  # 0 current
        self._send_eid(self.CAN_PACKET_SET_CURRENT_BRAKE, payload)
        print("🛑 Emergency stop sent")

    def close(self):
        """Clean shutdown"""
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
            print("🔌 CAN bus closed")

    # ✅ Context manager support
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False  # Don't suppress exceptions