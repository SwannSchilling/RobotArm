#!/usr/bin/env python3
"""
CubeMars AK-Series Controller - Production Implementation
Matches CAN protocol documentation V3.2.0 exactly.
"""
import can
import struct
import time
import sys

class CubeMarsActuator:
    # Control Mode IDs (from docs Table: Control Mode IDs)
    MODE_DUTY = 0
    MODE_CURRENT = 1
    MODE_BRAKE = 2
    MODE_VELOCITY = 3
    MODE_POSITION = 4
    MODE_ORIGIN = 5
    MODE_POS_VEL_ACC = 6  # ← This is what we use
    MODE_MIT = 8
    MODE_DISABLE = 15
    
    # Feedback IDs
    FEEDBACK_STATUS = 0x29  # Real-time telemetry
    FEEDBACK_POS_EXT = 0x2A # Optional extended position
    
    def __init__(self, channel="can0", drive_id=104, bitrate=1000000):
        self.drive_id = drive_id
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        print(f"🔌 CubeMars Actuator @ {drive_id} on {channel} @ {bitrate//1000}k")
        
    def _make_eid(self, mode_id: int) -> int:
        """CAN ID = (Control_Mode_ID << 8) | Drive_ID"""
        return (mode_id << 8) | self.drive_id
        
    def _send(self, mode_id: int, data: bytes, repeat: int = 1, delay: float = 0.02):
        """Send Extended CAN frame with optional repetition"""
        msg = can.Message(
            arbitration_id=self._make_eid(mode_id),
            data=data,
            is_extended_id=True,  # Docs: "Extended Frame format"
            dlc=len(data)
        )
        for _ in range(repeat):
            self.bus.send(msg)
            time.sleep(delay)
            
    def set_origin(self, permanent: bool = False):
        """Set current position as origin (Mode 5). Required before absolute moves."""
        mode = 1 if permanent else 0
        self._send(self.MODE_ORIGIN, bytes([mode]))
        time.sleep(0.5)  # Flash write needs time
        print(f"✅ Origin set ({'permanent' if permanent else 'temporary'})")
        
    def set_position_vel_acc(self, pos_deg: float, speed_erpm: int, accel_erpm2: int, repeat: int = 30):
        """
        Move to position with speed/accel control (Mode 6).
        
        Args:
            pos_deg: Target position in degrees (-36000 to 36000)
            speed_erpm: Speed in ERPM (-32768 to 32767) → actual = speed_erpm * 10
            accel_erpm2: Acceleration in ERPM/s² (0 to 32767) → actual = accel_erpm2 * 10
                       ⚠️ MUST BE ≥ 0 per documentation!
            repeat: Frames to send for reliability
        """
        # Validate acceleration (critical per docs)
        if accel_erpm2 < 0:
            print(f"⚠️ Acceleration must be ≥ 0. Clamping {accel_erpm2} → 0")
            accel_erpm2 = 0
            
        # Pack exactly as docs specify: Big-Endian, int32 + int16 + int16
        pos_raw = int(pos_deg * 10000.0)  # Position scale: /10000.0
        payload = struct.pack('>i', pos_raw) + struct.pack('>hh', speed_erpm, accel_erpm2)
        
        # Debug: print exact hex to compare with working bash command
        hex_payload = payload.hex().upper()
        print(f"📤 Mode 6 Payload: {hex_payload} (Target: {pos_deg}°, Speed: {speed_erpm*10} ERPM, Accel: {accel_erpm2*10} ERPM/s²)")
        
        self._send(self.MODE_POS_VEL_ACC, payload, repeat=repeat)
        
    def set_position_simple(self, pos_deg: float, repeat: int = 30):
        """Simpler Position Loop Mode (Mode 4) for testing."""
        pos_raw = int(pos_deg * 10000.0)
        payload = struct.pack('>i', pos_raw)
        print(f"📤 Mode 4 Payload: {payload.hex().upper()} (Target: {pos_deg}°)")
        self._send(self.MODE_POSITION, payload, repeat=repeat)
        
    def read_feedback(self, timeout: float = 0.1) -> dict | None:
        """Read telemetry frame (ID 0x29XX). Matches docs Table: Feedback Messages."""
        feedback_id = self._make_eid(self.FEEDBACK_STATUS)
        self.bus.set_filters([{"can_id": feedback_id, "can_mask": 0x1FFFFFFF, "extended": True}])
        
        msg = self.bus.recv(timeout=timeout)
        if not msg or len(msg.data) < 8:
            return None
            
        d = msg.data
        # Decode exactly per docs formulas:
        pos = ((d[0] << 8) | d[1]) * 0.1      # int16 / 10.0 → degrees
        spd = ((d[2] << 8) | d[3]) * 10.0     # int16 * 10.0 → ERPM
        cur = ((d[4] << 8) | d[5]) * 0.01     # int16 * 0.01 → Amps
        temp = d[6] if d[6] < 128 else d[6] - 256  # int8 signed
        err = d[7]
        
        return {"pos": pos, "spd": spd, "cur": cur, "temp": temp, "err": err, "ts": msg.timestamp}
        
    def disable(self):
        """Safe shutdown: send Mode 15 (Disable)."""
        self._send(self.MODE_DISABLE, b'\x00' * 8)
        print("🛑 Motor disabled")
        
    def close(self):
        self.bus.shutdown()

# ─── MAIN: Oscillation with Proper Protocol ───
def main():
    arm = CubeMarsActuator("can0", drive_id=104)
    
    try:
        # 1. Set origin FIRST (critical for absolute positioning)
        print("🎯 Setting temporary origin at current position...")
        arm.set_origin(permanent=False)
        time.sleep(1.0)
        
        # 2. Verify we can read feedback
        telem = arm.read_feedback()
        if telem:
            print(f"📊 Initial: {telem['pos']:.1f}° | Err: {telem['err']}")
        else:
            print("⚠️ No feedback received. Check wiring/bitrate.")
            
        # 3. Test simple position mode first (Mode 4)
        print("\n🧪 Testing Mode 4 (Position Loop)...")
        arm.set_position_simple(90.0, repeat=20)
        time.sleep(3)
        telem = arm.read_feedback()
        if telem: print(f"📊 After Mode 4: {telem['pos']:.1f}°")
        
        # 4. Now use full Position-Velocity-Accel mode (Mode 6)
        print("\n🔄 Starting oscillation with Mode 6 (Pos+Vel+Acc)...")
        targets = [180.0, 0.0, 180.0]
        
        for i, target in enumerate(targets, 1):
            # Speed: 200 ERPM_raw = 2000 ERPM actual
            # Accel: 200 ERPM2_raw = 2000 ERPM/s² actual (MUST be ≥0)
            print(f"\n[{i}/3] Moving to {target}°...")
            arm.set_position_vel_acc(
                pos_deg=target,
                speed_erpm=200,      # 200 * 10 = 2000 ERPM
                accel_erpm2=200,     # 200 * 10 = 2000 ERPM/s² (positive!)
                repeat=30
            )
            
            # Wait and monitor
            start = time.time()
            while time.time() - start < 5.0:
                telem = arm.read_feedback()
                if telem:
                    status = "⚠️ ERR" if telem['err'] else "✅"
                    print(f"\r📊 {telem['pos']:6.1f}° | {telem['spd']:6.0f} ERPM | {telem['cur']:5.2f}A {status}", end="", flush=True)
                    if abs(telem['pos'] - target) < 2.0:
                        print(f"\n🎯 Reached {target}°!")
                        break
                time.sleep(0.1)
            time.sleep(0.5)  # Brief pause between moves
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
    finally:
        arm.disable()
        arm.close()
        print("🔌 CAN bus closed")

if __name__ == "__main__":
    main()