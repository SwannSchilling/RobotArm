#!/usr/bin/env python3
import can
import struct
import time
import sys

class RobotArm:
    def __init__(self, channel="can0", controller_id=104):
        self.controller_id = controller_id
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.telem_id = 0x2968  # Telemetry ID from your Arduino
        
    def _make_eid(self, cmd_id):
        return self.controller_id | (cmd_id << 8)
        
    def _send(self, cmd_id, data):
        msg = can.Message(
            arbitration_id=self._make_eid(cmd_id),
            data=data,
            is_extended_id=True,
            dlc=len(data)
        )
        self.bus.send(msg)
        time.sleep(0.02)

    def enable_motor(self):
        """Clear faults & wake controller"""
        self._send(1, struct.pack('>i', 0))  # CAN_PACKET_SET_CURRENT
        time.sleep(0.1)

    def set_position_spd(self, pos_deg, spd_raw=200, rpa_raw=200, repeat=15):
        """
        Move to position with speed/accel control.
        spd_raw / 10.0 = actual °/s (matches Arduino firmware)
        """
        pos_raw = int(pos_deg * 10000.0)
        payload = struct.pack('>i', pos_raw) + struct.pack('>hh', spd_raw, rpa_raw)
        for _ in range(repeat):
            self._send(6, payload)  # CAN_PACKET_SET_POS_SPD

    def read_telemetry(self, timeout=0.05):
        self.bus.set_filters([{"can_id": self.telem_id, "can_mask": 0x1FFFFFFF, "extended": True}])
        msg = self.bus.recv(timeout=timeout)
        if not msg or len(msg.data) < 8:
            return None
        d = msg.data
        return {
            "position": ((d[0] << 8) | d[1]) * 0.1,
            "speed":    ((d[2] << 8) | d[3]) * 10.0,
            "current":  ((d[4] << 8) | d[5]) * 0.01,
            "temp":     d[6],
            "error":    d[7]
        }

    def close(self):
        self.bus.shutdown()

# ─── OSCILLATION SCRIPT ───
def main():
    arm = RobotArm("can0")
    targets = [180.0, 0.0]  # Oscillation pattern
    cycles = 3              # How many full back-and-forth cycles
    tolerance = 2.0         # Degrees within target to consider "reached"
    spd_raw = 250           # 250/10 = 25°/s (smooth, controlled)
    rpa_raw = 250           # Acceleration ramp (match spd for trapezoidal profile)

    print("🔌 Initializing CAN bus...")
    try:
        arm.enable_motor()
        time.sleep(0.5)
        print("✅ Motor enabled. Starting oscillation...\n")

        for cycle in range(1, cycles + 1):
            for target in targets:
                print(f"🔄 Cycle {cycle}/{cycles} → Moving to {target}°...")
                arm.set_position_spd(target, spd_raw=spd_raw, rpa_raw=rpa_raw, repeat=20)

                # Poll until target reached or timeout
                start = time.time()
                timeout = 10.0  # Max seconds per move
                while time.time() - start < timeout:
                    telem = arm.read_telemetry()
                    if telem:
                        err_flag = "⚠️ ERR" if telem['error'] else "✅"
                        # Carriage return for clean live updates
                        print(f"\r📊 Pos: {telem['position']:5.1f}° | Target: {target:5.1f}° | "
                              f"Spd: {telem['speed']:5.0f} | Cur: {telem['current']:4.2f}A {err_flag}", 
                              end="", flush=True)
                        
                        if abs(telem['position'] - target) < tolerance:
                            print(f"\n🎯 Reached {target}°! Resting...")
                            time.sleep(0.5)
                            break
                    time.sleep(0.05)
                else:
                    print(f"\n⏱ Timeout for {target}° (stopped at {telem['position'] if telem else 'N/A'}°)")
                    
            print("-" * 50)

        print("🏁 Oscillation complete.")

    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user.")
    finally:
        arm.close()
        print("🔌 CAN bus closed.")

if __name__ == "__main__":
    main()