"""
Waveshare Servo - Safe Torque Enable
=====================================
1. Reads each servo's CURRENT position
2. Sets that position as the GOAL position  ← prevents sudden movement
3. THEN enables torque                       ← arm holds where it already is
4. Confirms torque is active
No movement commanded.
"""

import time
import serial.tools.list_ports
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ── Config ────────────────────────────────────────────────────────────────────
SERVO_IDS                 = [1, 2, 3]
BAUDRATE                  = 1_000_000
PROTOCOL_END              = 0
WAVESHARE_VID             = 0x1A86
WAVESHARE_PID             = 0x55D3

ADDR_SCS_TORQUE_ENABLE    = 40
ADDR_SCS_GOAL_ACC         = 41
ADDR_SCS_GOAL_POSITION    = 42
ADDR_SCS_GOAL_SPEED       = 46
ADDR_SCS_PRESENT_POSITION = 56

# ── Port finder ───────────────────────────────────────────────────────────────
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# ── Main ──────────────────────────────────────────────────────────────────────
def run():
    print("=" * 60)
    print("  Safe Torque Enable")
    print("=" * 60)

    device_name = find_device(WAVESHARE_VID, WAVESHARE_PID)
    if not device_name:
        print("❌  Adapter not found.")
        return

    portHandler   = PortHandler(device_name)
    packetHandler = PacketHandler(PROTOCOL_END)

    if not portHandler.openPort():
        print("❌  Failed to open port.")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        print("❌  Failed to set baud rate.")
        portHandler.closePort()
        return

    print(f"✅  Connected on {device_name} @ {BAUDRATE} baud\n")

    all_ok = True

    for sid in SERVO_IDS:
        print(f"── Servo ID {sid} ──")

        # Step 1: Read current position
        pos, comm_result, err = packetHandler.read2ByteTxRx(
            portHandler, sid, ADDR_SCS_PRESENT_POSITION
        )
        if comm_result != COMM_SUCCESS:
            print(f"  ❌  Could not read position: {packetHandler.getTxRxResult(comm_result)}")
            all_ok = False
            print()
            continue
        print(f"  📍 Current position : {pos}")

        # Step 2: Write goal position = current position (so torque-on is a hold, not a snap)
        _, cr, _ = packetHandler.write2ByteTxRx(
            portHandler, sid, ADDR_SCS_GOAL_POSITION, pos
        )
        if cr != COMM_SUCCESS:
            print(f"  ⚠️  Goal position write failed: {packetHandler.getTxRxResult(cr)}")
        else:
            print(f"  ✅  Goal position set to {pos} (hold in place)")

        # Step 3: Set gentle speed and acceleration so if there's any tiny
        #         difference between read and actual, it moves slowly
        packetHandler.write1ByteTxRx(portHandler, sid, ADDR_SCS_GOAL_ACC, 10)    # gentle accel
        packetHandler.write2ByteTxRx(portHandler, sid, ADDR_SCS_GOAL_SPEED, 100) # gentle speed

        # Step 4: Enable torque
        _, cr, _ = packetHandler.write1ByteTxRx(
            portHandler, sid, ADDR_SCS_TORQUE_ENABLE, 1
        )
        if cr != COMM_SUCCESS:
            print(f"  ❌  Torque enable failed: {packetHandler.getTxRxResult(cr)}")
            all_ok = False
            print()
            continue

        # Step 5: Confirm torque is now on
        torque_val, cr2, _ = packetHandler.read1ByteTxRx(
            portHandler, sid, ADDR_SCS_TORQUE_ENABLE
        )
        if cr2 == COMM_SUCCESS:
            if torque_val == 1:
                print(f"  ✅  Torque ENABLED and confirmed ✓")
            else:
                print(f"  ⚠️  Torque register reads {torque_val} — unexpected")
        else:
            print(f"  ⚠️  Could not confirm torque state")

        print()

    # ── Summary ───────────────────────────────────────────────────────────────
    print("=" * 60)
    if all_ok:
        print("🎉  All servos holding position with torque enabled.")
        print("    The arm should feel rigid. You can now:")
        print("    • Run your main script (WaveshareServoController)")
        print("    • Send position commands safely")
    else:
        print("⚠️  One or more servos had issues — check output above.")

    portHandler.closePort()
    print("\nPort closed.")

if __name__ == "__main__":
    run()