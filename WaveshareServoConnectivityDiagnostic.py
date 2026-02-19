"""
Waveshare Servo - Safe Torque Enable (fixed)
=============================================
scservo_sdk write functions return (comm_result, error)      ← 2 values
scservo_sdk read  functions return (data, comm_result, error) ← 3 values

Steps per servo:
  1. Read current position
  2. Set goal = current position  ← hold in place, no sudden snap
  3. Set gentle speed + accel
  4. Enable torque
  5. Confirm torque register reads back 1
"""

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

# ── Write helpers (2-value return) ───────────────────────────────────────────
def write1(ph, port, sid, addr, val, label=""):
    cr, err = ph.write1ByteTxRx(port, sid, addr, val)
    if cr != COMM_SUCCESS:
        print(f"  ❌  {label} write failed: {ph.getTxRxResult(cr)}")
        return False
    return True

def write2(ph, port, sid, addr, val, label=""):
    cr, err = ph.write2ByteTxRx(port, sid, addr, val)
    if cr != COMM_SUCCESS:
        print(f"  ❌  {label} write failed: {ph.getTxRxResult(cr)}")
        return False
    return True

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

        # Step 2: Set goal = current (hold in place before torque-on)
        if not write2(packetHandler, portHandler, sid, ADDR_SCS_GOAL_POSITION, pos, "Goal position"):
            all_ok = False
            print()
            continue
        print(f"  ✅  Goal position set to {pos} (hold in place)")

        # Step 3: Gentle speed/accel so any tiny correction is smooth
        write1(packetHandler, portHandler, sid, ADDR_SCS_GOAL_ACC,    10,  "Acceleration")
        write2(packetHandler, portHandler, sid, ADDR_SCS_GOAL_SPEED,  100, "Speed")

        # Step 4: Enable torque
        if not write1(packetHandler, portHandler, sid, ADDR_SCS_TORQUE_ENABLE, 1, "Torque enable"):
            all_ok = False
            print()
            continue

        # Step 5: Confirm torque is on
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

if __name__ == "__main__":
    run()