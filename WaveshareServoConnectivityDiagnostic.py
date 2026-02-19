"""
Waveshare Servo Connectivity Diagnostic v3
==========================================
Uses scservo_sdk + 1000000 baud — identical stack to WaveshareServoController.
No movement commands issued. Safe to run on a live robot arm.
"""

import time
import serial.tools.list_ports

# ── scservo_sdk imports (same as WaveshareServoController) ───────────────────
try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("❌  scservo_sdk not found.")
    print("    Install it with:  pip install scservo_sdk  (or pip3 install scservo_sdk)")
    raise SystemExit(1)

# ── Config — copied verbatim from WaveshareServoController ───────────────────
SERVO_IDS               = [1, 2, 3]
BAUDRATE                = 1_000_000      # WaveshareServoController.BAUDRATE
PROTOCOL_END            = 0             # WaveshareServoController.PROTOCOL_END
WAVESHARE_VID           = 0x1A86
WAVESHARE_PID           = 0x55D3

ADDR_SCS_TORQUE_ENABLE  = 40
ADDR_SCS_PRESENT_POSITION = 56

# ── Port finder ───────────────────────────────────────────────────────────────
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# ── Main diagnostic ───────────────────────────────────────────────────────────
def run():
    print("=" * 60)
    print("  Waveshare Servo Diagnostic  v3  (scservo_sdk / 1Mbaud)")
    print("=" * 60)

    # 1. Find adapter
    device_name = find_device(WAVESHARE_VID, WAVESHARE_PID)
    if device_name is None:
        print("\n❌  Waveshare adapter not found. Detected ports:")
        for p in serial.tools.list_ports.comports():
            print(f"   {p.device:12s}  VID={p.vid}  PID={p.pid}  {p.description}")
        return

    print(f"\n✅  Adapter found on {device_name}")

    # 2. Open port (same calls as _initialize_servo)
    portHandler   = PortHandler(device_name)
    packetHandler = PacketHandler(PROTOCOL_END)

    if not portHandler.openPort():
        print("❌  Failed to open port (is another process using it?)")
        return
    print("✅  Port opened")

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌  Failed to set baud rate to {BAUDRATE}")
        portHandler.closePort()
        return
    print(f"✅  Baud rate set to {BAUDRATE}")

    # 3. Per-servo checks
    print(f"\n── Checking servo IDs: {SERVO_IDS} ──\n")
    results = {}

    for sid in SERVO_IDS:
        print(f"  Servo ID {sid}:")

        # ── Read present position ─────────────────────────────────────────
        pos, comm_result, err = packetHandler.read2ByteTxRx(
            portHandler, sid, ADDR_SCS_PRESENT_POSITION
        )

        if comm_result != COMM_SUCCESS:
            reason = packetHandler.getTxRxResult(comm_result)
            print(f"    ❌  Position read FAILED  →  comm_result: {reason}")
            print(f"        (comm_result code = {comm_result})")
            results[sid] = "OFFLINE"
            print()
            continue

        if err != 0:
            reason = packetHandler.getRxPacketError(err)
            print(f"    ⚠️  Position read OK but packet error  →  {reason}")
        else:
            print(f"    ✅  Present Position : {pos}")

        # ── Try reading torque enable register (1 byte) ───────────────────
        torque, comm_result2, err2 = packetHandler.read1ByteTxRx(
            portHandler, sid, ADDR_SCS_TORQUE_ENABLE
        )
        if comm_result2 == COMM_SUCCESS:
            torque_str = "ENABLED" if torque else "DISABLED"
            print(f"    ✅  Torque state      : {torque_str} ({torque})")
        else:
            print(f"    ⚠️  Torque read failed: {packetHandler.getTxRxResult(comm_result2)}")

        results[sid] = "ONLINE"
        print()

    # 4. Summary
    print("── Summary ──")
    all_ok = True
    for sid in SERVO_IDS:
        status = results.get(sid, "NOT TESTED")
        icon   = "✅" if status == "ONLINE" else "❌"
        print(f"  Servo {sid}: {icon} {status}")
        if status != "ONLINE":
            all_ok = False

    print()
    if all_ok:
        print("🎉  All servos responding — safe to proceed.\n")
    else:
        print("⚠️  One or more servos offline. Troubleshooting guide:\n")
        print("  comm_result codes from scservo_sdk:")
        print("   -1001  COMM_PORT_BUSY   → Another process has the port open")
        print("                             (kill your main script first)")
        print("   -1002  COMM_TX_FAIL     → Packet could not be sent")
        print("   -3001  COMM_RX_TIMEOUT  → Servo didn't reply in time")
        print("                             (wrong baud rate, wiring, power)")
        print("   -3002  COMM_RX_CORRUPT  → Response garbled (loose TTL wire)")
        print()
        print("  Hardware checklist:")
        print("   1. 12V must be on the SERVO power header, not just ODrive bus")
        print("      — servo LED should flash briefly on power-up")
        print("   2. Data wire: adapter TTL pin → servo chain, shared GND")
        print("   3. Only ONE process can own the serial port at a time")
        print("   4. If daisy-chained: servo 1 OK but 2+3 fail → broken cable")
        print("      between servo 1 and 2")

    portHandler.closePort()
    print("\nPort closed. No movements were commanded.")

if __name__ == "__main__":
    run()