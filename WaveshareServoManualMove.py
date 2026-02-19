import serial.tools.list_ports
import time
from WaveshareServoController import WaveshareServoController

# ── Device Finder ─────────────────────────────────────────────────────────────
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# ── Configuration ─────────────────────────────────────────────────────────────
SERVO_IDS = [1, 2, 3]
WAVESHARE_VID = 0x1A86
WAVESHARE_PID = 0x55D3

# ── Main ──────────────────────────────────────────────────────────────────────
def run():
    port = find_device(WAVESHARE_VID, WAVESHARE_PID)
    if not port:
        print("❌ Waveshare Servo Adapter not found.")
        return

    print(f"✅ Found Waveshare Servo Adapter on {port}")

    # 1. Initialize Controller
    # Note: Your class starts the thread immediately in __init__
    controller = WaveshareServoController(
        servo_ids=SERVO_IDS,
        angle_range=(-180, 180),
        reduction_ratio=20.0,
        position_range=(100, 4000)
    )

    print("\n" + "="*50)
    print("🔍 INITIAL STATUS CHECK")
    print("="*50)

    # 2. Read positions and SYNC targets to prevent jerking
    initial_positions = {}
    for sid in SERVO_IDS:
        pos = controller.read_servo_position(sid)
        if pos is not None:
            initial_positions[sid] = pos
            # Update the background thread target to current position
            controller.set_target_position(sid, pos)
            print(f"📍 Servo {sid} is at: {pos} (Target Synced)")
        else:
            print(f"❌ Could not read Servo {sid}. Check power/ID.")

    # 3. Interactive Loop
    print("\n" + "="*50)
    print("READY: Single Servo Control")
    print("Commands: 'ID, Position' (e.g., 1, 2500)")
    print("Type 'exit' to quit.")
    print("="*50)

    try:
        while True:
            user_input = input("\n>> ").strip().lower()
            if user_input == 'exit':
                break

            try:
                if ',' not in user_input:
                    print("⚠️  Format: ID, Position (e.g., 2, 3000)")
                    continue

                sid_str, pos_str = user_input.split(',')
                sid = int(sid_str.strip())
                pos = int(pos_str.strip())

                if sid in SERVO_IDS:
                    # Updates the background thread target
                    controller.set_target_position(sid, pos)
                    print(f"✅ Moving Servo {sid} toward {pos}...")
                else:
                    print(f"⚠️  ID {sid} is not in the active list {SERVO_IDS}")

            except ValueError:
                print("⚠️  Invalid input. Use numbers like: 1, 2000")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        controller.close()
        print("🔌 Controller shut down.")

if __name__ == "__main__":
    run()