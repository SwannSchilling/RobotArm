import serial.tools.list_ports
from WaveshareServoController import WaveshareServoController

# ── Device Finder ─────────────────────────────────────────────────────────────
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

# ── Initialization ────────────────────────────────────────────────────────────
servo_ids = [1, 2, 3]
waveshare_servo_port = find_device(0x1A86, 0x55D3)

if not waveshare_servo_port:
    print("❌ Waveshare Servo Adapter not found. Please check connections.")
    exit()

print(f"✅ Found Waveshare Servo Adapter on {waveshare_servo_port}")

# Initialize the controller with your specific reduction and range
controller = WaveshareServoController(
    servo_ids=servo_ids,
    angle_range=(-180, 180),    
    reduction_ratio=20.0,
    position_range=(100, 4000)
)

# ── Step 1: Read Positions ────────────────────────────────────────────────────
print("\n" + "="*50)
print("🔍 INITIAL STATUS CHECK")
print("="*50)

for sid in servo_ids:
    # We use get_servo_position (standard for this wrapper)
    # If your specific class uses a different name, swap it here.
    pos = controller.get_servo_position(sid)
    print(f"📍 Servo {sid} is currently at: {pos}")

# ── Step 2: Interactive Single-Servo Loop ─────────────────────────────────────
print("\n" + "="*50)
print("READY: Single Servo Control")
print("Format: ID, Position  (e.g., 1, 2048)")
print("Type 'exit' to quit.")
print("="*50)

try:
    while True:
        user_input = input("\n>> ").strip().lower()

        if user_input == 'exit':
            break

        try:
            # Parse input as: ID, Position
            parts = [p.strip() for p in user_input.split(',')]
            
            if len(parts) != 2:
                print("⚠️  Invalid format. Please enter 'ID, Position'")
                continue

            sid = int(parts[0])
            pos = int(parts[1])

            if sid in servo_ids:
                # Use Method 1: Target position via background thread
                controller.set_target_position(sid, pos)
                print(f"✅ Command sent: Moving Servo {sid} -> {pos}")
            else:
                print(f"⚠️  Servo ID {sid} is not in your active list {servo_ids}")

        except ValueError:
            print("⚠️  Error: Please enter numbers only (e.g., 2, 3000)")

except KeyboardInterrupt:
    print("\nInterrupt received. Closing...")

finally:
    # Method 1: Clean shutdown
    controller.close()
    print("🔌 Connection closed. Torque released.")