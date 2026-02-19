import serial.tools.list_ports
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time

# ── Config ────────────────────────────────────────────────────────────────────
SERVO_IDS                 = [1, 2, 3]
BAUDRATE                  = 1_000_000
PROTOCOL_END              = 0
WAVESHARE_VID             = 0x1A86
WAVESHARE_PID             = 0x55D3

# Memory Addresses (SCS Series)
ADDR_SCS_TORQUE_ENABLE    = 40
ADDR_SCS_GOAL_ACC         = 41
ADDR_SCS_GOAL_POSITION    = 42
ADDR_SCS_GOAL_SPEED       = 46
ADDR_SCS_PRESENT_POSITION = 56

# ── Helpers ───────────────────────────────────────────────────────────────────
def find_device(vid, pid):
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

def write1(ph, port, sid, addr, val):
    cr, err = ph.write1ByteTxRx(port, sid, addr, val)
    return cr == COMM_SUCCESS

def write2(ph, port, sid, addr, val):
    cr, err = ph.write2ByteTxRx(port, sid, addr, val)
    return cr == COMM_SUCCESS

# ── Main Logic ────────────────────────────────────────────────────────────────
def run():
    print("=" * 60)
    print("  Waveshare Servo Control: Safe Enable + Manual Input")
    print("=" * 60)

    device_name = find_device(WAVESHARE_VID, WAVESHARE_PID)
    if not device_name:
        print("❌  Adapter not found. Check USB connection.")
        return

    portHandler   = PortHandler(device_name)
    packetHandler = PacketHandler(PROTOCOL_END)

    if not (portHandler.openPort() and portHandler.setBaudRate(BAUDRATE)):
        print("❌  Failed to initialize Serial Port.")
        return

    print(f"✅  Connected on {device_name}\n")

    # --- STEP 1: SAFE INITIALIZATION ---
    for sid in SERVO_IDS:
        print(f"Initializing Servo {sid}...")
        
        # Read current pos so we don't jump
        pos, cr, err = packetHandler.read2ByteTxRx(portHandler, sid, ADDR_SCS_PRESENT_POSITION)
        if cr != COMM_SUCCESS:
            print(f"  ❌ Failed to communicate with Servo {sid}")
            continue

        # Sync goal to current position
        write2(packetHandler, portHandler, sid, ADDR_SCS_GOAL_POSITION, pos)
        # Set gentle movement parameters
        write1(packetHandler, portHandler, sid, ADDR_SCS_GOAL_ACC, 15)
        write2(packetHandler, portHandler, sid, ADDR_SCS_GOAL_SPEED, 200)
        # Enable Torque
        write1(packetHandler, portHandler, sid, ADDR_SCS_TORQUE_ENABLE, 1)
        
        print(f"  ✅ Holding at {pos}. Torque ON.")

    # --- STEP 2: INTERACTIVE INPUT LOOP ---
    print("\n" + "=" * 60)
    print("READY: Enter positions for servos " + str(SERVO_IDS))
    print("Format: pos1, pos2, pos3 (e.g., 2048, 2048, 2048)")
    print("Type 'exit' to quit.")
    print("=" * 60)

    try:
        while True:
            user_input = input("\n>> Enter Positions: ").strip().lower()
            
            if user_input == 'exit':
                break
            
            try:
                # Split input by commas and convert to integers
                parts = [int(p.strip()) for p in user_input.split(',')]
                
                if len(parts) != len(SERVO_IDS):
                    print(f"⚠️ Error: Expected {len(SERVO_IDS)} values, got {len(parts)}.")
                    continue

                # Write to each servo
                for i, sid in enumerate(SERVO_IDS):
                    target = parts[i]
                    # Clamp values for safety (0-4095 for 12-bit servos)
                    target = max(0, min(4095, target))
                    write2(packetHandler, portHandler, sid, ADDR_SCS_GOAL_POSITION, target)
                    print(f"  Moving ID {sid} -> {target}")

            except ValueError:
                print("⚠️ Invalid input. Please enter numbers separated by commas.")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Optional: Disable torque on exit for safety
        for sid in SERVO_IDS:
            write1(packetHandler, portHandler, sid, ADDR_SCS_TORQUE_ENABLE, 0)
        portHandler.closePort()
        print("Port closed. Torque disabled.")

if __name__ == "__main__":
    run()