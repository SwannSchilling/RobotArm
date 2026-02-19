"""
Waveshare Servo Connectivity Diagnostic
========================================
Checks if servos are connected and responding WITHOUT moving them.
Uses the Dynamixel-style protocol that Waveshare servo adapters use.

Servo IDs tested: 1, 2, 3 (matching your WaveshareServoController config)
"""

import serial
import serial.tools.list_ports
import time
import struct

# ── Config ────────────────────────────────────────────────────────────────────
SERVO_IDS      = [1, 2, 3]
BAUD_RATE      = 115200
READ_TIMEOUT   = 0.2   # seconds
TARGET_VID_PID = (0x1A86, 0x55D3)   # same VID/PID as in your main script

# Dynamixel / SMS-STS register addresses (Waveshare ST-series servos)
ADDR_PRESENT_POSITION = 56   # 2 bytes, current position
ADDR_PRESENT_VOLTAGE  = 62   # 1 byte,  current voltage (×10 = mV*10 → V)
ADDR_PRESENT_TEMP     = 63   # 1 byte,  temperature in °C

# ── Port finder ───────────────────────────────────────────────────────────────
def find_port(vid, pid):
    for p in serial.tools.list_ports.comports():
        if p.vid == vid and p.pid == pid:
            return p.device
    return None

# ── Waveshare / SMS-STS packet helpers ───────────────────────────────────────
def checksum(data: list[int]) -> int:
    return (~sum(data)) & 0xFF

def build_read_packet(servo_id: int, address: int, length: int) -> bytes:
    """Build an SMS-STS READ_DATA instruction packet."""
    params   = [address, length]
    payload  = [servo_id, 2 + len(params), 0x02] + params   # 0x02 = READ
    cs       = checksum(payload)
    packet   = bytes([0xFF, 0xFF] + payload + [cs])
    return packet

def parse_response(raw: bytes, expected_len: int):
    """
    Parse a Dynamixel-style response packet.
    Returns (error_byte, data_bytes) or raises ValueError on bad packet.
    """
    if len(raw) < 6:
        raise ValueError(f"Response too short: {len(raw)} bytes → {raw.hex()}")
    if raw[0] != 0xFF or raw[1] != 0xFF:
        raise ValueError(f"Bad header: {raw[:2].hex()}")

    servo_id  = raw[2]
    length    = raw[3]
    error     = raw[4]
    data      = raw[5:5 + expected_len]
    cs_recv   = raw[5 + expected_len] if len(raw) > 5 + expected_len else None

    cs_calc = checksum([servo_id, length, error] + list(data))
    if cs_recv is not None and cs_recv != cs_calc:
        raise ValueError(f"Checksum mismatch: got {cs_recv:#04x}, expected {cs_calc:#04x}")

    return error, data

def read_register(ser: serial.Serial, servo_id: int, address: int, length: int):
    """Send a read request and return raw data bytes, or None on failure."""
    ser.reset_input_buffer()
    packet = build_read_packet(servo_id, address, length)
    ser.write(packet)
    time.sleep(READ_TIMEOUT)

    raw = ser.read(ser.in_waiting or 32)
    if not raw:
        return None, "No response"
    try:
        error, data = parse_response(raw, length)
        return data, error
    except ValueError as e:
        return None, str(e)

# ── Decode helpers ────────────────────────────────────────────────────────────
def decode_position(data: bytes) -> int:
    """SMS-STS: little-endian 16-bit position."""
    if len(data) < 2:
        raise ValueError("Not enough bytes for position")
    return struct.unpack('<H', data[:2])[0]

def decode_voltage(data: bytes) -> float:
    return data[0] / 10.0   # register unit is 0.1 V

def decode_temp(data: bytes) -> int:
    return data[0]           # °C

# ── Main diagnostic ───────────────────────────────────────────────────────────
def run_diagnostic():
    print("=" * 60)
    print("  Waveshare Servo Connectivity Diagnostic")
    print("=" * 60)

    # 1. Find port
    port = find_port(*TARGET_VID_PID)
    if not port:
        print(f"\n❌  Waveshare adapter NOT found (VID={TARGET_VID_PID[0]:#06x}, "
              f"PID={TARGET_VID_PID[1]:#06x})")
        print("\n── All detected serial ports ──")
        for p in serial.tools.list_ports.comports():
            print(f"   {p.device:12s}  VID={p.vid}  PID={p.pid}  → {p.description}")
        return
    print(f"\n✅  Adapter found on  {port}")

    # 2. Open port
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=READ_TIMEOUT)
        print(f"✅  Serial port opened ({BAUD_RATE} baud)")
    except serial.SerialException as e:
        print(f"❌  Could not open {port}: {e}")
        return

    time.sleep(0.3)   # let adapter settle

    # 3. Ping / read each servo
    print(f"\n── Checking servo IDs: {SERVO_IDS} ──\n")

    results = {}
    for sid in SERVO_IDS:
        print(f"  Servo ID {sid}:")

        # --- Position ---
        data, err = read_register(ser, sid, ADDR_PRESENT_POSITION, 2)
        if data is None:
            print(f"    ❌  Position read FAILED  → {err}")
            results[sid] = "OFFLINE"
            print()
            continue

        try:
            pos = decode_position(data)
            print(f"    ✅  Present Position : {pos}  (raw bytes: {data.hex()})")
        except Exception as e:
            print(f"    ⚠️  Position decode error: {e}")

        # --- Voltage ---
        data, err = read_register(ser, sid, ADDR_PRESENT_VOLTAGE, 1)
        if data:
            try:
                volts = decode_voltage(data)
                flag = "⚠️ LOW" if volts < 6.0 else ("⚠️ HIGH" if volts > 8.4 else "✅")
                print(f"    {flag}  Voltage          : {volts:.1f} V")
            except Exception as e:
                print(f"    ⚠️  Voltage decode error: {e}")
        else:
            print(f"    ⚠️  Voltage read failed  → {err}")

        # --- Temperature ---
        data, err = read_register(ser, sid, ADDR_PRESENT_TEMP, 1)
        if data:
            try:
                temp = decode_temp(data)
                flag = "🔥 HOT" if temp >= 50 else "✅"
                print(f"    {flag}  Temperature      : {temp} °C")
            except Exception as e:
                print(f"    ⚠️  Temp decode error: {e}")
        else:
            print(f"    ⚠️  Temperature read failed  → {err}")

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

    if all_ok:
        print("\n🎉  All servos responding — safe to proceed.")
    else:
        print("\n⚠️  One or more servos did not respond.")
        print("    Check: power supply, daisy-chain wiring, servo IDs, baud rate.")

    ser.close()
    print("\nPort closed. No movements were commanded.")

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    run_diagnostic()