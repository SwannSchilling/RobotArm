import serial
import time
import serial.tools.list_ports

def find_pico_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x239A and port.pid == 0x80F4:
            return port.device
    return None

def find_storm32_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x0483 and port.pid == 0x5740:
            return port.device
    return None

def find_arduino_nano_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == 0x1A86 and port.pid == 0x7523:
            return port.device
    return None

pico_port = find_pico_port()
storm32_port = find_storm32_port()
nano_port = find_arduino_nano_port()

if pico_port:
    print(f"✅ Found Pico on {pico_port}")
else:
    print("❌ Pico not found.")

if storm32_port:
    print(f"✅ Found Storm32 BGC on {storm32_port}")
else:
    print("❌ Storm32 BGC not found.")

if nano_port:
    print(f"✅ Found Arduino Nano on {nano_port}")
else:
    print("❌ Arduino Nano not found.")


# Replace with your actual serial port
PORT = pico_port       # e.g., 'COM4' on Windows or '/dev/ttyACM0' on Linux/macOS
BAUD = 115200

# Temperature threshold in Celsius
TEMP_THRESHOLD = 30.0

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    time.sleep(2)  # Wait for the Pico to reset
    print("Reading temperature from Pico...\n")
    
    while True:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            continue

        try:
            temp_c, _ = map(float, line.split(","))
            print(f"Temp: {temp_c:.2f} °C", end='')

            if temp_c > TEMP_THRESHOLD:
                print("  🔥 WARNING: Overheat!")
            else:
                print()

        except ValueError:
            print("⚠️ Malformed line:", line)
