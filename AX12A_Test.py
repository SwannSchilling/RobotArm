import serial
import serial.tools.list_ports
import time

# --- Configuration ---
# Based on your dmesg: idVendor=fff1, idProduct=ff48
ROBOTIS_VID = 0xFFF1
ROBOTIS_PID = 0xFF48
BAUD_RATE = 115200 

def find_device(vid, pid):
    """Locates the serial port based on Vendor and Product IDs."""
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

def main():
    port_path = find_device(ROBOTIS_VID, ROBOTIS_PID)
    
    if not port_path:
        print(f"Error: Device with VID:{hex(ROBOTIS_VID)} PID:{hex(ROBOTIS_PID)} not found.")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f" - {p.device} (VID: {hex(p.vid) if p.vid else 'None'}, PID: {hex(p.pid) if p.pid else 'None'})")
        return

    print(f"Connecting to: {port_path} at {BAUD_RATE} baud...")
    
    try:
        # Initialize serial connection
        ser = serial.Serial(port_path, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino/Pico to reset
        
        print("--- Dynamixel Tester Active ---")
        print("Commands: 0-255 (Position), 'sweep' (Auto-test), 'q' (Quit)")

        while True:
            user_input = input("Enter Command: ").strip().lower()

            if user_input == 'q':
                break
            
            elif user_input == 'sweep':
                print("Running sweep 0 -> 255 -> 0...")
                for val in list(range(0, 256, 10)) + list(range(255, -1, -10)):
                    ser.write(f"{val}\n".encode())
                    time.sleep(0.05)
                print("Sweep complete.")

            elif user_input.isdigit():
                val = int(user_input)
                if 0 <= val <= 255:
                    ser.write(f"{val}\n".encode())
                    # Read feedback from the Arduino Serial.print statements
                    time.sleep(0.1)
                    while ser.in_waiting:
                        print(f"Feedback: {ser.readline().decode().strip()}")
                else:
                    print("Please enter a value between 0 and 255.")
            else:
                print("Invalid input.")

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
