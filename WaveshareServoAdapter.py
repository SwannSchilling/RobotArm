import time
from scservo_sdk import *
import serial
import serial.tools.list_ports

# Control table addresses (matching working example)
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56

# Protocol version (KEY DIFFERENCE: Using 0 instead of 1.0)
PROTOCOL_END               = 0                 # SCServo bit end(STS/SMS=0, SCS=1)

# Default setting
SCS_ID                     = 1                 # Servo ID
BAUDRATE                   = 1000000           # Default baudrate
DEVICENAME                 = 'COM42'           # Your COM port

# Position limits
SCS_MINIMUM_POSITION_VALUE = 100               # Minimum position
SCS_MAXIMUM_POSITION_VALUE = 4000              # Maximum position
SCS_MOVING_STATUS_THRESHOLD = 20               # Movement threshold
SCS_MOVING_SPEED           = 0                 # Moving speed (0 = max speed)
SCS_MOVING_ACC             = 0                 # Moving acceleration

def find_device(vid, pid):
    """Find the COM port of a USB device by VID and PID"""
    for port in serial.tools.list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port.device
    return None

def initialize_servo():
    """Initialize the servo communication"""
    # Find Waveshare Servo Adapter automatically
    DEVICENAME = find_device(0x1A86, 0x55D3)   # VID=1A86, PID=55D3
    if DEVICENAME is None:
        print("❌ Waveshare Servo Adapter not found.")
        quit()
    else:
        print(f"✅ Found Waveshare Servo Adapter on {DEVICENAME}")

    # Initialize PortHandler instance
    portHandler = PortHandler(DEVICENAME)
    
    # Initialize PacketHandler instance
    packetHandler = PacketHandler(PROTOCOL_END)
    
    # Open port
    if portHandler.openPort():
        print("✅ Succeeded to open the port")
    else:
        print("❌ Failed to open the port")
        quit()
    
    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print(f"✅ Baudrate set to {BAUDRATE}")
    else:
        print("❌ Failed to change the baudrate")
        quit()
    
    # Write SCServo acceleration
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
        portHandler, SCS_ID, ADDR_SCS_GOAL_ACC, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # Write SCServo speed
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(
        portHandler, SCS_ID, ADDR_SCS_GOAL_SPEED, SCS_MOVING_SPEED)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    
    print("Servo initialized successfully")
    return portHandler, packetHandler

def move_to_position(portHandler, packetHandler, target_position):
    """Move servo to target position"""
    # Write goal position
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(
        portHandler, SCS_ID, ADDR_SCS_GOAL_POSITION, target_position)
    
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

def wait_for_movement_complete(portHandler, packetHandler, target_position):
    """Wait until servo reaches target position (using the working example method)"""
    while True:
        # Read present position and speed (KEY FIX: Using read4ByteTxRx)
        scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(
            portHandler, SCS_ID, ADDR_SCS_PRESENT_POSITION)
        
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
            continue
        elif scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))
            continue
        
        # Extract position from the combined value (KEY FIX)
        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        scs_present_speed = SCS_HIWORD(scs_present_position_speed)
        
        print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
              % (SCS_ID, target_position, scs_present_position, SCS_TOHOST(scs_present_speed, 15)))
        
        # Check if reached target position (using the working example logic)
        if not (abs(target_position - scs_present_position) > SCS_MOVING_STATUS_THRESHOLD):
            break
        
        time.sleep(0.1)  # Small delay

def sweep_min_max():
    """Main function to sweep between min and max positions"""
    try:
        # Initialize servo
        portHandler, packetHandler = initialize_servo()
        
        print("Starting position sweep...")
        print(f"Sweeping between {SCS_MINIMUM_POSITION_VALUE} and {SCS_MAXIMUM_POSITION_VALUE}")
        
        scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]
        index = 0
        sweep_count = 0
        max_sweeps = 10  # Number of position changes
        
        while sweep_count < max_sweeps:
            print(f"\n--- Move {sweep_count + 1} ---")
            
            # Move to current goal position
            print(f"Moving to position: {scs_goal_position[index]}")
            move_to_position(portHandler, packetHandler, scs_goal_position[index])
            wait_for_movement_complete(portHandler, packetHandler, scs_goal_position[index])
            print("Position reached!")
            
            time.sleep(1)  # Pause at position
            
            # Change goal position (alternate between min and max)
            if index == 0:
                index = 1
            else:
                index = 0
            
            sweep_count += 1
        
        # Move to center position at the end
        center_position = (SCS_MINIMUM_POSITION_VALUE + SCS_MAXIMUM_POSITION_VALUE) // 2
        print(f"\nReturning to center position: {center_position}")
        move_to_position(portHandler, packetHandler, center_position)
        wait_for_movement_complete(portHandler, packetHandler, center_position)
        
        # Disable torque
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
            portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
        print("Torque disabled")
        
        # Close port
        portHandler.closePort()
        print("Port closed")
        
    except KeyboardInterrupt:
        print("\nSweep interrupted by user")
        # Disable torque and close port safely
        try:
            packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
            portHandler.closePort()
        except KeyboardInterrupt:
            print("\nScan interrupted")
            try:
                portHandler.closePort()
            except:
                pass

def continuous_sweep():
    """Alternative function for continuous sweeping until interrupted"""
    try:
        portHandler, packetHandler = initialize_servo()
        
        print("Starting continuous sweep (Press Ctrl+C to stop)...")
        print(f"Sweeping between {SCS_MINIMUM_POSITION_VALUE} and {SCS_MAXIMUM_POSITION_VALUE}")
        
        scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]
        index = 0
        
        while True:
            print(f"Moving to position: {scs_goal_position[index]}")
            move_to_position(portHandler, packetHandler, scs_goal_position[index])
            wait_for_movement_complete(portHandler, packetHandler, scs_goal_position[index])
            
            time.sleep(0.5)  # Pause at position
            
            # Alternate between positions
            if index == 0:
                index = 1
            else:
                index = 0
            
    except KeyboardInterrupt:
        print("\nContinuous sweep stopped")
        try:
            scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
                portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
            portHandler.closePort()
        except:
            pass

def manual_control():
    """Manual control like the original example"""
    try:
        portHandler, packetHandler = initialize_servo()
        
        scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]
        index = 0
        
        print("Manual control mode - Press any key to move, ESC to quit")
        
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            key = input("Press Enter to move (or 'q' to quit): ")
            if key.lower() == 'q':
                break
            
            print(f"Moving to position: {scs_goal_position[index]}")
            move_to_position(portHandler, packetHandler, scs_goal_position[index])
            wait_for_movement_complete(portHandler, packetHandler, scs_goal_position[index])
            
            # Change goal position
            if index == 0:
                index = 1
            else:
                index = 0
        
        # Disable torque
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
            portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
        portHandler.closePort()
        
    except KeyboardInterrupt:
        print("\nManual control stopped")
        try:
            packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_SCS_TORQUE_ENABLE, 0)
            portHandler.closePort()
        except:
            pass

def scan_servos():
    """Scan for servos on the bus"""
    try:
        print("=== Servo Scanner ===")
        print("Scanning for servos (this may take a moment)...")
        
        portHandler = PortHandler(DEVICENAME)
        packetHandler = PacketHandler(PROTOCOL_END)
        
        if not portHandler.openPort():
            print("Failed to open the port")
            return
            
        if not portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the baudrate")
            portHandler.closePort()
            return
        
        found_servos = []
        ADDR_SCS_ID = 5
        
        print("Scanning IDs 1-10 (extend range if needed)...")
        for test_id in range(1, 11):  # Scan IDs 1-10
            # Try to read ID register
            read_id, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(
                portHandler, test_id, ADDR_SCS_ID)
            
            if scs_comm_result == COMM_SUCCESS:
                print(f"Found servo at ID: {test_id}")
                found_servos.append(test_id)
            else:
                print(f"ID {test_id}: No response", end="\r")
        
        print()  # New line after scanning
        
        if found_servos:
            print(f"\nFound {len(found_servos)} servo(s) at ID(s): {found_servos}")
            
            # Ask user if they want to use one of the found servos
            if len(found_servos) == 1:
                choice = input(f"Use servo ID {found_servos[0]}? (y/n): ")
                if choice.lower() == 'y':
                    global SCS_ID
                    SCS_ID = found_servos[0]
                    print(f"SCS_ID updated to: {SCS_ID}")
            else:
                try:
                    choice = int(input(f"Select servo ID to use {found_servos}: "))
                    if choice in found_servos:
                        SCS_ID = choice
                        print(f"SCS_ID updated to: {SCS_ID}")
                    else:
                        print("Invalid selection")
                except ValueError:
                    print("Invalid input")
        else:
            print("No servos found. Check connections and try different baudrate if needed.")
        
        portHandler.closePort()
        
    except KeyboardInterrupt:
        print("\nScan interrupted")
        try:
            portHandler.closePort()
        except:
            pass

if __name__ == "__main__":
    print("Feetech STS3215-12V Servo Control Suite")
    print(f"Current servo ID: {SCS_ID}")
    print(f"COM Port: {DEVICENAME}")
    print("-" * 40)
    print("1. Fixed number of sweeps")
    print("2. Continuous sweep")
    print("3. Manual control (like original example)")
    print("4. Scan for servos")
    
    choice = input("Select option (1-4): ")
    
    if choice == "1":
        sweep_min_max()
    elif choice == "2":
        continuous_sweep()
    elif choice == "3":
        manual_control()
    elif choice == "4":
        scan_servos()
    else:
        print("Invalid choice")