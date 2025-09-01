import threading
import time
from scservo_sdk import *   # SCServo SDK
import serial.tools.list_ports
from typing import List, Dict, Optional


class WaveshareServoController:
    """
    A controller class for Waveshare servo motors using SCServo SDK.
    Provides both blocking and non-blocking servo control with threading support.
    Includes angle-based control for intuitive servo positioning.
    """
    
    # Control table addresses
    ADDR_SCS_TORQUE_ENABLE = 40
    ADDR_SCS_GOAL_ACC = 41
    ADDR_SCS_GOAL_POSITION = 42
    ADDR_SCS_GOAL_SPEED = 46
    ADDR_SCS_PRESENT_POSITION = 56
    
    # Protocol / defaults
    PROTOCOL_END = 0
    BAUDRATE = 1000000
    SCS_MINIMUM_POSITION_VALUE = 100
    SCS_MAXIMUM_POSITION_VALUE = 4000
    SCS_MOVING_SPEED = 0   # 0 = max
    SCS_MOVING_ACC = 0     # 0 = max
    
    # Waveshare Servo Adapter USB identifiers
    WAVESHARE_VID = 0x1A86
    WAVESHARE_PID = 0x55D3
    
    def __init__(self, servo_ids: List[int], auto_start_thread: bool = True, update_hz: int = 100,
                 angle_range: tuple = (-45, 45), position_range: tuple = None, 
                 reduction_ratio: float = 1.0):
        """
        Initialize the servo controller.
        
        Args:
            servo_ids: List of servo IDs to control
            auto_start_thread: Whether to automatically start the realtime control thread
            update_hz: Update frequency for the realtime control loop
            angle_range: Tuple of (min_angle, max_angle) in degrees for the OUTPUT joint
            position_range: Tuple of (min_position, max_position). If None, uses safe range
            reduction_ratio: Gear reduction ratio (e.g., 20:1 = 20.0). 
                           Higher values mean more servo movement for same joint angle.
        """
        self.servo_ids = servo_ids
        self.update_hz = update_hz
        self.reduction_ratio = reduction_ratio
        
        # Angle to position mapping
        self.angle_range = angle_range  # This is the OUTPUT joint angle range
        self.min_angle, self.max_angle = angle_range
        
        if position_range is None:
            # Use a safe range within the servo limits (leaving some margin)
            self.position_range = (500, 3500)
        else:
            self.position_range = position_range
        self.min_position, self.max_position = self.position_range
        
        # Calculate center position (corresponds to 0 degrees)
        self.center_position = (self.min_position + self.max_position) // 2
        
        # Initialize with center positions (0 degrees)
        self.current_targets = {servo_id: self.center_position for servo_id in servo_ids}
        
        self.portHandler = None
        self.packetHandler = None
        self._control_thread = None
        self._thread_running = False
        self._thread_lock = threading.Lock()
        
        # Initialize hardware connection
        self._initialize_servo()
        
        # Start realtime control thread if requested
        if auto_start_thread:
            self.start_realtime_control()
    
    def _find_device(self, vid: int, pid: int) -> Optional[str]:
        """Find the device port by USB VID/PID."""
        for port in serial.tools.list_ports.comports():
            if port.vid == vid and port.pid == pid:
                return port.device
        return None
    
    def _initialize_servo(self):
        """Initialize the servo hardware connection."""
        device_name = self._find_device(self.WAVESHARE_VID, self.WAVESHARE_PID)
        if device_name is None:
            raise RuntimeError("❌ Waveshare Servo Adapter not found.")
        
        print(f"✅ Found Waveshare Servo Adapter on {device_name}")
        
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(self.PROTOCOL_END)
        
        if not self.portHandler.openPort():
            raise RuntimeError("❌ Failed to open port")
        
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            raise RuntimeError("❌ Failed to set baudrate")
        
        # Enable torque for all servos
        for servo_id in self.servo_ids:
            result = self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_TORQUE_ENABLE, 1
            )
            if result[0] != COMM_SUCCESS:
                print(f"⚠️ Warning: Failed to enable torque for servo {servo_id}")
        
        print("✅ Torque enabled for all servos")
    
    def joint_angle_to_servo_angle(self, joint_angle: float) -> float:
        """
        Convert joint angle to servo angle accounting for gear reduction.
        
        Args:
            joint_angle: Angle of the output joint in degrees
            
        Returns:
            Required servo angle in degrees
        """
        return joint_angle * self.reduction_ratio
    
    def servo_angle_to_joint_angle(self, servo_angle: float) -> float:
        """
        Convert servo angle to joint angle accounting for gear reduction.
        
        Args:
            servo_angle: Servo angle in degrees
            
        Returns:
            Resulting joint angle in degrees
        """
        return servo_angle / self.reduction_ratio
    
    def angle_to_position(self, joint_angle: float) -> int:
        """
        Convert joint angle in degrees to servo position value.
        Automatically accounts for gear reduction.
        
        Args:
            joint_angle: Joint angle in degrees (within configured angle_range)
            
        Returns:
            Servo position value (clamped to valid range)
        """
        # Clamp joint angle to valid range
        joint_angle = max(self.min_angle, min(self.max_angle, joint_angle))
        
        # Convert joint angle to servo angle using reduction ratio
        servo_angle = self.joint_angle_to_servo_angle(joint_angle)
        
        # Calculate servo angle range based on joint range and reduction
        servo_min_angle = self.min_angle * self.reduction_ratio
        servo_max_angle = self.max_angle * self.reduction_ratio
        
        # Map servo angle to position
        servo_angle_span = servo_max_angle - servo_min_angle
        position_span = self.max_position - self.min_position
        
        # Normalize servo angle to 0 to 1, then map to position range
        if servo_angle_span != 0:
            normalized_angle = (servo_angle - servo_min_angle) / servo_angle_span
        else:
            normalized_angle = 0.5  # Center position if no range
            
        position = self.min_position + (normalized_angle * position_span)
        
        return int(position)
    
    def position_to_angle(self, position: int) -> float:
        """
        Convert servo position value to joint angle in degrees.
        Automatically accounts for gear reduction.
        
        Args:
            position: Servo position value
            
        Returns:
            Joint angle in degrees
        """
        # Clamp position to valid range
        position = max(self.min_position, min(self.max_position, position))
        
        # Calculate servo angle range based on joint range and reduction
        servo_min_angle = self.min_angle * self.reduction_ratio
        servo_max_angle = self.max_angle * self.reduction_ratio
        
        # Map position to servo angle
        position_span = self.max_position - self.min_position
        servo_angle_span = servo_max_angle - servo_min_angle
        
        # Normalize position to 0 to 1, then map to servo angle range
        if position_span != 0:
            normalized_position = (position - self.min_position) / position_span
        else:
            normalized_position = 0.5
            
        servo_angle = servo_min_angle + (normalized_position * servo_angle_span)
        
        # Convert servo angle back to joint angle
        joint_angle = self.servo_angle_to_joint_angle(servo_angle)
        
        return joint_angle
    
    def set_servo_position(self, servo_id: int, target_position: int, 
                          speed: int = None, acc: int = None, blocking: bool = False):
        """
        Set servo position (non-blocking by default).
        
        Args:
            servo_id: ID of the servo to move
            target_position: Target position (100-4000)
            speed: Movement speed (0 = max speed)
            acc: Acceleration (0 = max acceleration)
            blocking: If True, wait for movement to complete
        """
        if servo_id not in self.servo_ids:
            raise ValueError(f"Servo ID {servo_id} not in configured servo list")
        
        speed = speed if speed is not None else self.SCS_MOVING_SPEED
        acc = acc if acc is not None else self.SCS_MOVING_ACC
        
        # Clamp position to valid range
        target_position = max(self.SCS_MINIMUM_POSITION_VALUE,
                             min(self.SCS_MAXIMUM_POSITION_VALUE, target_position))
        
        # Send commands
        self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, self.ADDR_SCS_GOAL_ACC, acc)
        self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.ADDR_SCS_GOAL_SPEED, speed)
        self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, self.ADDR_SCS_GOAL_POSITION, target_position)
        
        if blocking:
            # Wait for servo to reach target (simplified - you might want to add timeout)
            while abs(self.read_servo_position(servo_id) - target_position) > 10:
                time.sleep(0.01)
    
    def set_servo_angle(self, servo_id: int, joint_angle: float, 
                       speed: int = None, acc: int = None, blocking: bool = False):
        """
        Set servo position using joint angle in degrees.
        Automatically accounts for gear reduction.
        
        Args:
            servo_id: ID of the servo to move
            joint_angle: Target joint angle in degrees (within configured angle_range)
            speed: Movement speed (0 = max speed)
            acc: Acceleration (0 = max acceleration)
            blocking: If True, wait for movement to complete
        """
        position = self.angle_to_position(joint_angle)
        self.set_servo_position(servo_id, position, speed, acc, blocking)
    
    def set_target_position(self, servo_id: int, target_position: int):
        """
        Set target position for realtime control thread.
        This updates the target that the background thread will continuously send.
        
        Args:
            servo_id: ID of the servo
            target_position: Target position (100-4000)
        """
        if servo_id not in self.servo_ids:
            raise ValueError(f"Servo ID {servo_id} not in configured servo list")
        
        with self._thread_lock:
            self.current_targets[servo_id] = max(self.SCS_MINIMUM_POSITION_VALUE,
                                               min(self.SCS_MAXIMUM_POSITION_VALUE, target_position))
    
    def set_target_angle(self, servo_id: int, joint_angle: float):
        """
        Set target joint angle for realtime control thread.
        Automatically accounts for gear reduction.
        
        Args:
            servo_id: ID of the servo
            joint_angle: Target joint angle in degrees (within configured angle_range)
        """
        position = self.angle_to_position(joint_angle)
        self.set_target_position(servo_id, position)
    
    def set_multiple_targets(self, targets: Dict[int, int]):
        """
        Set multiple servo targets at once.
        
        Args:
            targets: Dictionary mapping servo_id to target_position
        """
        with self._thread_lock:
            for servo_id, target_position in targets.items():
                if servo_id in self.servo_ids:
                    self.current_targets[servo_id] = max(self.SCS_MINIMUM_POSITION_VALUE,
                                                       min(self.SCS_MAXIMUM_POSITION_VALUE, target_position))
    
    def set_multiple_target_angles(self, joint_angles: Dict[int, float]):
        """
        Set multiple servo target joint angles at once.
        Automatically accounts for gear reduction.
        
        Args:
            joint_angles: Dictionary mapping servo_id to target_joint_angle (degrees)
        """
        positions = {}
        for servo_id, joint_angle in joint_angles.items():
            if servo_id in self.servo_ids:
                positions[servo_id] = self.angle_to_position(joint_angle)
        self.set_multiple_targets(positions)
    
    def read_servo_position(self, servo_id: int) -> Optional[int]:
        """
        Read current servo position.
        
        Args:
            servo_id: ID of the servo to read
            
        Returns:
            Current position or None if read failed
        """
        pos, comm_result, err = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, self.ADDR_SCS_PRESENT_POSITION
        )
        
        if comm_result != COMM_SUCCESS:
            print(f"Communication error reading servo {servo_id}: {self.packetHandler.getTxRxResult(comm_result)}")
            return None
        elif err != 0:
            print(f"Packet error reading servo {servo_id}: {self.packetHandler.getRxPacketError(err)}")
            return None
        
        return pos
    
    def read_servo_angle(self, servo_id: int) -> Optional[float]:
        """
        Read current joint angle in degrees.
        Automatically accounts for gear reduction.
        
        Args:
            servo_id: ID of the servo to read
            
        Returns:
            Current joint angle in degrees or None if read failed
        """
        position = self.read_servo_position(servo_id)
        if position is None:
            return None
        return self.position_to_angle(position)
    
    def read_all_positions(self) -> Dict[int, Optional[int]]:
        """Read positions of all configured servos."""
        positions = {}
        for servo_id in self.servo_ids:
            positions[servo_id] = self.read_servo_position(servo_id)
        return positions
    
    def read_all_angles(self) -> Dict[int, Optional[float]]:
        """Read joint angles of all configured servos in degrees."""
        angles = {}
        for servo_id in self.servo_ids:
            angles[servo_id] = self.read_servo_angle(servo_id)
        return angles
    
    def _realtime_control_loop(self):
        """Background thread for continuous servo control."""
        dt = 1.0 / self.update_hz
        
        while self._thread_running:
            with self._thread_lock:
                targets_copy = self.current_targets.copy()
            
            # Send position commands to all servos in one batch
            for servo_id, target in targets_copy.items():
                # Use write instruction without waiting for response for better performance
                self.packetHandler.write2ByteTxOnly(
                    self.portHandler, servo_id, self.ADDR_SCS_GOAL_POSITION, target
                )
            
            time.sleep(dt)
    
    def start_realtime_control(self):
        """Start the realtime control thread."""
        if self._thread_running:
            print("⚠️ Realtime control thread is already running")
            return
        
        self._thread_running = True
        self._control_thread = threading.Thread(target=self._realtime_control_loop, daemon=True)
        self._control_thread.start()
        print("✅ Realtime control thread started")
    
    def stop_realtime_control(self):
        """Stop the realtime control thread."""
        if not self._thread_running:
            print("⚠️ Realtime control thread is not running")
            return
        
        self._thread_running = False
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=1.0)
        print("✅ Realtime control thread stopped")
    
    def disable_torque_all(self):
        """Disable torque for all servos (makes them freely movable)."""
        for servo_id in self.servo_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_TORQUE_ENABLE, 0
            )
        print("✅ Torque disabled for all servos")
    
    def enable_torque_all(self):
        """Enable torque for all servos."""
        for servo_id in self.servo_ids:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_TORQUE_ENABLE, 1
            )
        print("✅ Torque enabled for all servos")
    
    def close(self):
        """Clean shutdown of the controller."""
        print("🔄 Shutting down servo controller...")
        self.stop_realtime_control()
        self.disable_torque_all()
        if self.portHandler:
            self.portHandler.closePort()
        print("✅ Servo controller closed")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit with cleanup."""
        self.close()


# Example usage and testing
if __name__ == "__main__":
    # Example usage with angles
    servo_ids = [1, 2, 3]
    
    try:
        # Initialize with custom angle range (-45° to +45°), position range, and reduction
        controller = WaveshareServoController(
            servo_ids, 
            auto_start_thread=True, 
            update_hz=100,
            angle_range=(-45, 45),      # Joint angle range (output)
            position_range=(500, 3500), # Safe servo position range
            reduction_ratio=20.0        # 20:1 gear reduction
        )
        
        print("Testing servo movements with angles and gear reduction...")
        print(f"Reduction ratio: {controller.reduction_ratio}:1")
        
        # Method 1: Using joint angle-based realtime control (recommended)
        time.sleep(1)
        controller.set_target_angle(1, -30)    # -30 degrees joint angle
        controller.set_target_angle(2, 0)      # 0 degrees joint angle (center)
        controller.set_target_angle(3, 45)     # +45 degrees joint angle
        
        time.sleep(2)
        
        # Method 2: Set multiple joint angles at once
        controller.set_multiple_target_angles({
            1: 45,    # +45 degrees joint angle
            2: -45,   # -45 degrees joint angle  
            3: 0      # 0 degrees joint angle (center)
        })
        
        time.sleep(2)
        
        # Method 3: Direct joint angle control (blocking)
        controller.set_servo_angle(1, 0, blocking=True)  # Move joint to center
        print("Servo 1 joint reached center position!")
        
        # Read current joint angles
        joint_angles = controller.read_all_angles()
        print(f"Current joint angles: {joint_angles}")
        
        # Show the mapping with gear reduction
        print(f"\nJoint Angle to Servo calculations (reduction ratio {controller.reduction_ratio}:1):")
        for joint_angle in [-45, 0, 45]:
            servo_angle = controller.joint_angle_to_servo_angle(joint_angle)
            position = controller.angle_to_position(joint_angle)
            print(f"Joint: {joint_angle:+3.0f}° → Servo: {servo_angle:+6.1f}° → Position: {position}")
        
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Clean shutdown
        if 'controller' in locals():
            controller.close()