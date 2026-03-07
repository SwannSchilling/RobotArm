import threading
import time
from scservo_sdk import *
import serial.tools.list_ports
from typing import List, Dict, Optional


class WaveshareServoController:

    ADDR_SCS_TORQUE_ENABLE    = 40
    ADDR_SCS_GOAL_ACC         = 41
    ADDR_SCS_GOAL_POSITION    = 42
    ADDR_SCS_GOAL_SPEED       = 46
    ADDR_SCS_PRESENT_POSITION = 56

    PROTOCOL_END = 0
    BAUDRATE = 1000000
    SCS_MINIMUM_POSITION_VALUE = 100
    SCS_MAXIMUM_POSITION_VALUE = 4000
    SCS_MOVING_SPEED = 0
    SCS_MOVING_ACC   = 0

    WAVESHARE_VID = 0x1A86
    WAVESHARE_PID = 0x55D3

    def __init__(self, servo_ids: List[int], auto_start_thread: bool = True,
                 update_hz: int = 100, angle_range: tuple = (-45, 45),
                 position_range: tuple = None, reduction_ratio: float = 1.0):

        self.servo_ids       = servo_ids
        self.update_hz       = update_hz
        self.reduction_ratio = reduction_ratio
        self.angle_range     = angle_range
        self.min_angle, self.max_angle = angle_range

        self.position_range = position_range if position_range else (500, 3500)
        self.min_position, self.max_position = self.position_range
        self.center_position = (self.min_position + self.max_position) // 2

        self.current_targets = {sid: self.center_position for sid in servo_ids}

        # ✅ FIX 1: One lock that guards ALL bus access
        self._bus_lock = threading.Lock()

        # ✅ FIX 2: Cache last known good positions to survive dropouts
        self._last_known_positions: Dict[int, int] = {
            sid: self.center_position for sid in servo_ids
        }
        self._dropout_counts: Dict[int, int] = {sid: 0 for sid in servo_ids}

        self.portHandler    = None
        self.packetHandler  = None
        self._control_thread  = None
        self._thread_running  = False
        self._thread_lock     = threading.Lock()

        self._initialize_servo()
        self._initialize_sync_reader()  # ✅ FIX 3: Set up bulk sync read

        if auto_start_thread:
            self.start_realtime_control()

    # Add this to the class
    def get_cached_positions(self) -> Dict[int, int]:
        """Return last known good positions without touching the bus.
        Safe to call from any thread at any time."""
        with self._thread_lock:
            return self._last_known_positions.copy()

    def get_cached_angles(self) -> Dict[int, float]:
        positions = self.get_cached_positions()
        return {sid: self.position_to_angle(pos) for sid, pos in positions.items()}

    def _initialize_sync_reader(self):
        """Set up GroupSyncRead for bulk position reads in one bus transaction."""
        self.groupSyncRead = GroupSyncRead(
            self.portHandler,
            self.packetHandler,
            self.ADDR_SCS_PRESENT_POSITION,
            2  # 2 bytes for position
        )
        for sid in self.servo_ids:
            if not self.groupSyncRead.addParam(sid):
                print(f"⚠️ Failed to add servo {sid} to sync read group")

    def _find_device(self, vid, pid):
        for port in serial.tools.list_ports.comports():
            if port.vid == vid and port.pid == pid:
                return port.device
        return None

    def _initialize_servo(self):
        device_name = self._find_device(self.WAVESHARE_VID, self.WAVESHARE_PID)
        if device_name is None:
            raise RuntimeError("❌ Waveshare Servo Adapter not found.")
        print(f"✅ Found Waveshare Servo Adapter on {device_name}")

        self.portHandler   = PortHandler(device_name)
        self.packetHandler = PacketHandler(self.PROTOCOL_END)

        if not self.portHandler.openPort():
            raise RuntimeError("❌ Failed to open port")
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            raise RuntimeError("❌ Failed to set baudrate")

        for servo_id in self.servo_ids:
            result = self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_TORQUE_ENABLE, 1
            )
            if result[0] != COMM_SUCCESS:
                print(f"⚠️ Warning: Failed to enable torque for servo {servo_id}")
        print("✅ Torque enabled for all servos")

    # ------------------------------------------------------------------ #
    #  CORE FIX: All bus operations now go through one locked helper      #
    # ------------------------------------------------------------------ #

    def _send_goal_positions(self, targets: Dict[int, int]):
        """Write goal positions for all servos — called inside bus lock."""
        for servo_id, target in targets.items():
            self.packetHandler.write2ByteTxOnly(
                self.portHandler, servo_id, self.ADDR_SCS_GOAL_POSITION, target
            )

    def _sync_read_positions(self) -> Dict[int, Optional[int]]:
        """
        Bulk-read all servo positions in one transaction — called inside bus lock.
        Falls back to last known good value on dropout and increments counter.
        """
        result = self.groupSyncRead.txRxPacket()
        positions = {}

        for sid in self.servo_ids:
            if result == COMM_SUCCESS and self.groupSyncRead.isAvailable(
                sid, self.ADDR_SCS_PRESENT_POSITION, 2
            ):
                pos = self.groupSyncRead.getData(
                    sid, self.ADDR_SCS_PRESENT_POSITION, 2
                )
                self._last_known_positions[sid] = pos
                positions[sid] = pos
            else:
                # ✅ Graceful degradation: return last known good, log dropout
                self._dropout_counts[sid] += 1
                positions[sid] = self._last_known_positions[sid]

        return positions

    def _realtime_control_loop(self):
        """
        Background thread: write goals then immediately read positions,
        all within a single bus lock acquisition per cycle.
        """
        dt = 1.0 / self.update_hz

        while self._thread_running:
            loop_start = time.time()

            with self._thread_lock:
                targets_copy = self.current_targets.copy()

            # ✅ FIX: One lock, write then read atomically
            with self._bus_lock:
                self._send_goal_positions(targets_copy)
                # Small gap for bus direction switch (TX→RX)
                time.sleep(0.001)
                self._sync_read_positions()

            elapsed = time.time() - loop_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ------------------------------------------------------------------ #
    #  Public read methods — all acquire bus lock                         #
    # ------------------------------------------------------------------ #

    def read_all_positions(self) -> Dict[int, Optional[int]]:
        """Bulk read all positions (one bus transaction, thread-safe)."""
        with self._bus_lock:
            return self._sync_read_positions()

    def read_all_angles(self) -> Dict[int, Optional[float]]:
        positions = self.read_all_positions()
        return {sid: self.position_to_angle(pos) if pos is not None else None
                for sid, pos in positions.items()}

    def read_servo_position(self, servo_id: int) -> Optional[int]:
        with self._bus_lock:
            positions = self._sync_read_positions()
        return positions.get(servo_id)

    def read_servo_angle(self, servo_id: int) -> Optional[float]:
        pos = self.read_servo_position(servo_id)
        return self.position_to_angle(pos) if pos is not None else None

    def get_dropout_stats(self) -> Dict[int, int]:
        """Return dropout counts per servo since startup — useful for diagnostics."""
        return self._dropout_counts.copy()

    # ------------------------------------------------------------------ #
    #  Target setters (unchanged except using shared _thread_lock)        #
    # ------------------------------------------------------------------ #

    def set_target_position(self, servo_id: int, target_position: int):
        if servo_id not in self.servo_ids:
            raise ValueError(f"Servo ID {servo_id} not in configured servo list")
        with self._thread_lock:
            self.current_targets[servo_id] = max(
                self.SCS_MINIMUM_POSITION_VALUE,
                min(self.SCS_MAXIMUM_POSITION_VALUE, target_position)
            )

    def set_target_angle(self, servo_id: int, joint_angle: float):
        self.set_target_position(servo_id, self.angle_to_position(joint_angle))

    def set_multiple_targets(self, targets: Dict[int, int]):
        with self._thread_lock:
            for sid, pos in targets.items():
                if sid in self.servo_ids:
                    self.current_targets[sid] = max(
                        self.SCS_MINIMUM_POSITION_VALUE,
                        min(self.SCS_MAXIMUM_POSITION_VALUE, pos)
                    )

    def set_multiple_target_angles(self, joint_angles: Dict[int, float]):
        self.set_multiple_targets({
            sid: self.angle_to_position(a) for sid, a in joint_angles.items()
            if sid in self.servo_ids
        })

    def set_servo_position(self, servo_id: int, target_position: int,
                           speed: int = None, acc: int = None, blocking: bool = False):
        speed = speed if speed is not None else self.SCS_MOVING_SPEED
        acc   = acc   if acc   is not None else self.SCS_MOVING_ACC
        target_position = max(self.SCS_MINIMUM_POSITION_VALUE,
                              min(self.SCS_MAXIMUM_POSITION_VALUE, target_position))
        with self._bus_lock:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_GOAL_ACC, acc)
            self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_GOAL_SPEED, speed)
            self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_GOAL_POSITION, target_position)

        if blocking:
            while True:
                pos = self.read_servo_position(servo_id)
                if pos is not None and abs(pos - target_position) <= 10:
                    break
                time.sleep(0.01)

    def set_servo_angle(self, servo_id: int, joint_angle: float,
                        speed: int = None, acc: int = None, blocking: bool = False):
        self.set_servo_position(servo_id, self.angle_to_position(joint_angle),
                                speed, acc, blocking)

    # ------------------------------------------------------------------ #
    #  Angle / position conversion (unchanged)                            #
    # ------------------------------------------------------------------ #

    def joint_angle_to_servo_angle(self, joint_angle: float) -> float:
        return joint_angle * self.reduction_ratio

    def servo_angle_to_joint_angle(self, servo_angle: float) -> float:
        return servo_angle / self.reduction_ratio

    def angle_to_position(self, joint_angle: float) -> int:
        joint_angle  = max(self.min_angle, min(self.max_angle, joint_angle))
        servo_angle  = self.joint_angle_to_servo_angle(joint_angle)
        servo_min    = self.min_angle * self.reduction_ratio
        servo_max    = self.max_angle * self.reduction_ratio
        span_angle   = servo_max - servo_min
        span_pos     = self.max_position - self.min_position
        normalized   = (servo_angle - servo_min) / span_angle if span_angle else 0.5
        return int(self.min_position + normalized * span_pos)

    def position_to_angle(self, position: int) -> float:
        position     = max(self.min_position, min(self.max_position, position))
        servo_min    = self.min_angle * self.reduction_ratio
        servo_max    = self.max_angle * self.reduction_ratio
        span_pos     = self.max_position - self.min_position
        span_angle   = servo_max - servo_min
        normalized   = (position - self.min_position) / span_pos if span_pos else 0.5
        servo_angle  = servo_min + normalized * span_angle
        return self.servo_angle_to_joint_angle(servo_angle)

    # ------------------------------------------------------------------ #
    #  Lifecycle                                                           #
    # ------------------------------------------------------------------ #

    def start_realtime_control(self):
        if self._thread_running:
            print("⚠️ Realtime control thread is already running")
            return
        self._thread_running = True
        self._control_thread = threading.Thread(
            target=self._realtime_control_loop, daemon=True)
        self._control_thread.start()
        print("✅ Realtime control thread started")

    def stop_realtime_control(self):
        self._thread_running = False
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=1.0)

    def disable_torque_all(self):
        with self._bus_lock:
            for sid in self.servo_ids:
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, sid, self.ADDR_SCS_TORQUE_ENABLE, 0)
        print("✅ Torque disabled for all servos")

    def enable_torque_all(self):
        with self._bus_lock:
            for sid in self.servo_ids:
                self.packetHandler.write1ByteTxRx(
                    self.portHandler, sid, self.ADDR_SCS_TORQUE_ENABLE, 1)
        print("✅ Torque enabled for all servos")

    def close(self):
        print("🔄 Shutting down servo controller...")
        self.stop_realtime_control()
        self.disable_torque_all()
        if self.portHandler:
            self.portHandler.closePort()
        print("✅ Servo controller closed")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()