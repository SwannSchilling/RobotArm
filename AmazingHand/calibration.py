import time
import serial

class ServoCalibrator:
    def __init__(self, serial_port="COM43", baudrate=115200, timeout=0.5):
        self.serial = serial.Serial(serial_port, baudrate, timeout=timeout)
        time.sleep(2)
        self.num_servos = 8
        
        # Calibration data structure
        self.middle = [90] * self.num_servos
        self.min_pos = [0] * self.num_servos
        self.max_pos = [180] * self.num_servos
        self.invert = [False] * self.num_servos

    def write_servo(self, servo_id, angle):
        angle = int(max(0, min(180, angle)))
        cmd = f"SERVO,{servo_id},{angle}\n"
        self.serial.write(cmd.encode())
        time.sleep(0.05)

    def calibrate_servo(self, servo_id):
        print(f"\n--- Calibrating Servo {servo_id} ---")
        print("Commands:")
        print("  [a] decrease angle   [d] increase angle")
        print("  [m] set middle       [n] set min")
        print("  [x] set max          [i] toggle invert")
        print("  [s] save & continue  [q] quit")

        angle = 90
        self.write_servo(servo_id, angle)

        while True:
            cmd = input(">> ").strip().lower()
            if cmd == "a":
                angle -= 5
                self.write_servo(servo_id, angle)
            elif cmd == "d":
                angle += 5
                self.write_servo(servo_id, angle)
            elif cmd == "m":
                self.middle[servo_id] = angle
                print(f"Set middle = {angle}")
            elif cmd == "n":
                self.min_pos[servo_id] = angle
                print(f"Set min = {angle}")
            elif cmd == "x":
                self.max_pos[servo_id] = angle
                print(f"Set max = {angle}")
            elif cmd == "i":
                self.invert[servo_id] = not self.invert[servo_id]
                print(f"Invert set to {self.invert[servo_id]}")
            elif cmd == "s":
                print(f"Saved Servo {servo_id}: middle={self.middle[servo_id]}, "
                      f"min={self.min_pos[servo_id]}, max={self.max_pos[servo_id]}, invert={self.invert[servo_id]}")
                break
            elif cmd == "q":
                return False
        return True

    def run_calibration(self):
        for i in range(self.num_servos):
            keep_going = self.calibrate_servo(i)
            if not keep_going:
                break

        print("\nFinal Calibration Data:")
        print(f"MiddlePos = {self.middle}")
        print(f"MinPos    = {self.min_pos}")
        print(f"MaxPos    = {self.max_pos}")
        print(f"Invert    = {self.invert}")


if __name__ == "__main__":
    c = ServoCalibrator(serial_port="COM43", baudrate=115200)
    c.run_calibration()
