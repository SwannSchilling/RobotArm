#!/usr/bin/env python3
import can
import struct
import time

def decode_arm_telemetry(data: bytes) -> dict:
    """Adjust this based on your arm's CAN protocol matrix."""
    # Example: Big-endian 16-bit unsigned integer at bytes 0-1
    raw_val = struct.unpack('>H', data[0:2])[0]  # FF 3C -> 65340
    
    # Example: Status byte at index 6
    status = data[6]  # 0x20 -> 32 decimal
    
    # Example: Sequence/counter at index 7
    seq = data[7]
    
    # If your arm uses scaling (e.g., value / 100.0 = degrees):
    # scaled_val = raw_val / 100.0
    
    return {"raw_val": raw_val, "status": status, "seq": seq}

def main():
    print("🔌 Initializing CAN bus on can0...")
    try:
        bus = can.interface.Bus(channel="can0", interface="socketcan")
        
        # Optional: Filter ONLY 0x2968 to save CPU
        bus.set_filters([{"can_id": 0x2968, "can_mask": 0x7FF}])
        
        print("✅ Listening on can0... (Ctrl+C to stop)\n")
        
        start = time.time()
        count = 0

        for msg in bus:
            count += 1
            decoded = decode_arm_telemetry(msg.data)
            elapsed = time.time() - start
            
            # Print every 20 frames (~2.5s at 50Hz)
            if count % 20 == 0:
                rate = count / elapsed if elapsed > 0 else 0
                print(f"[{msg.timestamp:.3f}] Rate: {rate:.1f} Hz | "
                      f"Val: {decoded['raw_val']} | "
                      f"Status: 0x{decoded['status']:02X} | Seq: {decoded['seq']}")

    except KeyboardInterrupt:
        print("\n⏹️ Stopped.")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()

if __name__ == "__main__":
    main()