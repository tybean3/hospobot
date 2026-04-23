import time
import board
import busio
import math
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

def main():
    try:
        # Re-initializing bus with standard parameters
        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C bus initialized. Resetting BNO086...")
        
        # Address is 0x4b based on your i2cdetect
        bno = BNO08X_I2C(i2c, address=0x4b)
        
        # The BNO08x sometimes gets stuck if it sends a packet the library doesn't like.
        # We'll try to enable the Rotation Vector directly.
        print("Enabling Rotation Vector...")
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        print("Success! Printing Yaw, Pitch, Roll. Press Ctrl+C to stop.")
        
        while True:
            time.sleep(0.05) # Faster polling can sometimes help drain the buffer
            quat = bno.quaternion # (i, j, k, real)
            
            # Simple conversion for a quick check
            # Yaw
            siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1])
            cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            
            print(f"Yaw: {yaw:7.2f} degrees", end="\r")
            
    except Exception as e:
        print(f"\nError: {e}")
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
