#!/usr/bin/env python3

import serial
import argparse
import time
import sys
import termios
import tty

def getch():
    """Reads a single character from standard input without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def clear_serial_buffer(ser):
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)

def calibrate_and_start(ser, name):
    print(f"[{name}] Clearing errors...")
    ser.write(b"sc\n")
    time.sleep(0.5)
    
    print(f"[{name}] Stating full calibration (Beeping + turning)...")
    clear_serial_buffer(ser)
    ser.write(b"w axis0.requested_state 3\n")
    
    print(f"[{name}] Waiting 10 seconds for calibration dance...")
    time.sleep(10.0)
    
    print(f"[{name}] Waiting for calibration to finish (Checking state)...")
    for _ in range(30):
        ser.write(b"r axis0.current_state\n")
        time.sleep(0.1)
        response = ser.readline().decode('ascii', errors='ignore').strip()
        if "1" in response:
            print(f"[{name}] State returned to 1 (Idle). Calibration done!")
            break
        time.sleep(0.5)
    
    print(f"[{name}] Setting to Closed Loop Control (State 8)...")
    ser.write(b"w axis0.requested_state 8\n")
    time.sleep(0.5)

def main():
    parser = argparse.ArgumentParser(description="WASD Teleop for Dual ODESC Rover")
    parser.add_argument('--left-port', default='/dev/ttyAMA0', help="Left motor port")
    parser.add_argument('--right-port', default='/dev/ttyAMA3', help="Right motor port")
    parser.add_argument('-b', '--baudrate', type=int, default=115200)
    parser.add_argument('-v', '--velocity', type=float, default=2.0, help="Forward/Back speed in turns/s")
    parser.add_argument('-t', '--turn-velocity', type=float, default=1.0, help="Turning speed in turns/s")
    parser.add_argument('--invert-left', action='store_true', help="Invert left motor direction")
    parser.add_argument('--invert-right', action='store_true', help="Invert right motor direction")
    
    args = parser.parse_args()

    # In a typical differential drive, one motor might need to be inverted
    # so that positive velocity moves the robot forward on both sides.
    # You can change defaults here or pass flags if your robot spins in circles on 'w'
    left_mult = -1.0 if args.invert_left else 1.0
    right_mult = -1.0 if args.invert_right else 1.0

    print("Connecting to ODESC controllers...")
    try:
        ser_left = serial.Serial(args.left_port, args.baudrate, timeout=0.1)
        ser_right = serial.Serial(args.right_port, args.baudrate, timeout=0.1)
    except Exception as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    print("\n--- Calibration Phase ---")
    calibrate_and_start(ser_left, "LEFT Motor")
    calibrate_and_start(ser_right, "RIGHT Motor")

    print("\n==================================")
    print("      ROVER TELEOP READY")
    print("==================================")
    print(" W : Forward")
    print(" S : Reverse")
    print(" A : Turn Left")
    print(" D : Turn Right")
    print(" SPACE : Stop")
    print(" Q : Quit")
    print("----------------------------------")

    v_lin = args.velocity
    v_trn = args.turn_velocity

    def send_vel(v_l, v_r):
        vl_actual = v_l * left_mult
        vr_actual = v_r * right_mult
        ser_left.write(f"v 0 {vl_actual}\n".encode('ascii'))
        ser_right.write(f"v 0 {vr_actual}\n".encode('ascii'))

    try:
        while True:
            ch = getch().lower()
            if ch == 'q':
                break
            elif ch == 'w':
                print("Forward")
                send_vel(v_lin, v_lin)
            elif ch == 's':
                print("Reverse")
                send_vel(-v_lin, -v_lin)
            elif ch == 'a':
                print("Left")
                send_vel(-v_trn, v_trn)
            elif ch == 'd':
                print("Right")
                send_vel(v_trn, -v_trn)
            elif ch == ' ':
                print("STOP")
                send_vel(0.0, 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping motors and returning to Idle...")
        send_vel(0.0, 0.0)
        time.sleep(0.1)
        ser_left.write(b"w axis0.requested_state 1\n")
        ser_right.write(b"w axis0.requested_state 1\n")
        ser_left.close()
        ser_right.close()
        print("Done.")

if __name__ == "__main__":
    main()
