#!/usr/bin/env python3

import serial
import argparse
import time
import threading
import sys

def read_serial(ser, name):
    while ser and ser.is_open:
        try:
            line = ser.readline()
            if line:
                print(f"[{name}] {line.decode('ascii', errors='replace').strip()}")
        except serial.SerialException:
            break
        except Exception as e:
            break

def main():
    parser = argparse.ArgumentParser(description="Test script for Dual ODESC4.2 boards")
    parser.add_argument('-p1', '--port1', type=str, default='/dev/ttyAMA0', help="Serial port 1 (default: /dev/ttyAMA0 -> Pins 14/15)")
    parser.add_argument('-p2', '--port2', type=str, default='/dev/ttyAMA3', help="Serial port 2 (default: /dev/ttyAMA3 -> GPIO 8/9 via uart3)")
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument('-c', '--clear-errors', action='store_true', help="Clear errors on both")
    parser.add_argument('-v', '--velocity', type=float, help="Velocity command in turns/s (runs both simultaneously)")
    parser.add_argument('-i', '--interactive', action='store_true', help="Enter interactive mode")

    args = parser.parse_args()

    # Connect Port 1
    try:
        ser1 = serial.Serial(args.port1, args.baudrate, timeout=0.1)
        print(f"Connected to Port 1: {args.port1}")
        threading.Thread(target=read_serial, args=(ser1, "ODESC 1"), daemon=True).start()
    except Exception as e:
        print(f"Failed to connect to Port 1 ({args.port1}): {e}")
        ser1 = None

    # Connect Port 2
    try:
        ser2 = serial.Serial(args.port2, args.baudrate, timeout=0.1)
        print(f"Connected to Port 2: {args.port2}")
        threading.Thread(target=read_serial, args=(ser2, "ODESC 2"), daemon=True).start()
    except Exception as e:
        print(f"Failed to connect to Port 2 ({args.port2}): {e}")
        ser2 = None

    if not ser1 and not ser2:
        print("Both serial connections failed. Exiting.")
        sys.exit(1)

    time.sleep(0.5)

    if args.clear_errors:
        print("Clearing errors on both...")
        if ser1: ser1.write(b'sc\n')
        if ser2: ser2.write(b'sc\n')
        time.sleep(0.1)
    
    if args.velocity is not None:
        cmd = f"v 0 {args.velocity}\n" # Assuming Axis 0 is used on both boards
        print(f"Sending velocity command to both: {cmd.strip()}")
        if ser1: ser1.write(cmd.encode('ascii'))
        if ser2: ser2.write(cmd.encode('ascii'))
        time.sleep(2)
        print("Stopping both motors...")
        if ser1: ser1.write(b"v 0 0.0\n")
        if ser2: ser2.write(b"v 0 0.0\n")

    if args.interactive:
        print("\n--- Dual Interactive Mode ---")
        print("Type commands normally (e.g. 'v 0 1.0') to send to BOTH motors.")
        print("Prefix with '1 ' or '2 ' to send to a specific motor (e.g. '1 f 0').")
        print("Type 'q' to quit.")
        try:
            while True:
                user_input = input("> ")
                if user_input.lower() in ('q', 'quit', 'exit'):
                    break
                
                user_input = user_input.strip()
                if not user_input:
                    continue
                
                # Check for prefix
                if user_input.startswith("1 "):
                    cmd = user_input[2:] + "\n"
                    if ser1: ser1.write(cmd.encode('ascii'))
                elif user_input.startswith("2 "):
                    cmd = user_input[2:] + "\n"
                    if ser2: ser2.write(cmd.encode('ascii'))
                else:
                    cmd = user_input + "\n"
                    if ser1: ser1.write(cmd.encode('ascii'))
                    if ser2: ser2.write(cmd.encode('ascii'))

        except KeyboardInterrupt:
            print("\nExiting...")

    time.sleep(0.5)
    if ser1: ser1.close()
    if ser2: ser2.close()
    print("Done.")

if __name__ == "__main__":
    main()
