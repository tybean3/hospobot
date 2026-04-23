#!/usr/bin/env python3

import serial
import argparse
import time
import threading
import sys

def read_serial(ser):
    while ser.is_open:
        try:
            line = ser.readline()
            if line:
                print(f"[ODESC] {line.decode('ascii', errors='replace').strip()}")
        except serial.SerialException:
            break
        except Exception as e:
            print(f"[Read Error] {e}")
            break

def main():
    parser = argparse.ArgumentParser(description="Test script for ODESC4.2 via UART")
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyAMA10', help="Serial port (default: /dev/ttyAMA10 which often maps to pins 14/15 on Pi 5)")
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument('-a', '--axis', type=int, default=0, help="Axis number to control (0 or 1, default: 0)")
    parser.add_argument('-v', '--velocity', type=float, help="Velocity command in turns/s (e.g. 0.5)")
    parser.add_argument('-P', '--position', type=float, help="Position command in turns (e.g. 10)")
    parser.add_argument('-c', '--clear-errors', action='store_true', help="Clear errors on the ODESC")
    parser.add_argument('-i', '--interactive', action='store_true', help="Enter interactive mode to type commands manually")

    args = parser.parse_args()

    try:
        print(f"Connecting to {args.port} at {args.baudrate} baud...")
        ser = serial.Serial(args.port, args.baudrate, timeout=0.1)
        print("Connected successfully!")
    except serial.SerialException as e:
        print(f"Failed to connect to serial port: {e}")
        print("Hint: Make sure hardware UART is enabled in raspi-config or /boot/config.txt")
        print("      Also ensure your user is in the 'dialout' group: sudo usermod -a -G dialout $USER")
        sys.exit(1)

    # Start a background thread to print any responses from the ODESC
    read_thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
    read_thread.start()

    time.sleep(0.5) # Give it a moment to connect

    if args.clear_errors:
        print("Clearing errors...")
        ser.write(b'sc\n')
        time.sleep(0.1)
    
    if args.velocity is not None:
        cmd = f"v {args.axis} {args.velocity}\n"
        print(f"Sending velocity command: {cmd.strip()}")
        ser.write(cmd.encode('ascii'))
        time.sleep(2) # Run for 2 seconds
        # Stop
        cmd_stop = f"v {args.axis} 0.0\n"
        print("Stopping motor...")
        ser.write(cmd_stop.encode('ascii'))

    elif args.position is not None:
        cmd = f"p {args.axis} {args.position}\n"
        print(f"Sending position command: {cmd.strip()}")
        ser.write(cmd.encode('ascii'))

    if args.interactive:
        print("\n--- Interactive Mode ---")
        print("Type commands (e.g. 'v 0 1.0', 'f 0') and press Enter. Type 'q' to quit.")
        try:
            while True:
                user_input = input("> ")
                if user_input.lower() in ('q', 'quit', 'exit'):
                    break
                # Force closed loop mode if giving a 'v' or 'p' command just in case?
                # Actually ODESC ASCII is simple, just send it.
                ser.write((user_input + '\n').encode('ascii'))
        except KeyboardInterrupt:
            print("\nExiting...")

    # Wait a bit to catch any remaining output
    time.sleep(0.5)
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
