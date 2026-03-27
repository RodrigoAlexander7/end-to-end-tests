#!/usr/bin/env python3
"""Simple XBee TX test - sends numbered messages every second."""

import sys
import time
import serial
import argparse


def main():
    parser = argparse.ArgumentParser(description="XBee TX Test")
    parser.add_argument("--port", type=str, required=True, help="Serial port")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate")
    args = parser.parse_args()

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baudrate, timeout=1)
        print(f"TX opened: {args.port} @ {args.baudrate} baud")
    except Exception as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    count = 0
    try:
        while True:
            msg = f"TEST:{count:05d}\n"
            ser.write(msg.encode())
            ser.flush()
            print(f"Sent: {msg.strip()}")
            count += 1
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
