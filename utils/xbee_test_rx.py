#!/usr/bin/env python3
"""Simple XBee RX test - prints everything received."""

import sys
import serial
import argparse


def main():
    parser = argparse.ArgumentParser(description="XBee RX Test")
    parser.add_argument("--port", type=str, required=True, help="Serial port")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate")
    args = parser.parse_args()

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baudrate, timeout=1)
        print(f"RX listening: {args.port} @ {args.baudrate} baud")
        print("Waiting for data... (Ctrl+C to stop)\n")
    except Exception as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                # Show as text and hex
                try:
                    text = data.decode('utf-8', errors='replace')
                    print(f"RX ({len(data)} bytes): {text.strip()}")
                except Exception:
                    print(f"RX ({len(data)} bytes): {data.hex()}")
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
