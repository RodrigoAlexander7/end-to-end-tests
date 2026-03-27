#!/usr/bin/env python3
"""
Stream sensor telemetry only (no images) - for testing real-time sensor data.
"""

import sys
import logging
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from sensors import SensorHub
from transmission import stream_telemetry_only

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cansat")


def main():
    parser = argparse.ArgumentParser(description="Stream sensor telemetry via XBee")
    parser.add_argument("--serial-port", type=str, required=True, help="XBee serial port")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between packets")
    parser.add_argument("--duration", type=float, default=None, help="Duration in seconds (None=forever)")

    args = parser.parse_args()

    # Initialize sensors
    sensors = SensorHub()
    if not sensors.initialize():
        log.error("No sensors available")
        sys.exit(1)

    # Open serial
    try:
        import serial
        ser = serial.Serial(port=args.serial_port, baudrate=args.baudrate, timeout=1)
        log.info("XBee opened: %s @ %d baud", args.serial_port, args.baudrate)
    except Exception as e:
        log.error("Failed to open serial: %s", e)
        sys.exit(1)

    # Stream telemetry
    try:
        stream_telemetry_only(ser, sensors.read_all, args.duration, args.interval)
    finally:
        ser.close()
        sensors.close()


if __name__ == "__main__":
    main()
