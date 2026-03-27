#!/usr/bin/env python3
"""Quick test to verify module imports work correctly."""

import sys
from pathlib import Path

# Add src to path (same as cansat_pipeline_sensors.py)
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

print("Testing imports...")

try:
    from protocol import crc16, build_image_packet, build_sensor_packet
    print("✓ protocol module imported")
except ImportError as e:
    print(f"✗ Failed to import protocol: {e}")
    sys.exit(1)

try:
    from sensors import SensorHub
    print("✓ sensors module imported")
except ImportError as e:
    print(f"✗ Failed to import sensors: {e}")
    sys.exit(1)

try:
    from transmission import transmit_images_with_telemetry, XBeeTransmitter
    print("✓ transmission module imported")
except ImportError as e:
    print(f"✗ Failed to import transmission: {e}")
    sys.exit(1)

print("\nAll imports successful! ✓")
