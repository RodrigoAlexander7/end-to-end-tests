#!/usr/bin/env python3
"""
Standalone test script for the CanSat pipeline.
Tests the C++ module without requiring a camera (uses synthetic images).
"""

import sys
import time
import numpy as np
from pathlib import Path


def test_pipeline_with_camera():
    """Test pipeline with an actual camera device."""
    try:
        import cansat_pipeline as cpp
    except ImportError:
        print("ERROR: Cannot import cansat_pipeline. Build it first.")
        print("  mkdir build && cd build && cmake .. && make")
        sys.exit(1)

    config = cpp.PipelineConfig()
    config.device_id = 0
    config.jpeg_quality = 80
    config.num_captures = 3
    config.interval_seconds = 3

    pipeline = cpp.Pipeline(config)

    # Load calibration if available
    calib_path = Path("calibration.npz")
    if calib_path.exists():
        data = np.load(str(calib_path))
        required = ["map1_left", "map2_left", "map1_right", "map2_right"]
        if all(k in data for k in required):
            pipeline.load_calibration(
                data["map1_left"], data["map2_left"],
                data["map1_right"], data["map2_right"],
            )
            print("Calibration loaded")
        else:
            print("Calibration file missing remap keys, proceeding without")
    else:
        print("No calibration file found, proceeding without")

    pipeline.initialize()

    print("\nRunning acquisition...")
    images = pipeline.run_acquisition()

    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)
    for idx, img_data in enumerate(images):
        if not img_data:
            print(f"  Image {idx}: FAILED (empty)")
            continue
        path = output_dir / f"test_anaglyph_{idx:02d}.jpg"
        with open(path, "wb") as f:
            f.write(img_data)
        print(f"  Image {idx}: {len(img_data)} bytes -> {path}")

    pipeline.shutdown()
    print("\nTest complete!")


def test_xbee_packets():
    """Test packet building and CRC without serial hardware."""
    sys.path.insert(0, str(Path(__file__).parent))
    from cansat_pipeline import crc16, build_packet

    print("Testing XBee packet construction...")

    test_data = b"Hello CanSat!" * 20  # 260 bytes
    max_payload = 200
    total_chunks = (len(test_data) + max_payload - 1) // max_payload

    for seq in range(total_chunks):
        offset = seq * max_payload
        chunk = test_data[offset: offset + max_payload]
        packet = build_packet(0, seq, total_chunks, chunk)
        print(f"  Packet {seq}: {len(packet)} bytes (payload: {len(chunk)})")

    print("Packet test passed!")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="CanSat Pipeline Test")
    parser.add_argument(
        "--mode",
        choices=["camera", "packets"],
        default="camera",
        help="Test mode: 'camera' (full pipeline) or 'packets' (XBee only)",
    )
    args = parser.parse_args()

    if args.mode == "camera":
        test_pipeline_with_camera()
    elif args.mode == "packets":
        test_xbee_packets()
