#!/usr/bin/env python3
"""
CanSat Pipeline with Sensor Telemetry

Captures images via C++ pipeline and transmits over XBee with interleaved
sensor data using 10:1 ratio (10 image chunks : 1 sensor packet).
"""

import sys
import time
import logging
import argparse
import numpy as np
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from sensors import SensorHub
from transmission import transmit_images_with_telemetry

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cansat")


def load_calibration(pipeline, calib_path: str) -> bool:
    """Load calibration remap matrices from a .npz file."""
    path = Path(calib_path)
    if not path.exists():
        log.warning("Calibration file not found: %s", calib_path)
        return False

    data = np.load(calib_path)
    required = ["map1_left", "map2_left", "map1_right", "map2_right"]

    if all(k in data for k in required):
        pipeline.load_calibration(
            data["map1_left"], data["map2_left"],
            data["map1_right"], data["map2_right"],
        )
        log.info("Calibration loaded from %s", calib_path)
        return True

    # Try computing from intrinsics
    try:
        import cv2

        maps = []
        for cam_key, dist_key in [("K_L", "D_L"), ("K_R", "D_R")]:
            if cam_key not in data or dist_key not in data:
                return False
            K, D = data[cam_key], data[dist_key]
            w, h = (int(data["img_size"][0]), int(data["img_size"][1])) if "img_size" in data else (1280, 720)
            new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_32FC1)
            maps.extend([m1, m2])

        pipeline.load_calibration(maps[0], maps[1], maps[2], maps[3])
        log.info("Calibration computed from intrinsics")
        return True
    except (ImportError, KeyError):
        return False


def run(args):
    """Main pipeline execution."""
    t_start = time.perf_counter()

    # Import C++ module
    try:
        import cansat_pipeline as cpp
    except ImportError:
        log.error("Cannot import cansat_pipeline. Build with: mkdir build && cd build && cmake .. && make")
        sys.exit(1)

    # Initialize sensors
    t_sensors_start = time.perf_counter()
    sensors = SensorHub()
    if not sensors.initialize():
        log.warning("No sensors available, continuing without telemetry")
        sensor_reader = None
    else:
        sensor_reader = sensors.read_all
    t_sensors = time.perf_counter() - t_sensors_start
    log.info("[TIMING] Sensor init: %.2fs", t_sensors)

    # Configure pipeline
    config = cpp.PipelineConfig()
    config.device_id = args.device
    config.frame_width = args.width
    config.frame_height = args.height
    config.jpeg_quality = args.quality
    config.num_captures = args.num_captures
    config.interval_seconds = args.interval

    t_pipeline_init = time.perf_counter()
    pipeline = cpp.Pipeline(config)
    pipeline.initialize()

    if args.calibration:
        load_calibration(pipeline, args.calibration)
    t_pipeline = time.perf_counter() - t_pipeline_init
    log.info("[TIMING] Pipeline init: %.2fs", t_pipeline)

    # Acquire images
    log.info("Starting acquisition...")
    t_capture_start = time.perf_counter()
    images = pipeline.run_acquisition()
    t_capture = time.perf_counter() - t_capture_start

    total_bytes = sum(len(img) for img in images if img)
    log.info("[TIMING] Capture: %.2fs for %d images (%.1f KB total)",
             t_capture, len(images), total_bytes / 1024)

    # Transmit or save
    if args.serial_port:
        try:
            import serial

            ser = serial.Serial(
                port=args.serial_port,
                baudrate=args.baudrate,
                timeout=1,
            )
            log.info("XBee opened: %s @ %d baud", args.serial_port, args.baudrate)

            t_tx_start = time.perf_counter()
            success = transmit_images_with_telemetry(
                ser, images, sensor_reader, inter_image_delay=0.5
            )
            t_tx = time.perf_counter() - t_tx_start

            throughput = total_bytes / t_tx if t_tx > 0 else 0
            log.info("[TIMING] Transmission: %.2fs for %d/%d images (%.1f B/s)",
                     t_tx, success, len(images), throughput)

            ser.close()
        except ImportError:
            log.error("pyserial not installed: pip install pyserial")
        except Exception as e:
            log.error("Serial error: %s", e)
    else:
        # Save to disk
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        for idx, img_data in enumerate(images):
            if not img_data:
                continue
            out_path = output_dir / f"anaglyph_{idx:02d}.jpg"
            with open(out_path, "wb") as f:
                f.write(img_data)
            log.info("Saved %s (%d bytes)", out_path, len(img_data))

    # Cleanup
    pipeline.shutdown()
    sensors.close()

    t_total = time.perf_counter() - t_start
    log.info("[TIMING] Total pipeline: %.2fs", t_total)


def main():
    parser = argparse.ArgumentParser(
        description="CanSat Pipeline with Sensor Telemetry"
    )
    parser.add_argument("--device", type=int, default=0, help="Camera device ID")
    parser.add_argument("--width", type=int, default=2560, help="Frame width")
    parser.add_argument("--height", type=int, default=720, help="Frame height")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality")
    parser.add_argument("--num-captures", type=int, default=3, help="Number of captures")
    parser.add_argument("--interval", type=int, default=3, help="Seconds between captures")
    parser.add_argument("--calibration", type=str, default="calibration.npz", help="Calibration file")
    parser.add_argument("--serial-port", type=str, default=None, help="XBee serial port")
    parser.add_argument("--baudrate", type=int, default=9600, help="Serial baudrate")
    parser.add_argument("--output-dir", type=str, default="output", help="Output directory")

    run(parser.parse_args())


if __name__ == "__main__":
    main()
