#!/usr/bin/env python3
"""
CanSat Pipeline - Orchestration and XBee Transmission

Calls the C++ pipeline for image capture/processing, then transmits
JPEG images over XBee serial with chunked packets and CRC-16 integrity.
"""

import sys
import time
import struct
import logging
import argparse
import numpy as np
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cansat")


def initialize_servo(gpio_pin=18):
    """Inicializa la rutina de movimiento del servo al arrancar."""
    try:
        from gpiozero import Servo
        from time import sleep
        
        # GPIO 18 (pin físico 12)
        servo = Servo(
            gpio_pin,
            min_pulse_width=0.5/1000,
            max_pulse_width=2.5/1000
        )

        log.info("Moviendo al máximo (derecha)...")
        servo.max()
        sleep(2)

        log.info("Moviendo al mínimo (izquierda)...")
        servo.min()
        sleep(2)

        log.info("Regresando al centro...")
        servo.mid()
        sleep(1)
        
        servo.detach()
    except Exception as e:
        log.warning("Servo initialization fallback: %s", e)


# ---------------------------------------------------------------------------
# CRC-16 (CCITT) for packet integrity
# ---------------------------------------------------------------------------

_CRC_TABLE = None


def _build_crc_table():
    global _CRC_TABLE
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        table.append(crc & 0xFFFF)
    _CRC_TABLE = table


def crc16(data: bytes) -> int:
    """Compute CRC-16/CCITT over data."""
    if _CRC_TABLE is None:
        _build_crc_table()
    crc = 0xFFFF
    for byte in data:
        crc = ((_CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]) ^ (crc << 8)) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# XBee Packet Transmission
# ---------------------------------------------------------------------------

# Packet structure:
#   [SYNC (2B)] [IMG_ID (1B)] [SEQ (2B)] [TOTAL (2B)] [LEN (2B)] [PAYLOAD (≤200B)] [CRC (2B)]
SYNC_BYTES = b"\xAA\x55"
MAX_PAYLOAD = 200  # Conservative for XBee packet limits
HEADER_SIZE = 9    # sync(2) + img_id(1) + seq(2) + total(2) + len(2)
CRC_SIZE = 2
INTER_PACKET_DELAY = 0.05  # 50ms between packets to reduce CPU/radio contention


def build_packet(image_id: int, seq: int, total: int, payload: bytes) -> bytes:
    """Build a transmission packet with CRC-16 integrity."""
    header = struct.pack(
        ">2sBHHH",
        SYNC_BYTES,
        image_id & 0xFF,
        seq,
        total,
        len(payload),
    )
    crc = crc16(header[2:] + payload)  # CRC over everything after sync
    return header + payload + struct.pack(">H", crc)


def transmit_image(serial_port, image_data: bytes, image_id: int) -> bool:
    """
    Transmit a JPEG image over XBee serial in chunked packets.

    Args:
        serial_port: Open serial.Serial instance.
        image_data:  Raw JPEG bytes.
        image_id:    Image identifier (0-255).

    Returns:
        True if all packets were sent successfully.
    """
    total_chunks = (len(image_data) + MAX_PAYLOAD - 1) // MAX_PAYLOAD
    log.info(
        "TX image %d: %d bytes in %d packets", image_id, len(image_data), total_chunks
    )

    for seq in range(total_chunks):
        offset = seq * MAX_PAYLOAD
        chunk = image_data[offset : offset + MAX_PAYLOAD]
        packet = build_packet(image_id, seq, total_chunks, chunk)

        try:
            serial_port.write(packet)
            serial_port.flush()
        except Exception as e:
            log.error("TX failed at packet %d/%d: %s", seq + 1, total_chunks, e)
            return False

        # Small delay to avoid saturating the radio and reduce CPU usage
        time.sleep(INTER_PACKET_DELAY)

    log.info("TX image %d complete", image_id)
    return True


# ---------------------------------------------------------------------------
# Pipeline Orchestration
# ---------------------------------------------------------------------------


def load_calibration(pipeline, calib_path: str):
    """Load calibration remap matrices from a .npz file and pass to pipeline."""
    path = Path(calib_path)
    if not path.exists():
        log.warning("Calibration file not found: %s (proceeding without calibration)", calib_path)
        return False

    data = np.load(calib_path)

    # Expected keys for precomputed remap matrices
    required_keys = ["map1_left", "map2_left", "map1_right", "map2_right"]

    # Check if remap matrices exist directly
    if all(k in data for k in required_keys):
        pipeline.load_calibration(
            data["map1_left"],
            data["map2_left"],
            data["map1_right"],
            data["map2_right"],
        )
        log.info("Calibration loaded from %s", calib_path)
        return True

    # If not, try to compute from intrinsics/distortion (requires OpenCV in Python)
    try:
        import cv2

        intrinsic_keys = [
            ("K_L", "D_L"),
            ("K_R", "D_R"),
        ]
        maps = []
        for cam_key, dist_key in intrinsic_keys:
            if cam_key not in data or dist_key not in data:
                log.warning("Missing key %s or %s in calibration", cam_key, dist_key)
                return False
            K = data[cam_key]
            D = data[dist_key]
            
            if "img_size" in data:
                w, h = int(data["img_size"][0]), int(data["img_size"][1])
            else:
                w, h = 1280, 720
                
            new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_32FC1)
            maps.extend([m1, m2])

        pipeline.load_calibration(maps[0], maps[1], maps[2], maps[3])
        log.info("Calibration computed from intrinsics in %s", calib_path)
        return True
    except ImportError:
        log.warning("OpenCV not available in Python; cannot compute remap from intrinsics")
        return False


def run(args):
    """Main pipeline execution."""
    try:
        import cansat_pipeline as cpp
    except ImportError:
        log.error(
            "Cannot import cansat_pipeline C++ module. "
            "Build it first: mkdir build && cd build && cmake .. && make"
        )
        sys.exit(1)

    # Configure pipeline
    config = cpp.PipelineConfig()
    config.device_id = args.device
    config.frame_width = args.width
    config.frame_height = args.height
    config.jpeg_quality = args.quality
    config.num_captures = args.num_captures
    config.interval_seconds = args.interval

    pipeline = cpp.Pipeline(config)

    # Initialize
    pipeline.initialize()
    
    # Initialize servo sweep just like the example
    if not getattr(args, 'no_servo', False):
        initialize_servo(getattr(args, 'servo_pin', 18))

    # Load calibration if available
    if args.calibration:
        load_calibration(pipeline, args.calibration)

    # Run acquisition
    images = pipeline.run_acquisition()

    # Transmit via XBee if serial port specified
    if args.serial_port:
        try:
            import serial

            ser = serial.Serial(
                port=args.serial_port,
                baudrate=args.baudrate,
                timeout=1,
            )
            log.info("XBee serial opened: %s @ %d baud", args.serial_port, args.baudrate)

            for idx, img_data in enumerate(images):
                if not img_data:
                    log.warning("Skipping empty image %d", idx)
                    continue
                transmit_image(ser, img_data, idx)
                if idx < len(images) - 1:
                    time.sleep(0.5)  # Brief pause between images

            ser.close()
            log.info("XBee serial closed")
        except ImportError:
            log.error("pyserial not installed. Install with: pip install pyserial")
        except Exception as e:
            log.error("Serial error: %s", e)
    else:
        # Save to disk for testing
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        for idx, img_data in enumerate(images):
            if not img_data:
                log.warning("Skipping empty image %d", idx)
                continue
            out_path = output_dir / f"anaglyph_{idx:02d}.jpg"
            with open(out_path, "wb") as f:
                f.write(img_data)
            log.info("Saved %s (%d bytes)", out_path, len(img_data))

    pipeline.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="CanSat Image Pipeline - Capture, Process, Transmit"
    )
    parser.add_argument("--device", type=int, default=0, help="Camera device ID")
    parser.add_argument("--width", type=int, default=2560, help="Frame width (stereo)")
    parser.add_argument("--height", type=int, default=720, help="Frame height")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality (0-100)")
    parser.add_argument("--num-captures", type=int, default=3, help="Number of captures")
    parser.add_argument("--interval", type=int, default=3, help="Seconds between captures")
    parser.add_argument(
        "--calibration", type=str, default="calibration.npz",
        help="Path to calibration .npz file",
    )
    parser.add_argument(
        "--serial-port", type=str, default=None,
        help="XBee serial port (e.g., /dev/ttyUSB0). If omitted, saves to disk.",
    )
    parser.add_argument("--baudrate", type=int, default=9600, help="Serial baudrate")
    parser.add_argument(
        "--output-dir", type=str, default="output",
        help="Directory for output images (when not using serial)",
    )
    parser.add_argument(
        "--servo-pin", type=int, default=18,
        help="BCM GPIO pin for servo (default: 18)",
    )
    parser.add_argument(
        "--no-servo", action="store_true",
        help="Skip servo initialization",
    )

    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
