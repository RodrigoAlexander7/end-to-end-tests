#!/usr/bin/env python3
"""
CanSat Receiver - XBee Image and Sensor Reception

Receives interleaved image chunks and sensor telemetry packets,
reassembles images, and logs sensor data.

Features:
- Saves partial images when new image starts or on timeout
- Real-time telemetry logging to CSV
"""

import sys
import time
import struct
import logging
import argparse
import csv
from pathlib import Path
from datetime import datetime

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from protocol import (
    crc16,
    parse_sensor_payload,
    SYNC_BYTES,
    PACKET_TYPE_SENSOR,
    HEADER_SIZE,
    CRC_SIZE,
)

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cansat_rx")

# Timeout for incomplete images (seconds)
IMAGE_TIMEOUT = 30.0


class CansatReceiver:
    """Receiver for interleaved image and sensor packets."""

    def __init__(self, output_dir: str, telemetry_file: str = None, save_partial: bool = True):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.save_partial = save_partial

        self.images_in_progress = {}
        self.sensor_count = 0
        self.crc_errors = 0

        # CSV logging for telemetry
        if telemetry_file:
            self.telemetry_path = Path(telemetry_file)
        else:
            self.telemetry_path = self.output_dir / "telemetry.csv"

        self._csv_file = None
        self._csv_writer = None

    def _init_csv(self):
        """Initialize CSV writer on first sensor packet."""
        if self._csv_writer is not None:
            return

        self._csv_file = open(self.telemetry_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "recv_time", "timestamp",
            "mag_x", "mag_y", "mag_z",
            "temp", "pressure", "altitude",
            "acc_x", "acc_y", "acc_z",
            "gyro_x", "gyro_y", "gyro_z",
            "voltage", "current", "power",
        ])

    def receive(self, serial_port):
        """Main receive loop."""
        log.info("Listening on %s @ %d baud...", serial_port.portstr, serial_port.baudrate)
        buffer = b""
        last_activity = time.time()

        try:
            while True:
                if serial_port.in_waiting > 0:
                    buffer += serial_port.read(serial_port.in_waiting)
                    last_activity = time.time()
                else:
                    # Check for image timeout
                    self._check_timeouts()
                    time.sleep(0.01)
                    continue

                buffer = self._process_buffer(buffer)

        except KeyboardInterrupt:
            log.info("Receiver stopped")
            # Save any remaining partial images
            self._save_all_partial()
        finally:
            if self._csv_file:
                self._csv_file.close()
            log.info("Stats: %d sensor packets, %d CRC errors", self.sensor_count, self.crc_errors)

    def _check_timeouts(self):
        """Check for timed-out images and save them as partial."""
        now = time.time()
        timed_out = []

        for img_id, img in self.images_in_progress.items():
            if now - img.get("last_update", now) > IMAGE_TIMEOUT:
                timed_out.append(img_id)

        for img_id in timed_out:
            img = self.images_in_progress[img_id]
            log.warning("Image %d timed out, saving partial", img_id)
            self._save_image(img_id, img, partial=True)
            del self.images_in_progress[img_id]

    def _save_all_partial(self):
        """Save all in-progress images as partial."""
        for img_id, img in list(self.images_in_progress.items()):
            self._save_image(img_id, img, partial=True)
        self.images_in_progress.clear()

    def _process_buffer(self, buffer: bytes) -> bytes:
        """Process all complete packets in buffer."""
        while len(buffer) >= HEADER_SIZE:
            # Find sync
            sync_pos = buffer.find(SYNC_BYTES)
            if sync_pos == -1:
                return buffer[-1:]
            if sync_pos > 0:
                buffer = buffer[sync_pos:]

            if len(buffer) < HEADER_SIZE:
                break

            # Parse header
            _, pkt_type, seq, total, pkt_len = struct.unpack(">2sBHHH", buffer[:HEADER_SIZE])

            total_size = HEADER_SIZE + pkt_len + CRC_SIZE
            if len(buffer) < total_size:
                break

            # Extract packet
            packet = buffer[:total_size]
            buffer = buffer[total_size:]

            # Verify CRC
            header_no_sync = packet[2:HEADER_SIZE]
            payload = packet[HEADER_SIZE:HEADER_SIZE + pkt_len]
            recv_crc = struct.unpack(">H", packet[-CRC_SIZE:])[0]
            calc_crc = crc16(header_no_sync + payload)

            if recv_crc != calc_crc:
                self.crc_errors += 1
                if self.crc_errors % 20 == 1:  # Log every 20th error
                    log.warning("CRC mismatch #%d: got %04X, expected %04X",
                               self.crc_errors, recv_crc, calc_crc)
                continue

            # Route by packet type
            if pkt_type == PACKET_TYPE_SENSOR:
                self._handle_sensor(payload)
            else:
                self._handle_image(pkt_type, seq, total, payload)

        return buffer

    def _handle_sensor(self, payload: bytes):
        """Process sensor telemetry packet."""
        data = parse_sensor_payload(payload)
        self.sensor_count += 1

        self._init_csv()
        self._csv_writer.writerow([
            datetime.now().isoformat(),
            data["timestamp"],
            f"{data['mag_x']:.2f}", f"{data['mag_y']:.2f}", f"{data['mag_z']:.2f}",
            f"{data['temp']:.2f}", f"{data['pressure']:.2f}", f"{data['altitude']:.2f}",
            f"{data['acc_x']:.3f}", f"{data['acc_y']:.3f}", f"{data['acc_z']:.3f}",
            f"{data['gyro_x']:.2f}", f"{data['gyro_y']:.2f}", f"{data['gyro_z']:.2f}",
            f"{data['voltage']:.3f}", f"{data['current']:.2f}", f"{data['power']:.2f}",
        ])
        self._csv_file.flush()

        if self.sensor_count % 10 == 0:
            log.info(
                "Sensor #%d: T=%.1f°C P=%.1fhPa Alt=%.1fm",
                self.sensor_count, data["temp"], data["pressure"], data["altitude"]
            )

    def _handle_image(self, img_id: int, seq: int, total: int, payload: bytes):
        """Process image chunk packet."""
        # If new image starts and we have old ones, save them as partial
        if img_id not in self.images_in_progress:
            # Save previous images as partial
            for old_id in list(self.images_in_progress.keys()):
                if old_id < img_id:
                    old_img = self.images_in_progress[old_id]
                    received = len(old_img["chunks"])
                    pct = 100 * received / old_img["total"]
                    log.info("New image %d started, saving image %d (%.0f%% complete)",
                            img_id, old_id, pct)
                    self._save_image(old_id, old_img, partial=True)
                    del self.images_in_progress[old_id]

            log.info("Receiving image %d (%d packets)", img_id, total)
            self.images_in_progress[img_id] = {
                "chunks": {},
                "total": total,
                "last_update": time.time(),
            }

        img = self.images_in_progress[img_id]
        img["last_update"] = time.time()

        if seq not in img["chunks"]:
            img["chunks"][seq] = payload
            received = len(img["chunks"])
            if received % max(1, total // 5) == 0:
                log.info("Image %d: %d/%d (%.0f%%)", img_id, received, total, 100 * received / total)

        # Check completion
        if len(img["chunks"]) == img["total"]:
            self._save_image(img_id, img, partial=False)
            del self.images_in_progress[img_id]

    def _save_image(self, img_id: int, img: dict, partial: bool = False):
        """Reassemble and save image (complete or partial)."""
        received = len(img["chunks"])
        total = img["total"]
        pct = 100 * received / total

        if received == 0:
            log.warning("Image %d has no chunks, skipping", img_id)
            return

        # Reassemble with gaps filled by zeros
        data = b""
        missing = 0
        for seq in range(total):
            if seq in img["chunks"]:
                data += img["chunks"][seq]
            else:
                # Fill missing chunks with zeros (will cause JPEG artifacts)
                missing += 1
                data += b"\x00" * 200  # Approximate chunk size

        if partial:
            out_path = self.output_dir / f"image_{img_id:03d}_partial_{pct:.0f}pct.jpg"
            log.warning("Saved PARTIAL %s (%d/%d chunks, %d missing)",
                       out_path.name, received, total, missing)
        else:
            out_path = self.output_dir / f"image_{img_id:03d}.jpg"
            log.info("Saved %s (%d bytes, 100%% complete)", out_path.name, len(data))

        with open(out_path, "wb") as f:
            f.write(data)


def main():
    parser = argparse.ArgumentParser(description="CanSat Image & Sensor Receiver")
    parser.add_argument("--serial-port", type=str, required=True, help="Serial port")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baudrate")
    parser.add_argument("--output-dir", type=str, default="received", help="Output directory")
    parser.add_argument("--telemetry-file", type=str, default=None, help="Telemetry CSV path")

    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        log.error("pyserial not installed: pip install pyserial")
        sys.exit(1)

    receiver = CansatReceiver(args.output_dir, args.telemetry_file)

    try:
        ser = serial.Serial(port=args.serial_port, baudrate=args.baudrate, timeout=1)
    except Exception as e:
        log.error("Failed to open %s: %s", args.serial_port, e)
        sys.exit(1)

    try:
        receiver.receive(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
