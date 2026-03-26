"""
CanSat Transmission - XBee Pro S1 transmission with interleaved sensor/image data.

Implements 10:1 ratio: sends 10 image chunks, then 1 sensor packet.
This ensures continuous telemetry while transmitting images.
"""

import time
import logging
from typing import Callable, Optional

from .protocol import (
    build_image_packet,
    build_sensor_packet,
    MAX_PAYLOAD,
    INTER_PACKET_DELAY,
)

log = logging.getLogger("cansat.tx")

# Transmission ratio: N image packets per 1 sensor packet
IMAGE_TO_SENSOR_RATIO = 10


class XBeeTransmitter:
    """XBee transmission handler with interleaved sensor telemetry."""

    def __init__(self, serial_port, sensor_reader: Optional[Callable[[], dict]] = None):
        """
        Args:
            serial_port: Open serial.Serial instance.
            sensor_reader: Callable that returns current sensor data dict.
                           If None, sensor packets are skipped.
        """
        self._serial = serial_port
        self._sensor_reader = sensor_reader
        self._packet_count = 0

    def _send_packet(self, packet: bytes) -> bool:
        """Send a single packet over serial."""
        try:
            self._serial.write(packet)
            self._serial.flush()
            return True
        except Exception as e:
            log.error("TX failed: %s", e)
            return False

    def _maybe_send_sensor(self):
        """Send sensor packet if it's time (every N image packets)."""
        if self._sensor_reader is None:
            return

        self._packet_count += 1
        if self._packet_count >= IMAGE_TO_SENSOR_RATIO:
            self._packet_count = 0
            sensor_data = self._sensor_reader()
            packet = build_sensor_packet(sensor_data)
            if self._send_packet(packet):
                log.debug("TX sensor packet (t=%d)", sensor_data.get("timestamp", 0))
            time.sleep(INTER_PACKET_DELAY)

    def transmit_image(self, image_data: bytes, image_id: int) -> bool:
        """
        Transmit a JPEG image with interleaved sensor packets.

        Sends image in chunks, inserting a sensor packet every 10 image chunks.

        Args:
            image_data: Raw JPEG bytes.
            image_id: Image identifier (0-254).

        Returns:
            True if all packets sent successfully.
        """
        total_chunks = (len(image_data) + MAX_PAYLOAD - 1) // MAX_PAYLOAD
        log.info("TX image %d: %d bytes, %d packets", image_id, len(image_data), total_chunks)

        for seq in range(total_chunks):
            # Extract chunk
            offset = seq * MAX_PAYLOAD
            chunk = image_data[offset : offset + MAX_PAYLOAD]

            # Build and send image packet
            packet = build_image_packet(image_id, seq, total_chunks, chunk)
            if not self._send_packet(packet):
                log.error("TX failed at packet %d/%d", seq + 1, total_chunks)
                return False

            time.sleep(INTER_PACKET_DELAY)

            # Interleave sensor packet
            self._maybe_send_sensor()

        log.info("TX image %d complete", image_id)
        return True

    def transmit_sensor_only(self) -> bool:
        """Send a single sensor packet (for telemetry-only mode)."""
        if self._sensor_reader is None:
            log.warning("No sensor reader configured")
            return False

        sensor_data = self._sensor_reader()
        packet = build_sensor_packet(sensor_data)
        return self._send_packet(packet)


def transmit_images_with_telemetry(
    serial_port,
    images: list,
    sensor_reader: Optional[Callable[[], dict]] = None,
    inter_image_delay: float = 0.5,
) -> int:
    """
    Transmit multiple images with interleaved sensor telemetry.

    Args:
        serial_port: Open serial.Serial instance.
        images: List of JPEG byte arrays.
        sensor_reader: Callable returning sensor data dict.
        inter_image_delay: Seconds to wait between images.

    Returns:
        Number of successfully transmitted images.
    """
    tx = XBeeTransmitter(serial_port, sensor_reader)
    success_count = 0

    for idx, img_data in enumerate(images):
        if not img_data:
            log.warning("Skipping empty image %d", idx)
            continue

        if tx.transmit_image(img_data, idx):
            success_count += 1

        # Brief pause between images
        if idx < len(images) - 1:
            time.sleep(inter_image_delay)

    return success_count
