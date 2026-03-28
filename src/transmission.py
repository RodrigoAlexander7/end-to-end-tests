"""
CanSat Transmission - XBee Pro S1 transmission with real-time sensor telemetry.

Architecture:
- Sensor telemetry runs in a background thread, sending every ~500ms
- Image transmission happens in foreground with 10:1 interleaving
- Thread-safe serial access via lock
"""

import time
import logging
import threading
from typing import Callable, Optional

from protocol import (
    build_image_packet,
    build_sensor_packet,
    MAX_PAYLOAD,
    INTER_PACKET_DELAY,
)

log = logging.getLogger("cansat.tx")

# Transmission ratio: N image packets per 1 sensor packet
IMAGE_TO_SENSOR_RATIO = 10

# Real-time telemetry interval (seconds)
TELEMETRY_INTERVAL = 0.5


class XBeeTransmitter:
    """XBee transmission handler with real-time sensor telemetry."""

    def __init__(self, serial_port, sensor_reader: Optional[Callable[[], dict]] = None):
        """
        Args:
            serial_port: Open serial.Serial instance.
            sensor_reader: Callable that returns current sensor data dict.
        """
        self._serial = serial_port
        self._sensor_reader = sensor_reader
        self._packet_count = 0
        self._lock = threading.Lock()

        # Real-time telemetry thread
        self._telemetry_thread = None
        self._telemetry_running = False

    def _send_packet(self, packet: bytes) -> bool:
        """Send a single packet over serial (thread-safe)."""
        try:
            with self._lock:
                self._serial.write(packet)
                self._serial.flush()
                # Mantener el delay DENTRO del lock asegura que la placa XBee 
                # siempre tenga una ventana de tiempo (gap) para empaquetar 
                # y enviar por el aire sin concatenarse con el thread de telemetría.
                time.sleep(INTER_PACKET_DELAY)
            return True
        except Exception as e:
            log.error("TX failed: %s", e)
            return False

    # -------------------------------------------------------------------------
    # Real-time Telemetry
    # -------------------------------------------------------------------------

    def start_telemetry(self):
        """Start background telemetry thread."""
        if self._sensor_reader is None:
            log.warning("No sensor reader, telemetry disabled")
            return

        if self._telemetry_running:
            return

        self._telemetry_running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()
        log.info("Real-time telemetry started (interval=%.1fs)", TELEMETRY_INTERVAL)

    def stop_telemetry(self):
        """Stop background telemetry thread."""
        self._telemetry_running = False
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None
        log.info("Telemetry stopped")

    def _telemetry_loop(self):
        """Background loop that sends sensor packets continuously."""
        while self._telemetry_running:
            try:
                sensor_data = self._sensor_reader()
                packet = build_sensor_packet(sensor_data)
                if self._send_packet(packet):
                    log.debug("TX telemetry (t=%d)", sensor_data.get("timestamp", 0))
            except Exception as e:
                log.error("Telemetry error: %s", e)

            time.sleep(TELEMETRY_INTERVAL)

    # -------------------------------------------------------------------------
    # Image Transmission (with interleaved telemetry)
    # -------------------------------------------------------------------------

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
                log.debug("TX sensor (interleaved, t=%d)", sensor_data.get("timestamp", 0))

    def transmit_image(self, image_data: bytes, image_id: int) -> bool:
        """
        Transmit a JPEG image with interleaved sensor packets.

        Args:
            image_data: Raw JPEG bytes.
            image_id: Image identifier (0-254).

        Returns:
            True if all packets sent successfully.
        """
        total_chunks = (len(image_data) + MAX_PAYLOAD - 1) // MAX_PAYLOAD
        log.info("TX image %d: %d bytes, %d packets", image_id, len(image_data), total_chunks)

        t_start = time.perf_counter()
        sensor_packets = 0

        for seq in range(total_chunks):
            offset = seq * MAX_PAYLOAD
            chunk = image_data[offset : offset + MAX_PAYLOAD]

            packet = build_image_packet(image_id, seq, total_chunks, chunk)
            if not self._send_packet(packet):
                log.error("TX failed at packet %d/%d", seq + 1, total_chunks)
                return False

            # Track sensor packets sent during image
            old_count = self._packet_count
            self._maybe_send_sensor()
            if self._packet_count < old_count:  # Reset means sensor was sent
                sensor_packets += 1

        t_elapsed = time.perf_counter() - t_start
        throughput = len(image_data) / t_elapsed if t_elapsed > 0 else 0
        avg_pkt_time = (t_elapsed / total_chunks * 1000) if total_chunks > 0 else 0

        log.info("[TIMING] Image %d: %.2fs, %.1f B/s, %.1fms/pkt, %d sensor pkts",
                 image_id, t_elapsed, throughput, avg_pkt_time, sensor_packets)
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
    realtime_telemetry: bool = True,
) -> int:
    """
    Transmit multiple images with real-time sensor telemetry.

    Args:
        serial_port: Open serial.Serial instance.
        images: List of JPEG byte arrays.
        sensor_reader: Callable returning sensor data dict.
        inter_image_delay: Seconds to wait between images.
        realtime_telemetry: If True, sends telemetry continuously in background.

    Returns:
        Number of successfully transmitted images.
    """
    tx = XBeeTransmitter(serial_port, sensor_reader)

    # Start real-time telemetry in background
    if realtime_telemetry:
        tx.start_telemetry()

    success_count = 0
    try:
        for idx, img_data in enumerate(images):
            if not img_data:
                log.warning("Skipping empty image %d", idx)
                continue

            if tx.transmit_image(img_data, idx):
                success_count += 1

            if idx < len(images) - 1:
                time.sleep(inter_image_delay)
    finally:
        tx.stop_telemetry()

    return success_count


def stream_telemetry_only(
    serial_port,
    sensor_reader: Callable[[], dict],
    duration: float = None,
    interval: float = TELEMETRY_INTERVAL,
):
    """
    Stream only sensor telemetry (no images).

    Args:
        serial_port: Open serial.Serial instance.
        sensor_reader: Callable returning sensor data dict.
        duration: How long to stream (None = forever until Ctrl+C).
        interval: Seconds between packets.
    """
    tx = XBeeTransmitter(serial_port, sensor_reader)
    log.info("Streaming telemetry only (interval=%.2fs)", interval)

    start = time.time()
    count = 0
    try:
        while duration is None or (time.time() - start) < duration:
            if tx.transmit_sensor_only():
                count += 1
                if count % 10 == 0:
                    log.info("Sent %d telemetry packets", count)
            time.sleep(interval)
    except KeyboardInterrupt:
        pass

    log.info("Telemetry stream ended: %d packets sent", count)
