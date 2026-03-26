#!/usr/bin/env python3
"""
CanSat Receiver - XBee Image Reception

Listens on a serial port for incoming image packets from the cansat_pipeline,
verifies CRC-16 integrity, reassembles chunked packets, and saves the final JPEG images.
"""

import sys
import time
import struct
import logging
import argparse
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cansat_receiver")

# ---------------------------------------------------------------------------
# CRC-16 (CCITT) for packet integrity (matches tx script)
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
# Receiver Implementation
# ---------------------------------------------------------------------------

SYNC_BYTES = b"\xAA\x55"
HEADER_SIZE = 9
CRC_SIZE = 2

class ImageReceiver:
    def __init__(self, output_dir: str):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        # Dictionary to hold images currently being received.
        # Key is img_id, Value is a dict with details and received chunks
        self.images_in_progress = {}

    def receive(self, serial_port):
        log.info(f"Listening on {serial_port.portstr} at {serial_port.baudrate} baud...")
        buffer = b""
        
        while True:
            # Read whatever data is currently available in the serial buffer
            if serial_port.in_waiting > 0:
                buffer += serial_port.read(serial_port.in_waiting)
            else:
                time.sleep(0.01)
                continue
                
            # Try to process as many full packets as possible from the buffer
            while len(buffer) >= HEADER_SIZE:
                # Find SYNC bytes
                sync_pos = buffer.find(SYNC_BYTES)
                if sync_pos == -1:
                    # Sync missing entirely. Discard all but the last byte just in case 
                    # it is the first byte of a SYNC sequence (`\xAA`)
                    buffer = buffer[-1:]
                    break
                
                # Fast forward to the SYNC sequence
                if sync_pos > 0:
                    buffer = buffer[sync_pos:]
                    
                if len(buffer) < HEADER_SIZE:
                    break  # Wait for the rest of the header
                    
                # Parse header
                # Format: >2sBHHH 
                # SYNC_BYTES(2), img_id(1), seq(2), total(2), pkt_len(2)
                _, img_id, seq, total, pkt_len = struct.unpack(">2sBHHH", buffer[:HEADER_SIZE])
                
                total_packet_size = HEADER_SIZE + pkt_len + CRC_SIZE
                
                if len(buffer) < total_packet_size:
                    # Header is intact, but we need more data for this packet's payload/CRC
                    break
                    
                # Extract the full packet from the buffer
                packet = buffer[:total_packet_size]
                buffer = buffer[total_packet_size:]
                
                # Check the integrity of the packet using CRC-16.
                # The transmitter computes CRC over everything *after* the SYNC bytes
                header_no_sync = packet[2:HEADER_SIZE]
                payload = packet[HEADER_SIZE:HEADER_SIZE+pkt_len]
                received_crc = struct.unpack(">H", packet[-CRC_SIZE:])[0]
                
                computed_crc = crc16(header_no_sync + payload)
                if received_crc != computed_crc:
                    log.warning(f"CRC mismatch for image {img_id} packet {seq}! Got {received_crc}, Expected {computed_crc}")
                    continue
                    
                self.handle_packet(img_id, seq, total, payload)

    def handle_packet(self, img_id: int, seq: int, total: int, payload: bytes):
        if img_id not in self.images_in_progress:
            log.info(f"Receiving new image {img_id} ({total} packets expected)...")
            self.images_in_progress[img_id] = {
                'chunks': {},
                'total': total,
                'last_update': time.time()
            }
            
        img_state = self.images_in_progress[img_id]
        
        # Avoid duplicate packets if they happen to be resent
        if seq not in img_state['chunks']:
            img_state['chunks'][seq] = payload
            img_state['last_update'] = time.time()
            
            # Periodically report progress to the console
            chunks_received = len(img_state['chunks'])
            if chunks_received % max(1, total // 10) == 0 or chunks_received == total:
                log.info(f"Image {img_id}: {chunks_received}/{total} packets received ({(chunks_received/total)*100:.1f}%)")
                
        # Check if the image acquisition is complete
        if len(img_state['chunks']) == img_state['total']:
            self.save_image(img_id, img_state)
            del self.images_in_progress[img_id]

    def save_image(self, img_id: int, img_state: dict):
        log.info(f"Image {img_id} complete! Reassembling and saving...")
        
        # Sort chunks by sequence number and concatenate
        chunks = img_state['chunks']
        image_data = b""
        for seq in range(img_state['total']):
            if seq in chunks:
                image_data += chunks[seq]
            else:
                log.error(f"Missing chunk {seq} for image {img_id}! Cannot save corrupted image.")
                return
                
        # Write to disk
        out_path = self.output_dir / f"received_image_{img_id:03d}.jpg"
        with open(out_path, "wb") as f:
            f.write(image_data)
            
        log.info(f"Saved successfully to {out_path} ({len(image_data)} bytes)")


def main():
    parser = argparse.ArgumentParser(description="CanSat Image Receiver")
    parser.add_argument("--serial-port", type=str, required=True, help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("--baudrate", type=int, default=9600, help="Serial baudrate (must match transmitter)")
    parser.add_argument("--output-dir", type=str, default="received_images", help="Directory to save received images")
    
    args = parser.parse_args()
    
    try:
        import serial
    except ImportError:
        log.error("pyserial is not installed. Please try running `pip install pyserial`")
        sys.exit(1)
    
    receiver = ImageReceiver(args.output_dir)
    
    try:
        ser = serial.Serial(
            port=args.serial_port,
            baudrate=args.baudrate,
            timeout=1,
        )
    except Exception as e:
        log.error(f"Failed to open serial port {args.serial_port}: {e}")
        sys.exit(1)
        
    try:
        receiver.receive(ser)
    except KeyboardInterrupt:
        log.info("Receiver stopped manually via KeyboardInterrupt.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
