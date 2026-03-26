"""
CanSat Protocol - Packet building and CRC-16 for XBee transmission.

Packet Types:
- IMAGE (0x00-0xFE): Image chunk packets with sequence/total
- SENSOR (0xFF): Sensor telemetry packets
"""

import struct

# Sync bytes for packet detection
SYNC_BYTES = b"\xaa\x55"

# Packet type identifiers
PACKET_TYPE_IMAGE = 0x00   # 0x00-0xFE reserved for image IDs
PACKET_TYPE_SENSOR = 0xFF  # Sensor telemetry

# Size limits
MAX_PAYLOAD = 200  # Conservative for XBee Pro S1
HEADER_SIZE = 9    # sync(2) + type(1) + seq(2) + total(2) + len(2)
CRC_SIZE = 2

# Timing
INTER_PACKET_DELAY = 0.05  # 50ms between packets

# ---------------------------------------------------------------------------
# CRC-16 (CCITT)
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
# Packet Building
# ---------------------------------------------------------------------------


def build_image_packet(image_id: int, seq: int, total: int, payload: bytes) -> bytes:
    """
    Build an image chunk packet.

    Format: [SYNC(2)] [IMG_ID(1)] [SEQ(2)] [TOTAL(2)] [LEN(2)] [PAYLOAD] [CRC(2)]
    """
    header = struct.pack(
        ">2sBHHH",
        SYNC_BYTES,
        image_id & 0xFF,
        seq,
        total,
        len(payload),
    )
    crc = crc16(header[2:] + payload)
    return header + payload + struct.pack(">H", crc)


def build_sensor_packet(sensor_data: dict) -> bytes:
    """
    Build a sensor telemetry packet.

    Format: [SYNC(2)] [TYPE=0xFF(1)] [SEQ(2)] [TOTAL(2)] [LEN(2)] [PAYLOAD] [CRC(2)]

    Payload format (fixed 32 bytes):
        - mag_x, mag_y, mag_z: 3x float16 (6 bytes)
        - temp, pressure, altitude: 3x float16 (6 bytes)
        - acc_x, acc_y, acc_z: 3x float16 (6 bytes)
        - gyro_x, gyro_y, gyro_z: 3x float16 (6 bytes)
        - voltage, current, power: 3x float16 (6 bytes)
        - timestamp: uint16 (2 bytes)
    Total: 32 bytes
    """
    import numpy as np

    # Pack sensor values as float16 for compactness
    values = [
        sensor_data.get("mag_x", 0.0),
        sensor_data.get("mag_y", 0.0),
        sensor_data.get("mag_z", 0.0),
        sensor_data.get("temp", 0.0),
        sensor_data.get("pressure", 0.0),
        sensor_data.get("altitude", 0.0),
        sensor_data.get("acc_x", 0.0),
        sensor_data.get("acc_y", 0.0),
        sensor_data.get("acc_z", 0.0),
        sensor_data.get("gyro_x", 0.0),
        sensor_data.get("gyro_y", 0.0),
        sensor_data.get("gyro_z", 0.0),
        sensor_data.get("voltage", 0.0),
        sensor_data.get("current", 0.0),
        sensor_data.get("power", 0.0),
    ]

    # Convert to float16 bytes
    float_data = np.array(values, dtype=np.float16).tobytes()

    # Add timestamp (seconds since start, mod 65536)
    timestamp = int(sensor_data.get("timestamp", 0)) % 65536
    payload = float_data + struct.pack(">H", timestamp)

    # Build packet with type=0xFF, seq=0, total=1
    header = struct.pack(
        ">2sBHHH",
        SYNC_BYTES,
        PACKET_TYPE_SENSOR,
        0,  # seq (not used for sensor)
        1,  # total (single packet)
        len(payload),
    )
    crc = crc16(header[2:] + payload)
    return header + payload + struct.pack(">H", crc)


def parse_sensor_payload(payload: bytes) -> dict:
    """Parse sensor payload back to dictionary (for receiver)."""
    import numpy as np

    if len(payload) < 32:
        return {}

    float_data = np.frombuffer(payload[:30], dtype=np.float16)
    timestamp = struct.unpack(">H", payload[30:32])[0]

    return {
        "mag_x": float(float_data[0]),
        "mag_y": float(float_data[1]),
        "mag_z": float(float_data[2]),
        "temp": float(float_data[3]),
        "pressure": float(float_data[4]),
        "altitude": float(float_data[5]),
        "acc_x": float(float_data[6]),
        "acc_y": float(float_data[7]),
        "acc_z": float(float_data[8]),
        "gyro_x": float(float_data[9]),
        "gyro_y": float(float_data[10]),
        "gyro_z": float(float_data[11]),
        "voltage": float(float_data[12]),
        "current": float(float_data[13]),
        "power": float(float_data[14]),
        "timestamp": timestamp,
    }
