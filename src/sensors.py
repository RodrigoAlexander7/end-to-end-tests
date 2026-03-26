"""
CanSat Sensors - I2C sensor reading for telemetry.

Sensors supported:
- MMC5603: Magnetometer (I2C via adafruit)
- MS5611: Barometer (temperature, pressure, altitude)
- MPU9250: Accelerometer/Gyroscope
- INA226: Power monitor (voltage, current, power)
"""

import time
import logging

log = logging.getLogger("cansat.sensors")

# I2C Addresses
ADDR_MS5611 = 0x77
ADDR_MPU9250 = 0x68
ADDR_INA226 = 0x40

# MS5611 commands
CMD_RESET = 0x1E
CMD_CONVERT_D1 = 0x48
CMD_CONVERT_D2 = 0x50


class SensorHub:
    """Unified sensor reading interface."""

    def __init__(self):
        self._bus = None
        self._mag_sensor = None
        self._ms5611_prom = None
        self._start_time = time.time()
        self._initialized = False

    def initialize(self) -> bool:
        """Initialize all sensors. Returns True if at least one sensor works."""
        try:
            import smbus2 as smbus
            self._bus = smbus.SMBus(1)
        except Exception as e:
            log.error("Failed to open I2C bus: %s", e)
            return False

        ok_count = 0

        # Magnetometer (adafruit)
        try:
            import board
            import busio
            import adafruit_mmc56x3

            i2c = busio.I2C(board.SCL, board.SDA)
            self._mag_sensor = adafruit_mmc56x3.MMC5603(i2c)
            log.info("MMC5603 magnetometer initialized")
            ok_count += 1
        except Exception as e:
            log.warning("MMC5603 not available: %s", e)

        # Barometer MS5611
        try:
            self._bus.write_byte(ADDR_MS5611, CMD_RESET)
            time.sleep(0.01)
            self._ms5611_prom = self._read_ms5611_prom()
            log.info("MS5611 barometer initialized")
            ok_count += 1
        except Exception as e:
            log.warning("MS5611 not available: %s", e)

        # MPU9250 accel/gyro
        try:
            self._bus.write_byte_data(ADDR_MPU9250, 0x6B, 0x00)  # Wake up
            log.info("MPU9250 accel/gyro initialized")
            ok_count += 1
        except Exception as e:
            log.warning("MPU9250 not available: %s", e)

        # INA226 power monitor
        try:
            self._read_ina226()  # Test read
            log.info("INA226 power monitor initialized")
            ok_count += 1
        except Exception as e:
            log.warning("INA226 not available: %s", e)

        self._initialized = ok_count > 0
        log.info("Sensors initialized: %d/4 available", ok_count)
        return self._initialized

    def read_all(self) -> dict:
        """Read all sensors and return a telemetry dictionary."""
        data = {
            "mag_x": 0.0, "mag_y": 0.0, "mag_z": 0.0,
            "temp": 0.0, "pressure": 0.0, "altitude": 0.0,
            "acc_x": 0.0, "acc_y": 0.0, "acc_z": 0.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "voltage": 0.0, "current": 0.0, "power": 0.0,
            "timestamp": int(time.time() - self._start_time),
        }

        if not self._initialized:
            return data

        # Magnetometer
        if self._mag_sensor:
            try:
                mx, my, mz = self._mag_sensor.magnetic
                data["mag_x"], data["mag_y"], data["mag_z"] = mx, my, mz
            except Exception:
                pass

        # Barometer
        if self._ms5611_prom:
            try:
                temp, pres = self._read_ms5611()
                data["temp"] = temp
                data["pressure"] = pres
                data["altitude"] = self._calculate_altitude(pres)
            except Exception:
                pass

        # Accel/Gyro
        try:
            data["acc_x"] = self._read_mpu_raw(0x3B) / 16384.0
            data["acc_y"] = self._read_mpu_raw(0x3D) / 16384.0
            data["acc_z"] = self._read_mpu_raw(0x3F) / 16384.0
            data["gyro_x"] = self._read_mpu_raw(0x43) / 131.0
            data["gyro_y"] = self._read_mpu_raw(0x45) / 131.0
            data["gyro_z"] = self._read_mpu_raw(0x47) / 131.0
        except Exception:
            pass

        # Power monitor
        try:
            v, i, p = self._read_ina226()
            data["voltage"], data["current"], data["power"] = v, i, p
        except Exception:
            pass

        return data

    # -------------------------------------------------------------------------
    # MS5611 Barometer
    # -------------------------------------------------------------------------

    def _read_ms5611_prom(self) -> list:
        prom = []
        for i in range(8):
            d = self._bus.read_i2c_block_data(ADDR_MS5611, 0xA0 + (i * 2), 2)
            prom.append((d[0] << 8) | d[1])
        return prom

    def _read_ms5611_adc(self, cmd: int) -> int:
        self._bus.write_byte(ADDR_MS5611, cmd)
        time.sleep(0.01)
        d = self._bus.read_i2c_block_data(ADDR_MS5611, 0x00, 3)
        return (d[0] << 16) | (d[1] << 8) | d[2]

    def _read_ms5611(self) -> tuple:
        C = self._ms5611_prom
        D1 = self._read_ms5611_adc(CMD_CONVERT_D1)
        D2 = self._read_ms5611_adc(CMD_CONVERT_D2)

        dT = D2 - C[5] * 256
        TEMP = 2000 + dT * C[6] / 8388608
        OFF = C[2] * 65536 + (C[4] * dT) / 128
        SENS = C[1] * 32768 + (C[3] * dT) / 256
        P = (D1 * SENS / 2097152 - OFF) / 32768

        return TEMP / 100.0, P / 100.0

    @staticmethod
    def _calculate_altitude(pressure: float, sea_level: float = 1013.25) -> float:
        return 44330.0 * (1.0 - (pressure / sea_level) ** (1 / 5.255))

    # -------------------------------------------------------------------------
    # MPU9250 Accel/Gyro
    # -------------------------------------------------------------------------

    def _read_mpu_raw(self, reg: int) -> int:
        high = self._bus.read_byte_data(ADDR_MPU9250, reg)
        low = self._bus.read_byte_data(ADDR_MPU9250, reg + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    # -------------------------------------------------------------------------
    # INA226 Power Monitor
    # -------------------------------------------------------------------------

    def _read_ina226(self, r_shunt: float = 0.1) -> tuple:
        # Bus voltage
        d_v = self._bus.read_i2c_block_data(ADDR_INA226, 0x02, 2)
        raw_v = (d_v[0] << 8) | d_v[1]
        voltage = raw_v * 0.00125

        # Shunt voltage -> current
        d_s = self._bus.read_i2c_block_data(ADDR_INA226, 0x01, 2)
        raw_s = (d_s[0] << 8) | d_s[1]
        if raw_s > 32767:
            raw_s -= 65536
        shunt_v = raw_s * 0.0000025
        current_ma = (shunt_v / r_shunt) * 1000
        power_mw = voltage * current_ma

        return voltage, current_ma, power_mw

    def close(self):
        """Release resources."""
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
        self._initialized = False
