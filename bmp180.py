'''Micropython driver for Bosch BMP180.

Tested on esp8266-20191220-v1.12.

REPL use sample:
>    from machine import I2C, Pin
>    BMP180_I2C = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
>    BMP180 = Bmp180(BMP180_I2C)
>    print(BMP180.read_temperature())
>    print(BMP180.read_pressure())

'''

import ustruct
import utime

# oversampling modes
OSS_LOW_POWER = 0x00
OSS_STANDARD = 0x01
OSS_HIGH_RES = 0x02
OSS_ULTRA_HIGH_RES = 0x03

# general constants
I2C_ADDRESS = 0x77
CHIP_ID = 0x55
BYTE_ORDER = 'big'

# register addresses
CHIP_ID_REG = 0xD0
SOFT_RESET_REG = 0xE0
CONTROL_REG = 0xF4
MSB_REG = 0xF6
LSB_REG = 0xF7

# commands
READ_TEMPERATURE = b'\x2e'
READ_PRESSURE = 0x34
SOFT_RESET = b'\xb6'

# conversion times in microseconds
TEMPERATURE_CONVERSION_TIME = 4500
PRESSURE_CONVERSION_TIMES = (
    4500,
    7500,
    13500,
    25500
)


class CalibrationData():
    '''BMP180 calibration data.

    '''
    __CALIB_DATA = (
        ('ac1', 0xAA, '>h'),
        ('ac2', 0xAC, '>h'),
        ('ac3', 0xAE, '>h'),
        ('ac4', 0xB0, '>H'),
        ('ac5', 0xB2, '>H'),
        ('ac6', 0xB4, '>H'),
        ('b1', 0xB6, '>h'),
        ('b2', 0xB8, '>h'),
        ('mb', 0xBA, '>h'),
        ('mc', 0xBC, '>h'),
        ('md', 0xBE, '>h')
    )

    def __init__(self):
        '''Set all the coefficiants to zero.

        '''
        for attr, _, _ in self.__CALIB_DATA:
            setattr(self, attr, 0)

    def __iter__(self):
        '''Iterate through calibration data tuples.

        Each calibration data tuple consists of:
        - name
        - register address
        - format descriptor

        '''
        for entry in self.__CALIB_DATA:
            yield entry


class Bmp180():
    ''' BMP180 class.

    Allows reading of temperature and pressure using the Bosch BMP180 sensor.
    On initialization chip id is verified, calibration data is read.

    Attributes:
        oss (int): Oversampling setting from 0x00 to 0x03.
        cal (CalibrationData): Calibration data read from E2PROM

    '''
    def __init__(self, i2c, oss=OSS_STANDARD):
        '''Bmp180 initalization.

        Args:
            i2c (I2C): Instance of I2C with proper clock and data pins.
            oss (int): Oversampling value, 0x00 to 0x03.

        '''
        self.__i2c = i2c
        self.oss = oss
        self.cal = CalibrationData()
        self.__check_chip()
        self.__read_calibration_data()

    def _read(self, register, byte_count=1, data_format=None, check_sco=False):
        '''Read data over i2c returning an integer value.

        Args:
            register (int): Register address.
            byte_count (int): Number of bytes to read.
            data_format (str): Optional data format for conversion to interger.
            check_sco (bool): Verify that conversion is complete.

        Returns:
            int: Value read from sensor.

        '''
        if check_sco:
            assert (self._read(CONTROL_REG, 1) & 0b100000) == 0

        raw_bytes = self.__i2c.readfrom_mem(I2C_ADDRESS, register, byte_count)

        if data_format:
            value = ustruct.unpack(data_format, raw_bytes)[0]
        else:
            value = int.from_bytes(raw_bytes, byte_count, BYTE_ORDER)

        return value

    def _write(self, register, data):
        '''Write data over i2c.

        Args:
            register (int): Register address.
            data (bytes): Data to write.

        '''
        self.__i2c.writeto_mem(I2C_ADDRESS, register, data)

    def __check_chip(self):
        '''Verify the chip identifier.

        '''
        assert self.chip_id == CHIP_ID

    def __read_calibration_data(self):
        '''Read and cache calibration data from sensor.

        '''
        for name, addr, fmt in self.cal:
            setattr(self.cal, name, self._read(addr, 2, fmt))

    def __read_raw_temperature(self):
        '''Read uncompensated temperature value.

        Returns:
            int: Uncompensated temperature value.

        '''
        self._write(CONTROL_REG, READ_TEMPERATURE)
        utime.sleep_us(TEMPERATURE_CONVERSION_TIME)
        return self._read(MSB_REG, 2, check_sco=True)

    def __read_raw_pressure(self):
        '''Read uncompensated pressure value.

        Returns:
            int: Uncompensated pressure value.

        '''
        command = (READ_PRESSURE + (self.oss << 6)).to_bytes(1, BYTE_ORDER)
        self._write(CONTROL_REG, command)
        utime.sleep_us(PRESSURE_CONVERSION_TIMES[self.oss])
        return self._read(MSB_REG, 3, check_sco=True) >> (8 - self.oss)

    def __calculate_temperature(self, raw_temperature):
        '''Calculate real temperature value.

        Local variable nomenclature follows sensor datasheet.

        Args:
            raw_temperature (int): Uncompensated temperature value.

        Returns:
            float: Compensated temperature.

        '''
        x1 = (raw_temperature - self.cal.ac6) * self.cal.ac5 / (2 << 14)
        x2 = self.cal.mc * (2 << 10) / (x1 + self.cal.md)
        b5 = x1 + x2
        return (b5 + 8) / (2 << 3) * 0.1

    def __calculate_pressure(self, raw_pressure):
        '''Calculate real pressure value.

        Local variable nomenclature follows sensor datasheet.

        Args:
            raw_pressure (int): Uncompensated pressure value.

        Returns:
            float: Compensated pressure.

        '''
        raw_temperature = self.__read_raw_temperature()
        x1 = (raw_temperature - self.cal.ac6) * self.cal.ac5 / (2 << 14)
        x2 = self.cal.mc * (2 << 10) / (x1 + self.cal.md)
        b5 = x1 + x2

        b6 = b5 - 4000
        x1 = (self.cal.b2 * (b6 * b6 / (2 << 11))) / (2 << 10)
        x2 = self.cal.ac2 * b6 / (2 << 10)
        x3 = x1 + x2
        b3 = ((int(self.cal.ac1 * 4 + x3) << self.oss) + 2) / 4
        x1 = self.cal.ac3 * b6 / (2 << 12)
        x2 = (self.cal.b1 * (b6 * b6 / (2 << 11))) / (2 << 15)
        x3 = ((x1 + x2) + 2) / (2 << 1)
        b4 = self.cal.ac4 * (x3 + 32768) / (2 << 14)
        b7 = (raw_pressure - b3) * (50000 >> self.oss)
        p = (b7 * 2) / b4 if b7 < 0x80000000 else (b7 / b4) * 2
        x1 = (p / (2 << 7)) * (p / (2 << 7))
        x1 = (x1 * 3038) / (2 << 15)
        x2 = (-7357 * p) / (2 << 15)
        p = p + (x1 + x2 + 3791) / (2 << 3)
        p = p * 0.01
        return p

    @property
    def chip_id(self):
        '''int: Chip identifier.

        '''
        return self._read(CHIP_ID_REG, 1)

    def read_temperature(self):
        '''Read temperature from sensor.

        Returns:
            float: Temperature value in degrees celsius.

        '''
        raw_temp = self.__read_raw_temperature()
        real_temp = self.__calculate_temperature(raw_temp)
        return real_temp

    def read_pressure(self):
        '''Read pressure from sensor.

        Returns:
            float: Pressure value in hectopascals.

        '''
        raw_pressure = self.__read_raw_pressure()
        real_pressure = self.__calculate_pressure(raw_pressure)
        return real_pressure

    def recalibrate(self):
        '''Refresh calibration data from sensor.

        '''
        self.__read_calibration_data()

    def reset(self):
        '''Perform sensor soft reset

        '''
        self._write(SOFT_RESET_REG, SOFT_RESET)


if __name__ == '__main__':
    '''BMP180 class use sample

    '''
    from machine import I2C, Pin
    BMP180_I2C = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
    BMP180 = Bmp180(BMP180_I2C)
    print(BMP180.read_temperature())
    print(BMP180.read_pressure())
