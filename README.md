# Bmp180

Micropython library for Bosch BMP180 sensor.

## What is it?

* A Micropython library allowing easy retrieval of both temperature and pressure readings. 
* Supports four oversampling modes for pressure.
* Also allows retrieval of chip ip, soft reset and update of cached calibration data.

## How does it look like?

Usage sample with Bmp180 connected to pins 5 (clock) and 4 (data) of ESP8266:
```
from machine import I2C, Pin
BMP180_I2C = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
BMP180 = Bmp180(BMP180_I2C)
print(BMP180.read_temperature())
print(BMP180.read_pressure()
```

Run using Adafruit's ampy:
```
ampy -p /dev/ttyUSB0 -b 115200 run bmp180.py
24.3782
994.73
```

## Anything more?

In case of problems feel free to open an issue or create a pull request.

Official Bosch BMP180 driver in c can be found [HERE](https://github.com/BoschSensortec/BMP180_driver).
