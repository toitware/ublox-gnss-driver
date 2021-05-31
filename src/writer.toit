// Copyright (C) 2021 Toitware ApS. All rights reserved.

import serial

/**
Helper class to create a writer from a $serial.Device. Can be used when connecting
  to the GNSS chip using I2C or SPI.
*/
class Writer:
  device_/serial.Device

  constructor .device_:

  write data/ByteArray from/int=0 to/int=data.size:
    if from != 0 or to != data.size: data = data[from..to]
    device_.write data
    return to - from
