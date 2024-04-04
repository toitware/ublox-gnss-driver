// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import io
import serial

/**
Helper class to create a writer from a $serial.Device. Can be used when connecting
  to the GNSS chip using I2C or SPI.
*/
class Writer extends io.Writer:
  device_/serial.Device

  constructor .device_:

  try-write_ data/io.Data from/int=0 to/int=data.byte-size:
    if from != 0 or to != data.byte-size: data = data.byte-slice from to
    if data is ByteArray:
      device_.write (data as ByteArray)
    else:
      bytes := ByteArray (to - from)
      data.write-to-byte-array bytes --at=0 from to
      device_.write bytes
    return to - from
