// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import serial
import io

/**
Helper class to create an $io.Reader from a $serial.Device. Can be used when connecting
  to the GNSS chip using I2C or SPI.
*/
class Reader extends io.Reader:
  static WAIT-BEFORE-NEXT-READ-ATTEMPT_ ::= Duration --ms=5
  static MAX-BUFFER-SIZE_               ::= 64 // bytes
  static AVAILABLE-BYTES-REGISTER_      ::= 0xFD
  static DATA-STREAM-REGISTER_          ::= 0xFF

  registers_ /serial.Registers

  constructor device/serial.Device:
    registers_ = device.registers

  read_ -> ByteArray?:
    while true:
      bytes ::= read__
      if bytes: return bytes
      sleep WAIT-BEFORE-NEXT-READ-ATTEMPT_

  read__ -> ByteArray?:
    available-bytes ::= registers_.read-u16-be AVAILABLE-BYTES-REGISTER_
    if available-bytes == 0:
      return null

    return registers_.read-bytes
      DATA-STREAM-REGISTER_
      min MAX-BUFFER-SIZE_ available-bytes
