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
  static WAIT_BEFORE_NEXT_READ_ATTEMPT_ ::= Duration --ms=5
  static MAX_BUFFER_SIZE_               ::= 64 // bytes
  static AVAILABLE_BYTES_REGISTER_      ::= 0xFD
  static DATA_STREAM_REGISTER_          ::= 0xFF

  registers_ /serial.Registers

  constructor device/serial.Device:
    registers_ = device.registers

  read_ -> ByteArray?:
    while true:
      bytes ::= read__
      if bytes: return bytes
      sleep WAIT_BEFORE_NEXT_READ_ATTEMPT_

  read__ -> ByteArray?:
    available_bytes ::= registers_.read_u16_be AVAILABLE_BYTES_REGISTER_
    if available_bytes == 0:
      return null

    return registers_.read_bytes
      DATA_STREAM_REGISTER_
      min MAX_BUFFER_SIZE_ available_bytes
