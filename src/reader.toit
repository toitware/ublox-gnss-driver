// Copyright (C) 2021 Toitware ApS. All rights reserved.

// Driver for Max M8 GPS module.

import serial
import reader

/**
Helper class to create a $reader.Reader from a $serial.Device. Can be used when connecting
  to the GNSS chip using I2C or SPI.
*/
class Reader implements reader.Reader:
  static WAIT_BEFORE_NEXT_READ_ATTEMPT_ ::= Duration --ms=5
  static MAX_BUFFER_SIZE_               ::= 64 // bytes
  static AVAILABLE_BYTES_REGISTER_      ::= 0xFD
  static DATA_STREAM_REGISTER_          ::= 0xFF

  registers_ /serial.Registers

  constructor device/serial.Device:
    registers_ = device.registers

  read -> ByteArray?:
    while true:
      bytes ::= read_
      if bytes: return bytes
      sleep WAIT_BEFORE_NEXT_READ_ATTEMPT_

  read_ -> ByteArray?:
    available_bytes ::= registers_.read_u16_be AVAILABLE_BYTES_REGISTER_: 0
    if available_bytes == 0:
      return null

    return registers_.read_bytes
      DATA_STREAM_REGISTER_
      min MAX_BUFFER_SIZE_ available_bytes
