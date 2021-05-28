// Copyright (C) 2019 Toitware ApS. All rights reserved.

// Driver for Max M8 GPS module.

import monitor
import serial
import reader show *
import encoding.ubjson as ubjson
import math

import ubx.ubx_message
import gnss_location show GnssLocation
import location show Location

class MaxM8Driver:
  static METER_TO_MILLIMETER ::= 1000
  static METER_TO_CENTIMETER ::= 100
  static COORDINATE_FACTOR/float ::= 10_000_000.0

  time_to_first_fix_ := 0
  waiters_ := []

  location_ /GnssLocation? := null
  driver_ /MaxM8
  runner_ /Task_? := null

  constructor device/serial.Device:
    driver_ = MaxM8 device

  time_to_first_fix: return time_to_first_fix_

  location -> GnssLocation?:
    return location_

  location --blocking -> GnssLocation:
    latch := monitor.Latch
    waiters_.add latch
    return latch.get

  start:
    driver_.flush
    start_periodic_nav_packets
    runner_ = task::
      while true:
        message := driver_.next_message
        if message is ubx.NavStatus:
          process_nav_status_ message as ubx.NavStatus
        else if message is ubx.NavPvt:
          process_nav_pvt_ message as ubx.NavPvt

  process_nav_status_ message/ubx.NavStatus:
    if time_to_first_fix_ != 0: return

    time_to_first_fix_ = message.time_to_first_fix

  process_nav_pvt_ message/ubx.NavPvt:
    if message.is_gnss_fix:
      location_ = GnssLocation
        Location message.lat / COORDINATE_FACTOR message.lon / COORDINATE_FACTOR
        message.height_msl / METER_TO_MILLIMETER
        message.utc_time
        message.horizontal_acc.to_float / METER_TO_MILLIMETER
        message.vertical_acc.to_float / METER_TO_MILLIMETER
      waiters := waiters_
      waiters_ = []
      waiters.do: it.set location_

  stop:
    if runner_:
      runner_.cancel
      runner_ = null

  start_periodic_nav_packets:
    set_message_rate_ ubx.Message.NAV ubx.NavStatus.ID 2
    set_message_rate_ ubx.Message.NAV ubx.NavPvt.ID 2

  set_message_rate_ class_id message_id rate:
    driver_.send_packet (ubx.CfgMsg.message_rate --msg_class=class_id --msg_id=message_id --rate=rate).to_byte_array

// Low-level MaxM8 implementation configuration.
MAX_BUFFER_SIZE_                ::= 64 // bytes
WAIT_BEFORE_NEXT_READ_ATTEMPT_  ::= 5 // ms
STREAM_DELAY_                   ::= 1 // ms

class MaxM8:
  device_ /serial.Device? := null
  reader_ /BufferedReader := ?
  raw_reader_ /MaxM8Reader? := null

  constructor .device_:
    raw_reader_ = MaxM8Reader device_.registers
    reader_ = BufferedReader raw_reader_

  constructor.reader reader:
    reader_ = BufferedReader reader

  flush:
    // Flush all data up to this point.
    wait_until_receiver_available_

  reset:
    wait_until_receiver_available_
    // Reset and reload configuration (cold boot + reboot of processes).
    send_packet (ubx.CfgRst --reset_mode=1).to_byte_array
    // Wait for the reload to take effect, before flushing stale data.
    // This was tested with 10ms, so using 50ms.
    sleep --ms=50
    flush

  send_packet bytes/ByteArray:
    device_.write bytes
    sleep --ms=STREAM_DELAY_

  send_ubx message/ubx.Message:
    device_.write message.to_byte_array
    sleep --ms=STREAM_DELAY_

  next_message -> ubx.Message:
    while true:
      peek ::= reader_.byte 0
      if peek == 0xb5: // UBX protocol
        start ::= Time.now
        return ubx.Message.from_reader reader_
      // Go to next byte.
      reader_.skip 1

  wait_until_receiver_available_:
    // Block until we can read from the device.
    first ::= raw_reader_.read
    // Consume all data from the device before continuing (without blocking).
    while true:
      data ::= raw_reader_.read_
      if data == null: break

class MaxM8Reader implements Reader:
  static AVAILABLE_BYTES_REGISTER_ ::= 0xFD
  static DATA_STREAM_REGISTER_     ::= 0xFF

  registers_ /serial.Registers

  constructor .registers_/serial.Registers:

  read -> ByteArray?:
    while true:
      bytes ::= read_
      if bytes: return bytes
      sleep --ms=WAIT_BEFORE_NEXT_READ_ATTEMPT_

  read_ -> ByteArray?:
    available_bytes ::= registers_.read_u16_be AVAILABLE_BYTES_REGISTER_: 0
    if available_bytes == 0:
      return null

    return registers_.read_bytes
      DATA_STREAM_REGISTER_
      min MAX_BUFFER_SIZE_ available_bytes
