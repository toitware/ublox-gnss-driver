// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

// Driver for Max M8 GPS module.

import monitor
import serial
import math
import reader
import writer
import ubx_message
import gnss_location show GnssLocation
import location show Location

import .reader

I2C_ADDRESS ::= 0x42

class Driver:
  static METER_TO_MILLIMETER ::= 1000
  static METER_TO_CENTIMETER ::= 100
  static COORDINATE_FACTOR/float ::= 10_000_000.0

  time_to_first_fix_/Duration := Duration
  waiters_ := []

  location_ /GnssLocation? := null
  adapter_ /Adapter_
  runner_ /Task_? := null

  constructor reader/reader.Reader writer --auto_run/bool=true:
    adapter_ = Adapter_ reader writer

    if auto_run: task --background:: run

  time_to_first_fix: return time_to_first_fix_

  location -> GnssLocation?:
    return location_

  location --blocking -> GnssLocation:
    latch := monitor.Latch
    waiters_.add latch
    return latch.get

  run:
    assert: not runner_
    adapter_.flush
    start_periodic_nav_packets_
    runner_ = task::
      while true:
        message := adapter_.next_message
        if message is ubx_message.NavStatus:
          process_nav_status_ message as ubx_message.NavStatus
        else if message is ubx_message.NavPvt:
          process_nav_pvt_ message as ubx_message.NavPvt

  reset:
    adapter_.reset

  close:
    if runner_:
      runner_.cancel
      runner_ = null

  process_nav_status_ message/ubx_message.NavStatus:
    if time_to_first_fix_.in_ns != 0: return

    time_to_first_fix_ = Duration --ms=message.time_to_first_fix

  process_nav_pvt_ message/ubx_message.NavPvt:
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

  start_periodic_nav_packets_:
    set_message_rate_ ubx_message.Message.NAV ubx_message.NavStatus.ID 1
    set_message_rate_ ubx_message.Message.NAV ubx_message.NavPvt.ID 1

  set_message_rate_ class_id message_id rate:
    adapter_.send_packet (ubx_message.CfgMsg.message_rate --msg_class=class_id --msg_id=message_id --rate=rate).to_byte_array

class Adapter_:
  static STREAM_DELAY_ ::= Duration --ms=1

  raw_reader_/reader.Reader
  reader_ /reader.BufferedReader
  writer_ /writer.Writer

  constructor .raw_reader_ raw_writer:
    reader_ = reader.BufferedReader raw_reader_
    writer_ = writer.Writer raw_writer

  flush:
    // Flush all data up to this point.
    wait_until_receiver_available_

  reset:
    wait_until_receiver_available_
    // Reset and reload configuration (cold boot + reboot of processes).
    send_packet (ubx_message.CfgRst --reset_mode=1).to_byte_array
    // Wait for the reload to take effect, before flushing stale data.
    // This was tested with 10ms, so using 50ms.
    sleep --ms=50
    flush

  send_packet bytes/ByteArray:
    writer_.write bytes
    sleep STREAM_DELAY_

  send_ubx message/ubx_message.Message:
    writer_.write message.to_byte_array
    sleep STREAM_DELAY_

  next_message -> ubx_message.Message:
    while true:
      peek ::= reader_.byte 0
      if peek == 0xb5: // UBX protocol
        start ::= Time.now
        return ubx_message.Message.from_reader reader_
      // Go to next byte.
      reader_.skip 1

  wait_until_receiver_available_:
    // Block until we can read from the device.
    first ::= raw_reader_.read

    // Consume all data from the device before continuing (without blocking).
    while true:
      e := catch:
        with_timeout --ms=0:
          raw_reader_.read
      if e: return
