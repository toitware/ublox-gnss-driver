// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

// Driver for Max M8 GPS module.

import .diagnostics
import gnss_location show GnssLocation
import io
import location show Location
import log
import math
import monitor
import reader as old-reader
import serial
import ubx_message

import .reader
import .writer

I2C_ADDRESS ::= 0x42

class Driver:
  static METER_TO_MILLIMETER ::= 1000
  static METER_TO_CENTIMETER ::= 100
  static COORDINATE_FACTOR /float ::= 10_000_000.0

  static QUALITY_SAT_COUNT_ ::= 4

  time_to_first_fix_ /Duration := Duration.ZERO
  waiters_ := []

  diagnostics_ /Diagnostics := Diagnostics --known_satellites=0 --satellites_in_view=0 --signal_quality=0.0 --time_to_first_fix=Duration.ZERO
  location_ /GnssLocation? := null
  adapter_ /Adapter_
  runner_ /Task? := null

  /**
  Creates a new driver object.

  The $reader should be an $io.Reader, but $old-reader.Reader objects are
    still supported for backwards compatibility. Support for $old-reader.Reader
    is deprecated and will be removed in a future release.
  Use $Reader to create an $io.Reader from a $serial.Device.

  The $writer should be an $io.Writer, but "old-style" writers are still
    supported for backwards compatibility. Support for "old-style" writers is
    deprecated and will be removed in a future release.
  Use $Writer to create an $io.Writer from a $serial.Device.
  */
  constructor reader writer logger=log.default --auto_run/bool=true:
    if reader is old-reader.Reader:
      reader = io.Reader.adapt reader

    if writer is not io.Writer:
      writer = io.Writer.adapt writer

    adapter_ = Adapter_ reader writer logger

    if auto_run: task --background:: run

  time_to_first_fix -> Duration: return time_to_first_fix_

  diagnostics -> Diagnostics: return diagnostics_

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
        else if message is ubx_message.NavSat:
          process_nav_sat_ message as ubx_message.NavSat

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
        message.height_msl.to_float / METER_TO_MILLIMETER
        message.utc_time
        message.horizontal_acc.to_float / METER_TO_MILLIMETER
        message.vertical_acc.to_float / METER_TO_MILLIMETER
      waiters := waiters_
      waiters_ = []
      waiters.do: it.set location_

  process_nav_sat_ message/ubx_message.NavSat:
    cnos ::= []
    satellite_count ::= message.num_svs
    satellite_count.repeat: | index |
      satellite_data ::= message.satellite_data index
      cnos.add satellite_data.cno

    cnos.sort --in_place: | a b | b - a
    n ::= min cnos.size QUALITY_SAT_COUNT_
    sum := 0.0
    n.repeat: sum += cnos[it]
    quality ::= sum / QUALITY_SAT_COUNT_

    satellites_in_view := cnos.reduce --initial=0: | count cno |
      count + (cno > 0 ? 1 : 0)
    known_satellites := satellite_count
    diagnostics_ = Diagnostics
        --time_to_first_fix=time_to_first_fix
        --signal_quality=quality
        --satellites_in_view=satellites_in_view
        --known_satellites=known_satellites

  start_periodic_nav_packets_:
    set_message_rate_ ubx_message.Message.NAV ubx_message.NavStatus.ID 1
    set_message_rate_ ubx_message.Message.NAV ubx_message.NavPvt.ID 1
    set_message_rate_ ubx_message.Message.NAV ubx_message.NavSat.ID 1

  set_message_rate_ class_id message_id rate:
    adapter_.send_packet (ubx_message.CfgMsg.message_rate --msg_class=class_id --msg_id=message_id --rate=rate).to_byte_array

class Adapter_:
  static STREAM_DELAY_ ::= Duration --ms=1

  logger_/log.Logger
  reader_/io.Reader
  writer_/io.Writer

  constructor .reader_ .writer_ .logger_:

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
      peek ::= reader_.peek-byte 0
      if peek == 0xb5: // UBX protocol
        start ::= Time.now
        e := catch: return ubx_message.Message.from_reader reader_
        log.warn "error parsing ubx message" --tags={"error": e}
      // Go to next byte.
      reader_.skip 1

  wait_until_receiver_available_:
    // Block until we can read from the device.
    first ::= reader_.read

    // Consume all data from the device before continuing (without blocking).
    while true:
      e := catch:
        with_timeout --ms=0:
          reader_.read
      if e: return
