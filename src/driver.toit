// Copyright (C) 2025 Toit Contributors. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

// Driver for Max M8 GPS module.

import .diagnostics
import gnss-location show GnssLocation
import io
import location show Location
import log
import math
import monitor
import reader as old-reader
import serial
import ubx-message

import .reader
import .writer

I2C-ADDRESS ::= 0x42

class Driver:
  static METER-TO-MILLIMETER ::= 1000
  static METER-TO-CENTIMETER ::= 100
  static COORDINATE-FACTOR /float ::= 10_000_000.0

  static QUALITY-SAT-COUNT_ ::= 4

  time-to-first-fix_ /Duration := Duration.ZERO

  // Latches/Mutexes for managing and acknowledging commands
  waiters_ := []
  command-mutex_ := monitor.Mutex      // Used to ensure one command at once
  command-poll-latch_ := monitor.Latch // Used to ensure poll mutex gets the result
  command-cfg-latch_ := monitor.Latch  // Used to ensure cfg mutex gets the result

  diagnostics_ /Diagnostics := Diagnostics --known-satellites=0 --satellites-in-view=0 --signal-quality=0.0 --time-to-first-fix=Duration.ZERO
  location_ /GnssLocation? := null
  adapter_ /Adapter_
  runner_ /Task? := null
  logger_/log.Logger := ?

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
  constructor reader writer logger=log.default --auto-run/bool=true:
    logger_ = logger.with-name "ublox-gnss"

    if reader is old-reader.Reader:
      reader = io.Reader.adapt reader

    if writer is not io.Writer:
      writer = io.Writer.adapt writer

    adapter_ = Adapter_ reader writer logger

    if auto-run: task --background:: run

  time-to-first-fix -> Duration: return time-to-first-fix_

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
    start-periodic-nav-packets_
    runner_ = task::
      while true:
        message := adapter_.next-message
        logger_.debug  "Received: $message"

        if message is ubx-message.AckAck:
          // Message is an ACK-ACK - positive response.
          command-cfg-latch_.set (message as ubx-message.AckAck)
          process-ack-ack-message_ message as ubx-message.AckAck

        else if message is ubx-message.AckNak:
          // Message is an ACK-NACK - negative response. (A command sent didn't work.)
          command-cfg-latch_.set (message as ubx-message.AckNak)
          process-ack-nak-message_ message as ubx-message.AckNak

        else if message is ubx-message.NavStatus:
          process-nav-status_ message as ubx-message.NavStatus
        else if message is ubx-message.NavPvt:
          process-nav-pvt_ message as ubx-message.NavPvt
        else if message is ubx-message.NavSat:
          process-nav-sat_ message as ubx-message.NavSat

  reset:
    adapter_.reset

  close:
    if runner_:
      runner_.cancel
      runner_ = null

  process-ack-nak-message_ message/ubx-message.AckNak:
    logger_.debug "Received AckNak message." --tags={"class": message.class-id-text , "message": message.message-id-text}

  process-ack-ack-message_ message/ubx-message.AckAck:
    logger_.debug "Received AckAck message." --tags={"class": message.class-id-text , "message": message.message-id-text}

  process-nav-status_ message/ubx-message.NavStatus:
    if time-to-first-fix_.in-ns != 0: return

    time-to-first-fix_ = Duration --ms=message.time-to-first-fix

  process-nav-pvt_ message/ubx-message.NavPvt:
    if message.is-gnss-fix:
      location_ = GnssLocation
        Location message.lat / COORDINATE-FACTOR message.lon / COORDINATE-FACTOR
        message.height-msl.to-float / METER-TO-MILLIMETER
        message.utc-time
        message.horizontal-acc.to-float / METER-TO-MILLIMETER
        message.vertical-acc.to-float / METER-TO-MILLIMETER
      waiters := waiters_
      waiters_ = []
      waiters.do: it.set location_

  process-nav-sat_ message/ubx-message.NavSat:
    cnos ::= []
    satellite-count ::= message.num-svs
    satellite-count.repeat: | index |
      satellite-data ::= message.satellite-data index
      cnos.add satellite-data.cno

    cnos.sort --in-place: | a b | b - a
    n ::= min cnos.size QUALITY-SAT-COUNT_
    sum := 0.0
    n.repeat: sum += cnos[it]
    quality ::= sum / QUALITY-SAT-COUNT_

    satellites-in-view := cnos.reduce --initial=0: | count cno |
      count + (cno > 0 ? 1 : 0)
    known-satellites := satellite-count
    diagnostics_ = Diagnostics
        --time-to-first-fix=time-to-first-fix
        --signal-quality=quality
        --satellites-in-view=satellites-in-view
        --known-satellites=known-satellites

  start-periodic-nav-packets_:
    send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavStatus.ID 1
    send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavPvt.ID 1
    send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSat.ID 1

  /**
  Sends a subscription for specific message, and the defined rate.

  Constructs a UBX-CFG message asking for the specified class-id/ message-id
    message type to be sent at the specifid rate.
  */
  send-set-message-rate_ class-id message-id rate:
    logger_.debug "Set Message Rate." --tags={"class": class-id, "message": message-id, "rate": rate}
    message := ubx-message.CfgMsg.message-rate --msg-class=class-id --msg-id=message-id --rate=rate
    send-message-cfg_ message

 /**
  Sends various types of CFG messages, and waits for the response.

  Handles logic of success and failure messages, while not blocking other
    message traffic being handled by the driver.

  Todo: Could possibly collapse the two send-message-cfg/poll functions together.
  */
  send-message-cfg_ message/ubx-message.CfgMsg --return-immediately/bool=false:
    command-mutex_.do:
      if return-immediately:
        adapter_.send-packet message.to-byte-array
        return

      // Reset the latch
      command-cfg-latch_ = monitor.Latch

      //logger_.debug  "Sent request." --tags={"message":"$(message)"}
      adapter_.send-packet message.to-byte-array

      //To do: give a timeout.
      start := Time.monotonic-us
      adapter_.send-packet message.to-byte-array
      response := command-cfg-latch_.get
      duration := Duration --us=(Time.monotonic-us - start)
      logger_.debug  "Message Reponse." --tags={"message":"$(message)","response":"$(response)","ms":(duration.in-ms)}

      if response is ubx-message.AckAck:
        // logger_.debug  "Acknowledged."
      if response is ubx-message.AckNak:
        logger_.error  "NEGATIVE acknowledgement." --tags={"message":"$(message)","response":"$(response)","ms":(duration.in-ms)}


class Adapter_:
  static STREAM-DELAY_ ::= Duration --ms=1

  logger_/log.Logger
  reader_/io.Reader
  writer_/io.Writer

  constructor .reader_ .writer_ .logger_:

  flush:
    // Flush all data up to this point.
    wait-until-receiver-available_

  reset:
    wait-until-receiver-available_
    // Reset and reload configuration (cold boot + reboot of processes).
    send-packet (ubx-message.CfgRst --reset-mode=1).to-byte-array
    // Wait for the reload to take effect, before flushing stale data.
    // This was tested with 10ms, so using 50ms.
    sleep --ms=50
    flush

  send-packet bytes/ByteArray:
    writer_.write bytes
    sleep STREAM-DELAY_

  send-ubx message/ubx-message.Message:
    writer_.write message.to-byte-array
    sleep STREAM-DELAY_

  next-message -> ubx-message.Message:
    while true:
      peek ::= reader_.peek-byte 0
      if peek == 0xb5: // UBX protocol
        start ::= Time.now
        e := catch: return ubx-message.Message.from-reader reader_
        log.warn "error parsing ubx message" --tags={"error": e}
      // Go to next byte.
      reader_.skip 1

  wait-until-receiver-available_:
    // Block until we can read from the device.
    first ::= reader_.read

    // Consume all data from the device before continuing (without blocking).
    while true:
      e := catch:
        with-timeout --ms=0:
          reader_.read
      if e: return
