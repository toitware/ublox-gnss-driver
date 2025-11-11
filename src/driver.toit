// Copyright (C) 2021 Toitware Contributors.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
Driver for Ublox GNSS GPS Modules.

Originally written for a Max M8 GPS module.  Attempts to support this whilst
  also supporting past and future ublox devices by detecting the device version
  and using the correct message types for its generation.

*/

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
import monitor

import .reader
import .writer

I2C-ADDRESS ::= 0x42

class Driver:
  static METER-TO-MILLIMETER ::= 1000
  static METER-TO-CENTIMETER ::= 100
  static COORDINATE-FACTOR /float ::= 10_000_000.0
  static QUALITY-SAT-COUNT_ ::= 4

  static UBLOX7-HWVERSIONS ::= {
    "00070000",
  }
  static UBLOX6-HWVERSIONS ::= {
    "00040007",
  }

  // store the last received message of some message types  (Maybe convert to a map to make it extensible):
  last-nav-status-message_/ubx-message.NavStatus? := null
  last-mon-ver-message_/ubx-message.MonVer? := null

  // fixed properties extracted from messages
  time-to-first-fix_/Duration := Duration.ZERO
  device-protocol-version_/string? := null
  device-hw-version_/string? := null
  device-sw-version_/string? := null

  // Latches/Mutexes for managing and acknowledging commands
  waiters_ := []
  command-mutex_ := monitor.Mutex    // Used to ensure one command at once
  command-ver-latch_ := monitor.Latch    // Used to ensure mutex gets the result
  command-cfg-latch_ := monitor.Latch    // Used to ensure mutex gets the result

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

    //.run already puts runner task in the background
    //if auto-run: task --background:: run
    if auto-run: run

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

    // Start the message parser task to parse messages as they arrive.
    runner_ = task::
      message := ?
      while true:
        message = adapter_.next-message

        if message is ubx-message.AckAck:
          // Message is an ACK-ACK - positive response.
          command-cfg-latch_.set (message as ubx-message.AckAck)
          process-ack-ack-message_ message as ubx-message.AckAck

        else if message is ubx-message.AckNak:
          // Message is an ACK-NACK - negative response. (A command sent didn't work.)
          command-cfg-latch_.set (message as ubx-message.AckNak)
          process-ack-nak-message_ message as ubx-message.AckNak

        else if message is ubx-message.MonVer:
          // If a command was waiting for the response, pass it
          command-ver-latch_.set (message as ubx-message.MonVer)
          process-mon-ver_ message as ubx-message.MonVer

        else if message is ubx-message.NavStatus:
          process-nav-status_ message as ubx-message.NavStatus

        else if message is ubx-message.NavPvt:
          process-nav-pvt_ message as ubx-message.NavPvt

        else if message is ubx-message.NavSat:
          process-nav-sat_ message as ubx-message.NavSat

        else if message is ubx-message.NavPosLlh:
          process-nav-posllh_ message as ubx-message.NavPosLlh

        else if message is ubx-message.NavSvInfo:
          process-nav-svinfo_ message as ubx-message.NavSvInfo

        else if message is ubx-message.NavSol:
          process-nav-sol_ message as ubx-message.NavSol

        else if message is ubx-message.NavTimeUtc:
          process-nav-time-utc_ message as ubx-message.NavTimeUtc

        else:
          logger_.debug "Received UBX message unknown to the driver." --tags={"message": message.stringify}

    // Try and get device type info, before sending anything.
    send-message-deviceinfo-poll_

    // With the device type determined and parsed, configure the device.
    task:: start-periodic-nav-packets_

  reset:
    adapter_.reset

  close:
    if runner_:
      runner_.cancel
      runner_ = null

  // Functions returning information/objects directly to users

  device-version -> ubx-message.MonVer?:
    return last-mon-ver-message_

  device-status -> ubx-message.NavStatus?:
    return last-nav-status-message_

  // Functions processing the different return messages


  process-mon-ver_ message/ubx-message.MonVer:
    //logger_.debug "Received MonVer message." --tags={"sw-ver": device-sw-version_, "hw-ver": device-hw-version_, "prot-ver": device-protocol-version_}

    // Interprets information for the driver/users
    device-hw-version_ = message.hw-version
    device-sw-version_ = message.sw-version
    device-protocol-version_ = supported-protocol-version message

    // Cache ubx-mon-ver message for later queries
    last-mon-ver-message_ = message

  process-ack-nak-message_ message/ubx-message.AckNak:
    //logger_.debug "Received AckNak message." --tags={"class": message.class-id-text , "message": message.message-id-text}

  process-ack-ack-message_ message/ubx-message.AckAck:
    //logger_.debug "Received AckAck message." --tags={"class": message.class-id-text , "message": message.message-id-text}

  process-nav-status_ message/ubx-message.NavStatus:
    logger_.debug "Received NavStatus message." --tags={"gps-fix": message.gps-fix-text, "gps-fix": message.gps-fix-text}

    // Cache last status message
    last-nav-status-message_ = message

    // Cache time-to-first-fix (not sure what use it is but it's there)
    time-to-first-fix_ = (Duration --ms=message.time-to-first-fix)

  process-nav-posllh_ message/ubx-message.NavPosLlh:
    logger_.debug "Received NavPosLlh message." --tags={"latitude" : message.latitude-deg , "longtitude" : message.longitude-deg, "itow": message.itow }

  process-nav-time-utc_ message/ubx-message.NavTimeUtc:
    logger_.debug "Received NavTimeUtc message." --tags={"valid-utc" : message.valid-utc, "time-utc": message.utc-time }

  process-nav-sol_ message/ubx-message.NavSol:
    logger_.debug "Received NavSol message." --tags={"position-dop" : message.position-dop} // , "longtitude" : message.latitude-deg, "itow": message.itow }


  process-nav-pvt_ message/ubx-message.NavPvt:
    logger_.debug "Received NavPvt message."

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

  /** Function processes legacy (<=7M) satellite information messages */
  process-nav-svinfo_ message/ubx-message.NavSvInfo:
    logger_.debug "Received NavSvInfo message." --tags={"satellite-count" : message.satellite-count}

    cnos ::= []
    satellite-count ::= message.satellite-count
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
        --time-to-first-fix=time-to-first-fix_
        --signal-quality=quality
        --satellites-in-view=satellites-in-view
        --known-satellites=known-satellites

  /** Function processes legacy (M8+) satellite information messages */
  process-nav-sat_ message/ubx-message.NavSat:
    logger_.debug "Received NavSat message." --tags={"satellite-count" : message.satellite-count}

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
        --time-to-first-fix=time-to-first-fix_
        --signal-quality=quality
        --satellites-in-view=satellites-in-view
        --known-satellites=known-satellites

  /**
  Sends subscriptions for messages required for the driver to know location.

  Needs to understand what device is currently configured so that the right
    generation of messages can be requested.
  */
  start-periodic-nav-packets_ -> none:
    send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavTimeUtc.ID 5

    if (float.parse device-protocol-version_) >= 15.0 :
      logger_.debug "Setting up for M8 device type."
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavStatus.ID 1
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavPvt.ID 1
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSat.ID 1

    else if (float.parse device-protocol-version_) >= 14.0:
      logger_.debug "Setting up for 7M device type (legacy)."
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavStatus.ID 1
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavPosLlh.ID 1  // Legacy Equivalent to NavPvt Messages
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSvInfo.ID 1  // Legacy Equivalent to NavSat Messages
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSol.ID 1     // Legacy to help replace NavSat Messages

    else if (float.parse device-protocol-version_) >= 13.0:
      logger_.debug "Setting up for 7M device type (legacy)."
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavStatus.ID 1
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavPosLlh.ID 1  // Legacy Equivalent to NavPvt Messages
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSvInfo.ID 1  // Legacy Equivalent to NavSat Messages
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSol.ID 1     // Legacy to help replace NavSat Messages

    else:
      // Assume all others are M8 or later (for now):
      logger_.debug "Defaulting to M8 Device Type."
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavStatus.ID 1
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavPvt.ID 1
      send-set-message-rate_ ubx-message.Message.NAV ubx-message.NavSat.ID 1

  /**
  Determines the protocol version supported by the device.

  Makes assumptions if the device doesn't return the protocol version
    explicitly, in accordance with the table in README.md.  Optionally sets the
    class wide property, in case the user decides to specify it manually.
  */
  supported-protocol-version message/ubx-message.MonVer -> string:
    // Find an extension containing PROTVER and if exists parse it
    protver-ext/string? := message.extension "PROTVER"

    if protver-ext != null:
      // Protver exists, parse it:
      protver-ext = protver-ext.trim
      pos-eq/int := protver-ext.index-of "="
      pos-sp/int := protver-ext.index-of " "
      if pos-eq > -1:
        return protver-ext[(pos-eq + 1)..]
      else if pos-sp > -1:
        return protver-ext[(pos-sp + 1)..]
      else:
        throw "Couldn't parse protver string: '$(protver-ext)'"

    // Make some assumptions if the protver string doesn't exist:
    if (UBLOX7-HWVERSIONS.any: it == message.hw-version):
      // Assume a u-blox 7, with no PROTVER
      return "14.00"
    else if (UBLOX6-HWVERSIONS.any: it == message.hw-version):
      // Assume a u-blox 6,, with no PROTVER
      return "13.00"
    else:
      // Anything older = Assume 12. Address later if an issue is raised.
      return "12.00"

  /**
  Sends a subscription for specific message, and the defined rate.

  Constructs (and sends) a UBX-CFG message asking for the specified class-id/
    message-id message type, at the specifid rate
  */
  send-set-message-rate_ class-id message-id rate:
    message := ubx-message.CfgMsg.message-rate --msg-class=class-id --msg-id=message-id --rate=rate
    command-mutex_.do:
      // Reset the latch
      command-cfg-latch_ = monitor.Latch

      logger_.debug  "Sent request." --tags={"message":"$(message)"}
      adapter_.send-packet message.to-byte-array

      //To do: give a timeout.
      start := Time.monotonic-us
      adapter_.send-packet message.to-byte-array
      response := command-cfg-latch_.get
      duration := Duration --us=(Time.monotonic-us - start)
      logger_.debug  "Received answer." --tags={"message":"$(message)","response":"$(response)","ms":(duration.in-ms)}

      if response is ubx-message.AckAck:
        // logger_.debug  "Acknowledged."
      if response is ubx-message.AckNak:
        logger_.debug  "NEGATIVE acknowledged."

  /**
  Sends a request for device information, using UBX-MON-VER message
  */
  send-message-deviceinfo-poll_:
    message := ubx-message.MonVer.poll
    command-mutex_.do:
      // Reset the latch
      command-cfg-latch_ = monitor.Latch

      logger_.debug  "Sent request." --tags={"message":"$(message)"}
      start := Time.monotonic-us
      adapter_.send-packet message.to-byte-array
      response := command-ver-latch_.get
      duration := Duration --us=(Time.monotonic-us - start)

      logger_.debug  "Received answer." --tags={"message":"$(message)","response":"$(response)","ms":(duration.in-ms)}


class Adapter_:
  static STREAM-DELAY_ ::= Duration --ms=1

  logger_/log.Logger
  reader_/io.Reader
  writer_/io.Writer

  constructor .reader_ .writer_ logger:
    logger_ = logger.with-name "ublox-gnss"

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
//    logger_.debug "send-packet: sent message." --tags={"payload[0..3]": bytes[0..3]}
    writer_.write bytes
    sleep STREAM-DELAY_

  send-ubx message/ubx-message.Message:
//    logger_.debug "send-ubx: sent message." --tags={"payload": message.payload}
    writer_.write message.to-byte-array
    sleep STREAM-DELAY_

  next-message -> ubx-message.Message:
    msg/ubx-message.Message := ?
    while true:
      peek ::= reader_.peek-byte 0
      if peek == 0xb5: // UBX protocol
        start ::= Time.now
        e := catch:
          msg = ubx-message.Message.from-reader reader_
//          if msg.payload.size > 3:
//            logger_.debug "next-message: Got message." --tags={"payload[0..3]": msg.payload[0..3]}
//          else:
//            logger_.debug "next-message: Got message." --tags={"payload": #[]}
          return msg
        logger_.warn "next-message: error parsing ubx message" --tags={"error": e}
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
