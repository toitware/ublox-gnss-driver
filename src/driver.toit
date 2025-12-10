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

  // Maximum ms to wait for command latches.
  static COMMAND-TIMEOUT-MS_ ::= 5000

  // NMEA Helpers while a protocol specific parser doesn't exist.
  // See $disable-nmea-messages_
  static NMEA-CLASS-ID := 0xF0
  static NMEA-MESSAGE-IDs := {
    "GGA": 0x00,
    "GLL": 0x01,
    "GSA": 0x02,
    "GSV": 0x03,
    "RMC": 0x04,
    "VTG": 0x05,
    "GRS": 0x06,
    "GST": 0x07,
    "ZDA": 0x08,
    "GBS": 0x09,
    "DTM": 0x0A,
  }

  time-to-first-fix_ /Duration := Duration.ZERO

  // Latches/Mutexes for managing and acknowledging commands
  waiters_ := []
  command-mutex_ := monitor.Mutex      // Used to ensure one command at once.
  command-latch_ := monitor.Latch      // Used to ensure cfg gets the result.
  runner-start-latch_ := monitor.Latch // Used to ensure message reciever has started.

  diagnostics_ /Diagnostics := Diagnostics --known-satellites=0 --satellites-in-view=0 --signal-quality=0.0 --time-to-first-fix=Duration.ZERO
  location_ /GnssLocation? := null
  adapter_ /Adapter_
  runner_ /Task? := null
  logger_/log.Logger := ?

  // Map to contain the most recent message of all given types.
  latest-message/Map := {:}

  // HWVERSION.  Done this such that users can insert their own HWVERSION if needed.
  ublox-protversion-lookup/Map := {
    "00070000":"14.00",       // M7
    "00040007":"13.00",       // M6
  }

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

  Boolean $disable-auto-run is used to start basic operation.  For users looking
    for advanced operation (eg, starting message reciever task later, or
    subscribing to custom message types etc, the option is given to disable
    the automatic startup functions to start if/when desired.)
  */
  constructor reader writer logger=log.default --disable-auto-run/bool=false:
    logger_ = logger.with-name "ublox-gnss"

    if reader is old-reader.Reader:
      reader = io.Reader.adapt reader

    if writer is not io.Writer:
      writer = io.Writer.adapt writer

    adapter_ = Adapter_ reader writer logger

    if not disable-auto-run:
      // Wait for message reciever task to start.
      // Code moved here and now using a latch to prevent slow startup noticed
      // in one in appx 30 tests.  (Observed time differences have been  between
      // 25ms and >5000ms for the task startup `run` below.)
      start := Time.monotonic-us
      run
      started := runner-start-latch_.get
      duration := Duration --us=(Time.monotonic-us - start)
      logger_.debug "Message Reciever started." --tags={"ms":(duration.in-ms)}

      //sleep --ms=100

      // Turn off default (unused) NMEA messages.
      disable-nmea-messages_

      // Get device type info, before sending anything.
      send-get-mon-ver_

      // Start subscription to default messages.
      start-periodic-nav-packets_

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
      runner-start-latch_.set true
      while true:
        message := adapter_.next-message
        //logger_.debug  "Received: $message"

        if message is ubx-message.AckAck:
          // Message is an ACK-ACK - positive response to a CFG message.
          command-latch_.set (message as ubx-message.AckAck)
          process-ack-ack-message_ message as ubx-message.AckAck

        else if message is ubx-message.AckNak:
          // Message is an ACK-NACK - negative response to a CFG message.
          // (CFG command sent didn't work - unfortunately reasons not given.)
          //command-latch_.set (message as ubx-message.AckNak)
          process-ack-nak-message_ message as ubx-message.AckNak

        else if message is ubx-message.MonVer:
          // If a command was waiting for the response, pass it
          command-latch_.set (message as ubx-message.MonVer)
          process-mon-ver_ message as ubx-message.MonVer

        else if message is ubx-message.NavStatus:
          process-nav-status_ message as ubx-message.NavStatus
        else if message is ubx-message.NavPvt:
          process-nav-pvt_ message as ubx-message.NavPvt
        else if message is ubx-message.NavSat:
          process-nav-sat_ message as ubx-message.NavSat
        else:
          logger_.debug  "Driver received UNHANDLED message type: $message"

  reset:
    adapter_.reset

  close:
    if runner_:
      runner_.cancel
      runner_ = null

  process-ack-nak-message_ message/ubx-message.AckNak:
    //logger_.debug "Received AckNak message." --tags={"class": message.class-id, "message": message.message-id}

  process-ack-ack-message_ message/ubx-message.AckAck:
    //logger_.debug "Received AckAck message." --tags={"class": message.class-id, "message": message.message-id}

  process-nav-status_ message/ubx-message.NavStatus:
    logger_.debug "Received NavStatus message."
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

  process-mon-ver_ message/ubx-message.MonVer:
    device-protocol-version := supported-protocol-version message
    logger_.debug "Received MonVer message." --tags={"sw-ver": message.sw-version, "hw-ver": message.hw-version, "prot-ver": device-protocol-version}

    // Interprets information for the driver/users
    //device-hw-version_ = message.hw-version
    //device-sw-version_ = message.sw-version
    //device-protocol-version_ = supported-protocol-version message

    // Cache ubx-mon-ver message for later queries
    //last-mon-ver-message_ = message
    //latest-message["$(message.cls):$(message.id)"] = message
    //latest-message[message] = message

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
    send-message_ message

  /**
  Sends a request for device information, using UBX-MON-VER message.
  */
  send-get-mon-ver_:
    //logger_.debug "Send Version Request Poll."
    message := ubx-message.MonVer.poll
    send-message_ message

  /**
  Sends various types of CFG messages, and waits for the response.

  Handles logic of success and failure messages, while not blocking other
    message traffic being handled by the driver.

  Todo: Add [--if-error] [--if-success] blocks.
  */
  // ubx-message.Message is the parent class of all the messages.  Some message
  //   types do not have functionality for being sent TO the device.  This is not
  //   checked for here, but rather for the user to guard against.  The runner
  //   would also need latch handling for such messages to avoid always being
  //   handled via the $COMMAND-TIMEOUT-MS_ timeout path.
  send-message_ message/ubx-message.Message --return-immediately/bool=false:
    command-mutex_.do:
      if return-immediately:
        adapter_.send-packet message.to-byte-array
        return

      // Reset the latch
      command-latch_ = monitor.Latch

      start := Time.monotonic-us
      response := null
      duration/Duration := Duration.ZERO
      exception := catch:
        with-timeout --ms=COMMAND-TIMEOUT-MS_:
          adapter_.send-packet message.to-byte-array
          response = command-latch_.get

      duration = Duration --us=(Time.monotonic-us - start)
      if exception:
        logger_.error "Command timed out" --tags={"message":"$(message)", "ms":duration.in-ms}
        return

      if message is ubx-message.CfgMsg:
        if response is ubx-message.AckAck:
          logger_.debug  "Message Reponse." --tags={"message":"$(message)","response":"$(response)","ms":(duration.in-ms)}
          return
        if response is ubx-message.AckNak:
          logger_.error  "**NEGATIVE** acknowledgement." --tags={"message":"$(message)","response":"$(response)","ms":(duration.in-ms)}
          return

      // Other response types, if necessary, here.  Not normally required as the
      //   message receiver ($run) sends the message to the appropriate handler.

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
      // Protver exists, parse it.  Some devices delimit by ' ', some devices '=':
      protver-ext = protver-ext.trim
      pos-eq/int := protver-ext.index-of "="
      pos-sp/int := protver-ext.index-of " "
      if pos-eq > -1:
        return protver-ext[(pos-eq + 1)..]
      else if pos-sp > -1:
        return protver-ext[(pos-sp + 1)..]
      else:
        throw "Couldn't parse protver string: '$(protver-ext)'"

    // Use lookup if the protver string doesn't exist:
    if ublox-protversion-lookup.contains message.hw-version:
      // Assume a u-blox 7, with no PROTVER
      return ublox-protversion-lookup[message.hw-version]

    // All else fails = Assume 12. Address later if an issue is raised.
    return "12.00"

  /**
  Disable all default NMEA messages.

  When a Ublox device is turned on, NMEA messages arrive by default.  This
    command iterates through the set of known default messages and sets the rate
    for each to zero, and for all outputs.  Done this way until enough of an
    NMEA Parser is completed to make this useful.  Note: UBX-CFG-MSG is a legacy
    method.  Currently supported by later devices but could/should use
    UBX-CFG-VALSET at some later point.  This function uses Per-Port method
    because some outputs still send on all ports.  This function does NOT SAVE
    this configuration to the device.

  This has necessary to quieten the uart as much as possible, increasing
    accuracy for things like time synchronisation.
  */
  disable-nmea-messages_ -> none:
    // Bytearray gives zero rate for all outputs.
    rates := #[0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    NMEA-MESSAGE-IDs.values.do:
      logger_.debug "Disable NMEA." --tags={"class": NMEA-CLASS-ID, "message": it, "rate": 0}
      message := ubx-message.CfgMsg.per-port --msg-class=NMEA-CLASS-ID --msg-id=it --rates=rates
      send-message_ message

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
