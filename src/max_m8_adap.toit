// Copyright (C) 2019 Toitware ApS. All rights reserved.

// Driver for Max M8 GPS module.

import binary show LITTLE_ENDIAN
import bytes
import device
import serial
import reader show *
import monitor show Latch
import uuid
import encoding.ubjson as ubjson
import location show *
import log
import math
import metrics show METRICS_LEVEL_DEBUG

import ...kernel.metrics as metrics
import ...kernel.kv_store
import ...kernel.time show estimate_accuracy
import .ubx_packet as ubx
import ...modules.blob as blob
import uuid

// High-level MaxM8 implementation configuration.
ENABLE_ANO_PACKET_FILTERING_    ::= true
ENABLE_MGA_ACKS_                ::= true

// Low-level MaxM8 implementation configuration.
MAX_BUFFER_SIZE_                ::= 64 // bytes
WAIT_BEFORE_NEXT_READ_ATTEMPT_  ::= 5 // ms
STREAM_DELAY_                   ::= 1 // ms
BUSY_LOOP_WAIT_TIME_            ::= 2 // ms

MGA_ACK_TIMEOUT_                ::= 2_000 // ms
MGA_SEND_TIME_DELAY_ESTIMATE_   ::= 3 // ms
MGA_SEND_TIME_ATTEMPTS_         ::= 3
MGA_SEND_LOCATION_ATTEMPTS_     ::= 2

MIN_TIME_BETWEEN_LOCATION_SAVE_ ::= Duration --s=5
LAST_KNOWN_LOCATION_KEY_        ::= "__last_loc__"

class MaxM8:
  static gnss_store_/SerializedStore? := null

  device_/serial.Device? := null
  reader_/BufferedReader := ?
  raw_reader_/MaxM8Reader? := null
  logger_/log.Logger? := null

  // Information about last fix/location.
  last_fix_/Time? := null
  has_last_known_location_ := false

  // MGA ACK bookkeeping.
  mga_acks_ := 0
  mga_acks_failures_ := null

  static mga_acks_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.mga_acks" --level=METRICS_LEVEL_DEBUG
  static mga_time_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.mga_time" --level=METRICS_LEVEL_DEBUG
  static mga_data_age_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.mga_data_age" --level=METRICS_LEVEL_DEBUG
  static mga_data_sent_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.mga_data_sent" --level=METRICS_LEVEL_DEBUG

  constructor .device_ --logger/log.Logger=log.default:
    raw_reader_ = MaxM8Reader device_
    reader_ = BufferedReader raw_reader_
    with_logger logger

  constructor.reader reader --logger/log.Logger=log.default:
    reader_ = BufferedReader reader
    with_logger logger

  with_logger logger/log.Logger -> MaxM8:
    logger_ = logger.with_name "maxm8"
    has_last_known_location_ = gnss_store_ and (gnss_store_.get LAST_KNOWN_LOCATION_KEY_) != null
    return this

  /**
  Puts the receiver in a low power mode.
  It can only wake up if wired up correctly.
  // TODO(Lau): Make precise what needs to be wired up for the GPS to wake up.
  */
  power_down:
    wait_until_receiver_available_
    send_packet (ubx.RxmPmreq).to_byte_array

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
    last_fix_ = null

  send_packet bytes:
    device_.write bytes
    sleep --ms=STREAM_DELAY_

  next_packet [handler]:
    while true:
      peek ::= reader_.byte 0
      if peek == 0xb5: // UBX protocol
        start ::= Time.now
        packet := parse_ubx_
        if packet:
          if packet.is_ubx_nav_pvt:
            pvt := packet.ubx_nav_pvt
            update_last_known_location_ pvt
          return handler.call packet
      // Go to next byte.
      reader_.skip 1

  parse_ubx_ -> ubx.Message?:
    // Verify header.
    if (reader_.byte 0) != 0xb5 or (reader_.byte 1) != 0x62: return null

    // Verify length and get full the packet.
    length ::= (reader_.byte 4) | (((reader_.byte 5) & 0xff) << 8)
    if length < 0 or length > 512: return null
    packet ::= reader_.bytes length + 8

    // Verify the checksum.
    ck_a ::= packet[length + 6]
    ck_b ::= packet[length + 7]
    ubx.compute_checksum packet: | a b | if not (ck_a == a and ck_b == b): return null

    // Get message class, id, and payload.
    msg_class ::= packet[2]
    msg_id    ::= packet[3]
    payload   ::= packet.copy 6 (length + 6)  // TODO(kasper): Do we really have to copy this?

    // Consume the full packet from the reader and return it.
    reader_.skip length + 8
    return ubx.Message msg_class msg_id payload

  start_periodic_nav_packets:
    send_packet (ubx.CfgMsg --msg_class=ubx.Message.NAV --msg_id=0x03 --rate=2).to_byte_array
    send_packet (ubx.CfgMsg --msg_class=ubx.Message.NAV --msg_id=0x07 --rate=2).to_byte_array
    send_packet (ubx.CfgMsg --msg_class=ubx.Message.NAV --msg_id=0x35 --rate=2).to_byte_array
    send_packet (ubx.CfgMsg --msg_class=ubx.Message.NAV --msg_id=0x21 --rate=2).to_byte_array

  wait_for_mga_ack_ expected_packet_id/int -> bool:
    if not ENABLE_MGA_ACKS_: return true
    start ::= Time.monotonic_us
    while true:
      next_packet: | packet |
        if packet.pack_class == ubx.Message.MGA and packet.pack_id == 0x60:
          mga_acks_++
          id := packet.payload[3]
          if packet.payload[0] == 0x01:
            return (id == expected_packet_id)
          else:
            code ::= packet.payload[2]
            map := mga_acks_failures_.get id --init=: {:}
            count := map.get code --if_absent=: 0
            map[code] = count + 1
            return false
      elapsed ::= Time.monotonic_us - start
      if elapsed > MGA_ACK_TIMEOUT_ * 1000:
        logger_.debug "timed out waiting for mga ack #$mga_acks_ ($(elapsed / 1000) ms)"
        return false

  compute_aid_expiration bytes/ByteArray -> Time?:
    last/Time? := null
    offset := 0
    while offset < bytes.size:
      if (bytes[offset]) != 0xb5 or (bytes[offset + 1]) != 0x62: break
      length ::= (LITTLE_ENDIAN.uint16 bytes offset + 4) + 8  // 8 = 6 bytes header, 2 bytes checksum.
      packet_class ::= bytes[offset + 2]
      packet_id ::= bytes[offset + 3]
      if packet_class == ubx.Message.MGA and packet_id == 0x20:
        year  ::= (bytes[offset + 10]) + 2000  // Message year stored as years after 2000.
        month ::= (bytes[offset + 11])         // Message month stored as 1,..,12.
        day   ::= (bytes[offset + 12])
        entry ::= Time.utc year month day
        if not last or entry > last: last = entry
      offset += length
    return last

  provide_receiver_aid:
    start ::= Time.monotonic_us
    assistance_data ::= blob.fetch "/gps/data.full"
    if not assistance_data:
      logger_.debug "no assistance data blob found"
      return

    // Optionally enable acks for every MGA message sent.
    mga_acks_ = 0
    mga_acks_failures_ = {:}
    if ENABLE_MGA_ACKS_: send_packet (ubx.CfgNavx5 --ack_aiding=true).to_byte_array

    // Send the current time and accuracy to the receiver (if we have one).
    send_approximate_time_

    // Send last known location before almanac / ephemeris data.
    send_last_known_location_

    // Send offline assistance data.
    assistance_packets ::= send_todays_assistance_data_ assistance_data

    // Disable acks for MGA message again.
    if ENABLE_MGA_ACKS_:
      send_packet (ubx.CfgNavx5 --ack_aiding=false).to_byte_array
      mga_acks_gauge_.set mga_acks_

    // Track how long it took to provide receiver aid.
    elapsed ::= Time.monotonic_us - start
    logger_.debug "receiver aid provided in $(elapsed / 1000) ms with $mga_acks_ acks | $mga_acks_failures_"
    mga_time_gauge_.set elapsed
    mga_acks_failures_ = null

  send_approximate_time_:
    // TODO(kasper): Do we want to provide receiver aid at all if we don't
    // have a current time?
    accuracy := device.estimate_time_accuracy
    if not accuracy:
      logger_.debug "no approximate time to send"
      return

    // We estimate that there is a delay involved in sending the new time to the
    // device in order of a couple of milliseconds. We take that into account in
    // our accuracy computation and in the UTC time we send.
    delay ::= Duration --ms=MGA_SEND_TIME_DELAY_ESTIMATE_
    accuracy = accuracy + delay.in_ns
    ns     ::= accuracy % Duration.NANOSECONDS_PER_SECOND
    secs   ::= accuracy / Duration.NANOSECONDS_PER_SECOND

    MGA_SEND_TIME_ATTEMPTS_.repeat:
      sent ::= Time.now + delay
      packet ::= ubx.MgaIniTimeUtc --time=sent --second_accuracy=secs --nanosecond_accuracy=ns
      send_packet packet.to_byte_array
      if wait_for_mga_ack_ packet.id:
        logger_.debug "approximate time sent" --tags={"acc_secs": secs, "acc_ns": ns}
        return
    logger_.debug "failed to send approximate time in $MGA_SEND_TIME_ATTEMPTS_ attempts"

  send_last_known_location_:
    // When no location is available, the ublox u-center tool sends a dummy location with a high inaccuracy.
    // This seems to perform better than sending no location at all, and it looks like it forces the receiver
    // to keep the aiding data.
    latitude  := 0
    longitude := 0
    altitude  := 0
    accuracy  := 20_100_000_00  // in cm -> ~20100 km.
    last_known_location := null
    if gnss_store_: last_known_location = gnss_store_.get LAST_KNOWN_LOCATION_KEY_
    if last_known_location:
      latitude = last_known_location.get "lat"
      longitude = last_known_location.get "lon"
      altitude = last_known_location.get "alt"  // TODO(kasper): Shouldn't be in cm already.
      accuracy = last_known_location.get "acc"  // TODO(kasper): Shouldn't be in cm already.

    MGA_SEND_LOCATION_ATTEMPTS_.repeat:
      packet ::= ubx.MgaIniPosLlh --latitude=latitude --longitude=longitude --altitude=altitude --accuracy_cm=accuracy
      send_packet packet.to_byte_array
      if wait_for_mga_ack_ packet.id:
        logger_.debug "last known location sent" --tags={"lat": latitude, "lon": longitude, "alt": altitude, "acc": accuracy}
        return
    logger_.debug "failed to send last known location in $MGA_SEND_LOCATION_ATTEMPTS_ attempts"

  set_last_known_location --latitude/int --longitude/int --altitude/int --accuracy/int:
    if not gnss_store_: return
    gnss_store_.set LAST_KNOWN_LOCATION_KEY_ {
      "lat": latitude,
      "lon": longitude,
      "alt": altitude,
      "acc": accuracy,
    }
    has_last_known_location_ = true

  clear_last_known_location:
    if not gnss_store_: return
    gnss_store_.remove LAST_KNOWN_LOCATION_KEY_
    last_fix_ = null

  send_todays_assistance_data_ bytes/ByteArray:
    today ::= (Time.now.utc.with --h=0 --m=0 --s=0 --ns=0).time

    sent := 0
    offset := 0
    message_index := 0
    first_ano_message := true
    first_ano_message_sent := -1
    ano_messages_sent_count := 0
    while offset < bytes.size:
      if (bytes[offset]) != 0xb5 or (bytes[offset + 1]) != 0x62:
        logger_.debug "invalid ubx package in GPS assistance data" --tags={"data_offset": offset, "message_index": message_index}
        break
      length ::= (LITTLE_ENDIAN.uint16 bytes offset + 4) + 8  // 8 = 6 bytes header, 2 bytes checksum.
      transmit := true  // Send anything not explicitly filtered out.

      packet_class ::= bytes[offset + 2]
      packet_id ::= bytes[offset + 3]
      if ENABLE_ANO_PACKET_FILTERING_ and packet_class == ubx.Message.MGA and packet_id == 0x20:
        // Filter out UBX-MGA-ANO packets not for today.
        year  ::= bytes[offset + 10] + 2000  // Message year stored as years after 2000.
        month ::= bytes[offset + 11]         // Message month stored as 1,..,12.
        day   ::= bytes[offset + 12]
        entry ::= Time.utc year month day
        if first_ano_message:
          logger_.debug "assistance data from $entry"
          mga_data_age_gauge_.set (entry.to today).in_h
          first_ano_message = false
        if entry == today:
          if first_ano_message_sent < 0: first_ano_message_sent = message_index
          ano_messages_sent_count++
        transmit = (entry == today)

      if transmit:
        packet ::= bytes.copy offset (offset + length)
        send_packet packet
        wait_for_mga_ack_ packet_id
        sent++

      offset += length
      message_index++;

    mga_data_sent_gauge_.set sent
    logger_.debug "GPS assistance data" --tags={"total_sent": sent, "ano_sent": ano_messages_sent_count, "from": first_ano_message_sent, "length": message_index}
    return sent

  wait_until_receiver_available_:
    // Block until we can read from the device.
    first ::= raw_reader_.read
    // Consume all data from the device before continuing (without blocking).
    while true:
      data ::= raw_reader_.read_
      if data == null: break

  update_last_known_location_ packet/ubx.NavPvt:
    if not ubx.NavPvt.FIX_TYPE_2D <= packet.fix_type <= ubx.NavPvt.FIX_TYPE_GNNS_DEAD: return  // No fix was obtained.

    has_fix   ::= packet.is_gnss_fix
    longitude ::= packet.lon
    latitude  ::= packet.lat
    altitude  ::= packet.height / 10
    haccuracy ::= packet.horizontal_acc / 10
    vaccuracy ::= packet.vertical_acc / 10
    accuracy  := (math.sqrt (math.pow haccuracy 2) + (math.pow vaccuracy 2)).to_int

    // Debug aid.
    tags ::= {"fix": has_fix, "lat": latitude, "lon": longitude, "alt": altitude, "hacc": haccuracy, "vacc": vaccuracy, "type": packet.fix_type}

    // Don't update last known location with non-fixes unless we don't already have one. Also, don't update
    // it too frequently.
    if has_last_known_location_ and (not has_fix or (last_fix_ and (Duration.since last_fix_) < MIN_TIME_BETWEEN_LOCATION_SAVE_)):
      logger_.debug "new location" --tags=tags
      return

    set_last_known_location --latitude=latitude --longitude=longitude --altitude=altitude --accuracy=accuracy
    last_fix_ = Time.now
    logger_.debug "new location (last known)" --tags=tags

class MaxM8Reader implements Reader:
  static REGISTER_AVAILABLE_BYTES_ ::= 0xFD
  static REGISTER_DATA_STREAM_     ::= 0xFF

  registers_/serial.Registers ::= ?

  constructor device/serial.Device:
    registers_ = device.registers

  read -> ByteArray?:
    while true:
      bytes ::= read_
      if bytes: return bytes
      sleep --ms=WAIT_BEFORE_NEXT_READ_ATTEMPT_

  read_ -> ByteArray?:
    available_bytes ::= registers_.read_u16_be REGISTER_AVAILABLE_BYTES_: 0
    if available_bytes == 0:
      return null

    return registers_.read_bytes
      REGISTER_DATA_STREAM_
      min MAX_BUFFER_SIZE_ available_bytes
