// Copyright (C) 2020 Toitware ApS. All rights reserved.

import binary
import gnss_location show GnssLocation
import location show Location
import log
import math
import monitor show Semaphore Latch
import peripherals.rpc
import peripherals.gps show GpsDiagnostics
import metrics show METRICS_LEVEL_DEBUG METRICS_LEVEL_INFO
import uuid

import .esp.max_m8 show MaxM8
import .esp.ubx_packet as ubx
import .resource
import ..kernel.rpc
import ..kernel.processes
import ..kernel.blobs show Blob BlobManager
import ..kernel.blob_utils show path_to_blob_id
import ..kernel.context
import ..kernel.component as component
import ..kernel.metrics as metrics
import ..kernel.utils show status_tracker GPS_STATUS NO_FIX_STATUS SEARCHING_STATUS FIX_STATUS

import ..api.gps as gps

ENABLE_LOG_SATELLITE_DETAILS_ ::= false
QUALITY_SAT_COUNT_ ::= 4

METER_TO_MILLIMETER ::= 1000
METER_TO_CENTIMETER ::= 100

class MaxM8Api extends gps.Api:
  static NEEDED_GPS_FILES_ ::= ["/gps/data.full"]

  max_m8_/MaxM8 ::= ?

  constructor .max_m8_ component/MaxM8Component:
    super component

  needed_files -> List: return NEEDED_GPS_FILES_

  update_file_validity manager/BlobManager path/string -> none:
    blob/Blob? ::= manager.get path
    if not blob: return
    expiration/Time? ::= max_m8_.compute_aid_expiration blob.content
    if not expiration: return
    id/uuid.Uuid ::= path_to_blob_id path
    manager.update_expiration id expiration - (Duration --h=24)

class MaxM8Descriptor extends component.DescriptorBase implements gps.Descriptor:
  component/MaxM8Component ::= ?

  constructor.from_process .component process_manager/ProcessManager gid/int:
    super.from_process process_manager gid

  read -> GnssLocation?:
    return component.driver_.location

  diagnostics -> GpsDiagnostics:
    return GpsDiagnostics
      --time_to_first_fix=component.driver_.time_to_first_fix
      --signal_quality=component.driver_.signal_quality
      --satellites_in_view=component.driver_.satellites_in_view
      --known_satellites=component.driver_.known_satellites

  clear_last_known_location -> none:
    component.driver_.clear_last_known_location

  set_last_known_location location/Location altitude/float accuracy/float -> none:
    component.driver_.set_last_known_location location altitude accuracy

class MaxM8Component extends component.ASyncComponent implements gps.Component:
  driver_/GpsDriver ::= ?
  config_ := null

  constructor .driver_ --parents=[]:
    super "max-m8" --parents=parents

  open process_manager/ProcessManager gid/int config -> gps.Descriptor:
    // Note that only the initial start of the GPS module will be able to
    // provide a config.
    config_ = config
    return MaxM8Descriptor.from_process this process_manager gid

  transition_on:
    driver_.start config_

  transition_off:
    driver_.stop

abstract class GpsDriver:
  max_m8_/MaxM8 ::= ?

  task_ := null
  stop_latch_/Latch? := null

  location_/GnssLocation? := null
  signal_quality_/float := 0.0
  time_to_first_fix_/int := 0
  satellites_in_view_/int := 0
  known_satellites_/int := 0

  logger_/log.Logger? := null

  static COORDINATE_FACTOR/float ::= (math.pow 10 7)

  static ttff_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.ttff" --level=METRICS_LEVEL_INFO
  static hacc_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.hacc" --level=METRICS_LEVEL_DEBUG
  static vacc_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.vacc" --level=METRICS_LEVEL_DEBUG
  static signal_gauge_/metrics.Gauge ::= metrics.create_gauge "gps.signal" --level=METRICS_LEVEL_DEBUG

  constructor context/KernelApi .max_m8_ --logger/log.Logger=log.default:
    max_m8_.with_logger logger
    store ::= context.store "gnss"
    MaxM8.gnss_store_ = store
    with_logger logger

  with_logger logger/log.Logger:
    logger_ = logger.with_name "maxm8"

  start config:
    if task_: throw "MaxM8: Task already started"

    start_latch ::= Latch
    stop_latch_ = Latch
    task_ = task::
      try:
        critical_do: setup_gps_ config
        catch --trace:
          while not task.is_canceled:
            max_m8_.next_packet: | packet |
              if packet.is_ubx_nav_status:
                process_nav_status_ packet.ubx_nav_status
              else if packet.is_ubx_nav_pvt:
                process_nav_pvt_ packet.ubx_nav_pvt
              else if packet.is_ubx_nav_sat:
                process_nav_sat_ packet.ubx_nav_sat
                start_latch.set null
              else if packet.is_ubx_nav_timeutc:
                process_nav_timeutc_ packet.ubx_nav_timeutc
      finally:
        try:
          critical_do: stop_gps_
        finally:
          stop_latch_.set null
          stop_latch_ = null
          task_ = null

    incomplete := true
    try:
      // Wait for initial position data, before returning from start. That ensure
      // the assist data has been fully processed. If this fails, we attempt to stop
      // the GPS driver again in the finally-clause.
      start_latch.get
      incomplete = false
    finally:
      if incomplete: stop

  stop:
    if task_:
      stop_latch ::= stop_latch_
      task_.cancel
      if stop_latch: stop_latch.get

  location -> GnssLocation?:
    return location_

  signal_quality -> float:
    return signal_quality_

  time_to_first_fix -> int:
    return time_to_first_fix_

  satellites_in_view -> int:
    return satellites_in_view_

  known_satellites -> int:
    return known_satellites_

  stop_gps_:
    location_ = null
    time_to_first_fix_ = 0
    signal_quality_ = 0.0
    satellites_in_view_ = 0
    known_satellites_ = 0
    power_off
    status_tracker[GPS_STATUS] = NO_FIX_STATUS

  setup_gps_ config:
    status_tracker[GPS_STATUS] = SEARCHING_STATUS
    // Power-on will always reset (cold-boot) the device.
    power_on
    // Flush data
    max_m8_.flush
    if config and config.get "no_aid": debug "skipping receiver aid"
    else: max_m8_.provide_receiver_aid
    max_m8_.start_periodic_nav_packets

  process_nav_status_ status/ubx.NavStatus:
    if time_to_first_fix_ == 0:
      ttff ::= status.time_to_first_fix
      if ttff != 0:
        ttff_gauge_.set ttff
        time_to_first_fix_ = ttff

  process_nav_pvt_ pos/ubx.NavPvt:
    if pos.is_gnss_fix:
      location_ = GnssLocation
          Location.from_values pos.lat / COORDINATE_FACTOR pos.lon / COORDINATE_FACTOR
          pos.height_msl / METER_TO_MILLIMETER
          Time.now
          pos.horizontal_acc.to_float / METER_TO_MILLIMETER
          pos.vertical_acc.to_float / METER_TO_MILLIMETER
      hacc_gauge_.set pos.horizontal_acc
      vacc_gauge_.set pos.vertical_acc
      status_tracker[GPS_STATUS] = FIX_STATUS

  process_nav_sat_ packet/ubx.NavSat:
    cnos ::= []
    details ::= []
    satellite_count ::= packet.satellite_count
    satellite_count.repeat: | index |
      satellite_data ::= packet.satellite_data index
      cnos.add satellite_data.cno

      if ENABLE_LOG_SATELLITE_DETAILS_ and satellite_data.cno > 0:
        details.add satellite_data

    // Compute signal quality estimate.
    cnos.sort --in_place: | a b | b - a
    n ::= min cnos.size QUALITY_SAT_COUNT_
    sum := 0.0
    n.repeat: sum += cnos[it]
    quality ::= sum / QUALITY_SAT_COUNT_
    signal_gauge_.set quality
    signal_quality_ = quality

    // Update sattelites in view.
    satellites_in_view_ = cnos.reduce --initial=0: | count cno |
      count + (cno > 0 ? 1 : 0)
    known_satellites_ = satellite_count

    msg := "satellites $quality #$satellite_count = $details"
    if ENABLE_LOG_SATELLITE_DETAILS_:
      logger_.info msg
    status_tracker.log "GPS: $msg"

  process_nav_timeutc_ time/ubx.NavTimeutc:
    // The time and/or accuracy of the GPS chip can be debugged here.

  clear_last_known_location -> none:
    max_m8_.clear_last_known_location

  set_last_known_location location/Location altitude/float accuracy/float -> none:
    max_m8_.set_last_known_location
      --latitude=(location.latitude.value * COORDINATE_FACTOR).to_int
      --longitude=(location.longitude.value * COORDINATE_FACTOR).to_int
      --altitude=(altitude * METER_TO_CENTIMETER).to_int
      --accuracy=(accuracy * METER_TO_CENTIMETER).to_int

  abstract power_on -> none
  abstract power_off -> none
