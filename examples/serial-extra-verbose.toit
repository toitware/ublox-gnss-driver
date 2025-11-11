// Copyright (C) 2025 Toit contributors.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import i2c
import uart
import io

import ublox-gnss

TX-PIN := gpio.Pin 17
RX-PIN := gpio.Pin 18
BAUD   := 9600

main:
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=BAUD
  driver := ublox-gnss.Driver port.in port.out

  sleeptime := 0
  if driver.device-version != null:
  print " Hardware Version: $(driver.device-version.hw-version)"
  print " Software Version: $(driver.device-version.sw-version)"
  driver.device-version.extensions-raw.do:
    print " - Extension:     $(it)"

  //print "Getting location"
  // Non-blocking - we return diagnostics whilst a fix is happening.
  location := driver.location
  known := ""
  sats-iv := ""
  sig-q := ""
  ttff := ""
  time := ""
  diags := ?
  fixtype := ""

  while not location:
    location = driver.location
    diags = driver.diagnostics
    time = "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    known   = "Sats Known: $(diags.known-satellites)"
    sats-iv = "Sats In View: $(diags.satellites-in-view)"
    sig-q   = "SigQual: $(diags.signal-quality)"
    ttff   = "TTFF: $(diags.time-to-first-fix.in-s)"
    if driver.device-status != null:
      fixtype = "Fix: $(driver.device-status.gps-fix-text)"

    print " $time \t $fixtype $known $sats-iv $sig-q $ttff"
    sleep --ms=3000

  print
  print "Time to First Fix: $(driver.time-to-first-fix)"
  while true:
    time = " Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    location = driver.location --blocking
    print " $time \t Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000

  driver.close
