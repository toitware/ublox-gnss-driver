// Copyright (C) 2025 Toitware ApS. All rights reserved.
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

  print "Getting location"

  // Non-blocking - we return diagnostics whilst a fix is happening
  location := driver.location
  known := ""
  sats-iv := ""
  sig-q := ""
  ttff := ""
  time := ""
  diags := ?

  while location == null:
    location = driver.location
    diags = driver.diagnostics
    time = "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    known   = "Known Satellites: $(diags.known-satellites)"
    sats-iv = "Satellites in view: $(diags.satellites-in-view)"
    sig-q   = "Signal quality: $(diags.signal-quality)"
    ttff   = "Time To First Fix: $(diags.time-to-first-fix)"
    print "$time \t $known \t $sats-iv \t $sig-q \t $ttff"
    sleep --ms=3000

  print
  print "Time to First Fix: $(driver.time-to-first-fix)"
  while true:
    time = "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    location = driver.location --blocking
    print "$time \t Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000

  driver.close
