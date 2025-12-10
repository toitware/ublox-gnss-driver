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
  print "Opening on $BAUD"
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=BAUD
  driver := ublox-gnss.Driver port.in port.out

  exception := catch:
    with-timeout --ms=10_000:
      while (not driver.latest-message.contains "VER") or (driver.latest-message["VER"] == null):
        sleep --ms=250

  if not exception:
    version-message := driver.latest-message["VER"]
    print "Device detected:"
    print " Hardware Version: $(version-message.hw-version)"
    print " Software Version: $(version-message.sw-version)"
    version-message.extensions-raw.do:
      print " - Extension:     $(it)"
  else:
    print "Device detection timeout."

  // Non-blocking - we return diagnostics whilst a fix is happening.
  location := null
  known := ""
  sats-iv := ""
  sig-q := ""
  ttff := ""
  time := ""
  diags := ?
  fixtype := ""

  print "Awaiting Location Fix:"
  while not location:
    diags = driver.diagnostics
    time = "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    known   = "Sats Known: $(diags.known-satellites)"
    sats-iv = "Sats In View: $(diags.satellites-in-view)"
    sig-q   = "SigQual: $(diags.signal-quality)"
    ttff   = "TTFF: $(diags.time-to-first-fix.in-s)"

    if driver.latest-message["STATUS"] != null:
      fixtype = "Fix: $(driver.latest-message["STATUS"].gps-fix-text)"

    print " $time \t $fixtype $known $sats-iv $sig-q $ttff"
    if driver.location:
      location = driver.location
      break
    sleep --ms=3000

  print "Location found:"
  print " Time to First Fix: $(driver.time-to-first-fix))"
  while true:
    time = "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    location = driver.location --blocking
    print " $time \t Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000

  driver.close
