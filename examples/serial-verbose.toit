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
  // Open serial communication and start driver.
  print "Opening on $BAUD"
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=BAUD
  driver := ublox-gnss.Driver port.in port.out

  // Wait for the driver to get version message back for display.
  timeout := catch:
    with-timeout --ms=10_000:
      while (not driver.latest-message.contains "VER") or (driver.latest-message["VER"] == null):
        sleep --ms=250

  if not timeout:
    version-message := driver.latest-message["VER"]
    print "Device detected:"
    print " Hardware Version: $(version-message.hw-version)"
    print " Software Version: $(version-message.sw-version)"
    version-message.extensions-raw.do:
      print " - Extension:     $(it)"
  else:
    print "Device detection: timed out."

  // Non-blocking - we return diagnostics whilst a fix is happening.
  // Written to show diagnostics at least once. (If module already powered with
  // an existing fix, these may not be shown in the example.)
  print "Awaiting Location Fix:"
  location := null
  while not location:
    diags := driver.diagnostics
    time := "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    known := "Sats Known: $(diags.known-satellites)"
    sats-iv := "Sats in View: $(diags.satellites-in-view)"
    sig-q := "SigQual: $(diags.signal-quality)"
    ttff := "TTFF: $(diags.time-to-first-fix.in-s)"
    fixtype := ""

    if (driver.latest-message.contains "STATUS") and (driver.latest-message["STATUS"] != null):
      fixtype = "Fix: $(driver.latest-message["STATUS"].gps-fix-text)"

    print " $time \t $fixtype $known $sats-iv $sig-q $ttff"
    if driver.location:
      location = driver.location
      break
    sleep --ms=3000

  print "Location found:"
  print " Time to First Fix: $(driver.time-to-first-fix))"
  while true:
    time := "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    print " $time \t Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000
    location = driver.location --blocking

  driver.close
