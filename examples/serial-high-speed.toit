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
START-BAUD   := 9600
TARGET-BAUD  := 115200

main:
  // Open port, and driver, and configure to a new speed.
  print "Configuring on $START-BAUD for $TARGET-BAUD"
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=START-BAUD
  driver := ublox-gnss.Driver port.in port.out --auto-run=false
  driver.set-uart --baud=TARGET-BAUD

  // Reestablish UART at that higher speed, and connect.
  print "Opening on $TARGET-BAUD"
  port = uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=TARGET-BAUD
  driver = ublox-gnss.Driver port.in port.out

  // Show some data from the device, just to show its working.
  while not driver.location:
    diags := driver.diagnostics
    time := "Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    known := "Sats Known: $(diags.known-satellites)"
    sats-iv := "Sats In View: $(diags.satellites-in-view)"
    sig-q := "SigQual: $(diags.signal-quality)"
    ttff := "TTFF: $(diags.time-to-first-fix.in-s)"
    fixtype := ""
    if driver.device-status != null:
      fixtype = "Fix: $(driver.device-status.gps-fix-text)"

    print " $time \t $fixtype $known $sats-iv $sig-q $ttff"
    sleep --ms=3000

  print "Time to First Fix: $(driver.time-to-first-fix)"
  while true:
    time := " Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    location := driver.location --blocking
    print " $time \t Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000

  driver.close
