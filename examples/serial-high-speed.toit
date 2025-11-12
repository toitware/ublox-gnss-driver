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
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=START-BAUD
  driver := ublox-gnss.Driver port.in port.out --auto-run=false
  driver.set-uart --baud=TARGET-BAUD
  sleep --ms=50

  port = uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=TARGET-BAUD
  driver = ublox-gnss.Driver port.in port.out

  print "Time to First Fix: $(driver.time-to-first-fix)"
  while true:
    time := " Elapsed: $((Duration --us=(Time.monotonic-us)).in-s)s"
    location := driver.location
    print " $time \t Location: $location "
    sleep --ms=3000

  driver.close
