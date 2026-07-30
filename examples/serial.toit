// Copyright (C) 2025 Toit contributors.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import i2c
import uart
import io

import ublox-gnss

TX-PIN := 17
RX-PIN := 18
BAUD   := 9600
UART   := 0

main:
  port := uart.Port --tx=TX-PIN --rx=RX-PIN --baud-rate=BAUD

  driver := ublox-gnss.Driver port.in port.out

  print "getting location"

  driver.location --blocking
  print "Took: $(driver.time-to-first-fix)"

  while true:
    location := driver.location --blocking
    print "Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000

  driver.close
