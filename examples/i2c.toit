// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import i2c

import ublox-gnss

SDA ::= gpio.Pin 21
SCL ::= gpio.Pin 22

main:
  bus := i2c.Bus --sda=SDA --scl=SCL
  device := bus.device ublox-gnss.I2C-ADDRESS

  driver := ublox-gnss.Driver
    ublox-gnss.Reader device
    ublox-gnss.Writer device

  print "getting location"

  driver.location --blocking
  print "Took: $(driver.time-to-first-fix)"

  while true:
    location := driver.location --blocking
    print "Location: $location ($(max location.horizontal-accuracy location.vertical-accuracy))"
    sleep --ms=3000

  driver.close
