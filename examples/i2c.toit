// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import i2c

import ublox_gnss

SDA ::= gpio.Pin 21
SCL ::= gpio.Pin 22

main:
  bus := i2c.Bus --sda=SDA --scl=SCL
  device := bus.device ublox_gnss.I2C_ADDRESS

  driver := ublox_gnss.Driver
    ublox_gnss.Reader device
    ublox_gnss.Writer device

  print "getting location"

  driver.location --blocking
  print "Took: $(driver.time_to_first_fix)"

  while true:
    location := driver.location --blocking
    print "Location: $location ($(max location.horizontal_accuracy location.vertical_accuracy))"

  driver.close
