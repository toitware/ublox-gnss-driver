// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import serial.protocols.i2c as i2c

import ublox_m

SDA ::= gpio.Pin 21
SCL ::= gpio.Pin 22

main:
  bus := i2c.Bus --sda=SDA --scl=SCL
  device := bus.device ublox_m.I2C_ADDRESS

  driver := ublox_m.Driver
    ublox_m.Reader device
    ublox_m.Writer device

  print "getting location"

  driver.location --blocking
  print "Took: $(driver.time_to_first_fix)"

  while true:
    location := driver.location --blocking
    print "Location: $location ($(max location.horizontal_accuracy location.vertical_accuracy))"

  driver.close
