// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
Diagnostics information for a GNSS receiver.
*/
class GnssDiagnostics:
  /**
  The time to first fix.
  0, if no fix has been obtained.
  */
  time_to_first_fix/Duration

  // TODO(Lau): Detail this measure.
  /**
  The signal quality.
  */
  signal_quality/float

  /**
  Number of satellites currently in view of the GNSS antenna.
  */
  satellites_in_view/int

  /**
  Number of satellites currently known by the GNSS receiver.
  */
  known_satellites/int

  /**
  Constructs diagnostics for GNSS.
  */
  constructor --.time_to_first_fix --.signal_quality --.satellites_in_view --.known_satellites:
