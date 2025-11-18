// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

/**
Diagnostics information for a GNSS receiver.
*/
class Diagnostics:
  /**
  The time to first fix.
  0, if no fix has been obtained.
  */
  time-to-first-fix/Duration

  /** The signal quality. */
  signal-quality/float

  /** Number of satellites currently in view of the GNSS antenna. */
  satellites-in-view/int

  /** Number of satellites currently known by the GNSS receiver. */
  known-satellites/int

  /** Constructs diagnostics for GNSS. */
  constructor --.time-to-first-fix --.signal-quality --.satellites-in-view --.known-satellites:
