# UBlox GNSS
Driver for the u-blox M* GNSS receivers.

## Interface Support
| Interface | Support |
|-|-|
| I2C | Supported. [Toit code example.](https://github.com/toitware/ublox-gnss-driver/blob/main/examples/i2c.toit) |
| Serial | Supported. [Toit code example.](https://github.com/toitware/ublox-gnss-driver/blob/main/examples/serial.toit) |
| SPI | Supported. [Toit code example.]() |

## Devices Tested
This driver aims to support all UBX-compatible devices, though coverage is
limited by the hardware and chipsets available to us for testing.  The code was
first written for the M8 device, with others added later.

If you have a device that isn’t listed and can donate one, or help with
testing, please [get in touch](https://discord.gg/Q7Y9VQ5nh2) or open an issue —
contributions and reports are very welcome.

Devices and their reported HW/SW/PROTVER versions are listed below:
| Device | HW Version | 'ROM CORE' (SW-VER) | Minumum UBX Protocol Version | From Toit Driver Version | Notes |
|-|-|-|-|-|-|
| M8 | - | - | `15.00`  | >`1.2.0` | Originally written to support this device. |
| `NEO-7M-0-000` | `00070000` | >`1.00 (59842) Jun 27 2012` | `14.0` | `1.3.0` | Operational, undergoing testing. |
| `NEO-6M-0-001` | `00040007` | >`7.03 (45969) Mar 17 2011` | `14.0` (Assumed, device does not advertise  extensions)  | `1.3.0` | Operational, undergoing testing. |

## Protocol Support
Driver uses the [`ubx-message`](https://github.com/toitware/ubx-message) protocol, parser to
handle parsing messages from the ublox device.  The devices also support other
open protocols, which will be developed as needed.
| Protocol | Support | Notes |
|-|-|-|
|  [`ubx-message`](https://github.com/toitware/ubx-message)  | Yes | Binary, Open but proprietary.  Originally written alongside this driver in support of the M8 device.  |
| `NMEA` | Not yet. | ASCII text sentences (e.g., GGA, RMC) widely used for basic GNSS output.  Widely supported on ublox devices. |
| `RTCM` | Not yet. | Standard for GNSS differential/RTK correction messages. u-blox M8 documentation mentions RTCM, however F9M features full support. |


## How to manage specific tasks

### Switch Serial interface to higher baud rate
Whilst the serial driver is established at a specific speed, both the device/driver
need to be configured at the same time for the required speed.



### Getting time synchronisation from GPS
This is best done using the a provided pin. Some modules do not expose this pin
to the user, in which case, an averaging method is provided.

#### Method 1: Using the TIMEPULSE Pin
This pin (often labeled TP, TP1, or TIMEPULSE / PPS) pin on u-blox GNSS modules
is a precision timing output, most commonly used for 1 Hz pulse-per-second (PPS)
synchronization.  The pin outputs a square wave that’s aligned to GNSS time
(usually GPS time or UTC).  Its rising (or falling) edge can be used as a
precise timing reference.
```Toit
//todo: complete this example
// Idea - have the user connect a pin to the TIMEPULSE/PPS pin on the driver.
// Expose and send a UBX-CFG-TP5 message, controlling:
// - Frequency (0.25 Hz – 10 MHz, module-dependent)
// - Duty cycle (percentage high)
// - Time reference (GPS, UTC, GLONASS, etc.)
// - Alignment edge (rising/falling)
// - Behavior when GNSS fix is lost (hold last, switch to free-run, disable)
// - Polarity (active high / low)
// Then have the driver use the pin and set time with it only if/when
// drift exceeds a certain amount.  Check with Florian that there isn't
// something already made that could make this significantly easier.

```

#### Method 2: Averaging method
When `UBX-NAV-TIMEUTC` messages are received, they are first checked for
validity (when `.valid-utc=true`).  If valid, the driver immediately measures
the time offset between the time message and the local system time.  As these
messages are continually received, a moving average is calculated with them in
order to smooth out time differences/delays of local processing.  This average is exposed via the driver, and with that the system time can be set, as shown:
```Toit
//todo: complete this example
// Idea - First, clean up serial, increase speed, disable chatty NMEA messages.
// When time is valid, obtain accuracy, and a smart allowance for serialisation
// time.





```
