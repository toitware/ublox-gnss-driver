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
