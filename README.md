# UBlox GNSS
Driver for the u-blox M* GNSS receivers.

## Interface Support
| Interface | Support |
|-|-|
| I2C | Supported. [Toit code example.](https://github.com/toitware/ublox-gnss-driver/blob/main/examples/i2c.toit) |
| Serial | Supported. [Toit code example.](https://github.com/toitware/ublox-gnss-driver/blob/main/examples/serial.toit) |
| SPI | Supported. [Toit code example.]() |

## Device Support
This driver aims to support all UBX-compatible devices, though coverage is
limited by the hardware and chipsets available to us for testing.  The code was
first written for the M8 device, with others added later.

If you have a device that isn’t listed or can donate a device, or help with
testing, please [get in touch](https://discord.gg/Q7Y9VQ5nh2) or open an issue —
contributions and reports are very welcome.
| Device | UBX Protocol Version | Toit Driver Version | Notes |
|-|-|-|-|
| uBlox M8 | `18.00`+ | - | Originally written to support this device. |
| uBlox 7M | `14.5` (Assumed ) | - | Presently Untested |
| uBlox NEO-6M | `14.0` (Assumed)  | - | Operational, undergoing testing. |

## Protocol Support
Driver uses the [`ubx-message`](https://github.com/toitware/ubx-message) protocol, parser to
handle parsing messages from the ublox device.  The devices also support other
open protocols, which will be developed as needed.
| Protocol | Support | Notes |
|-|-|-|
| ubx-message | Yes | Binary, Originally written to support this device.  Open, but proprietary. |
| NMEA | Not yet. | ASCII text sentences (e.g., GGA, RMC) widely used for basic GNSS output. |
| RTCM | Not yet. | Standard for GNSS differential/RTK correction messages. Many u-blox modules accept RTCM input/output. |
