Pico HAB Tracker
========

This is a GPS/radio tracker for high altitude balloons.  It uses a GPS receiver to determine its position, and a LoRa transceiver to transmit that position to the ground.  It is written in C for the Pi Pico microcontroller, using the official C/C++ SDK.

The software was developed on a Raspberry Pi 4 and that is by far the easiest way to get set up for C/C++ development for the Pico.  The resulting executable code can be downloaded to the Pico using a serial programming/debug cable as described in the Pico C/C++ SDK manual.

High Altitude Ballooning
========

Flying a balloon is not a complex task however there is a lot to learn to ensure that your flight is both safe and successful.  Please read this introduction to begin with - http://www.daveakerman.com/?p=1732.

Receiving Telemetry
========

You will need a means of receiving the telemetry from your tracker.  One popular option is the Pi LoRa HAB gateway - https://www.daveakerman.com/?p=1719.  See my blog https://www.daveakerman.com/?page_id=2410 for other options and new developments.

Hardware
========

I built my tracker on a basic padboard to which I mounted the Pico, a UBlox GPS module, a LoRa radio module and connector for the BME280.  **Do not use a breadboard**.  Aside from these things being the work of the devil, able to introduce subtle and not-so-subtle problems at the touch of a wire, but they are totally inappropriate when it comes to flying your tracker which is likely to be shaken violently at balloon burst and at landing.  At sometimes at launch too.

The GPS should be a UBlox module since these are known to work at high altitudes (many other makes stop working above 18km altitude).  The tracker code includes the commands to set the UBlox into high altitude mode (up to 50km, which is plenty),  Make sure you purchase a module intended for 3V3 power.

The LoRa module can be any based on the SX1278 (433/434MHz) chip, provided that the SPI pins (MOSI/MISO etc) and DIO0 are brought out.

The BME280 needs to be mounted outside the payload in order to measure the external temperature.  You can use a moderate length of cable say 30cm to connect from the tracker to the sensor.

Connections
===============

Connect the GPS to the Pico thus:

- Vcc (+ power) connects to the Pico pin 36 (3V3 Out)
- GND (0V) connects to any Pico GND pin 
- TXD/TX connects to the Pico pin 7 (UART0 RX)
- RXD/RX connects to the Pico pin 6 (UART0 TX)



Connect the LoRa module to the Pico thus:

- Vcc (+ power) connects to the Pico pin 36 (3V3 Out)
- GND (0V) connects to any Pico GND pin 
- NSS connects to the Pico pin 22 (SPI0 CSn)
- MOSI connects to the Pico pin 25 (SPI0 TX)
- MISO connects to the Pico pin 21 (SPI0 RX)
- SCK connects to the Pico pin 24 (SPI0 SCK)
- DIO0 connects to the Pico pin 29 (GP22)



Connect the BME280 to the Pico thus:

- Vcc (+ power) connects to the Pico pin 36 (3V3 Out)
- GND (0V) connects to any Pico GND pin 
- SCL connects to the Pico pin 17 (I2C0 SCL)
- SDA connects to the Pico pin 16 (I2C0 SDA)

(a trailing <LF\> can be sent but is ignored).  Accepted commands are responded to with an OK (* <CR\> <LF\>) and rejected commands (unknown command, or invalid command value) with a WTF (? <CR\> <LF\>)

Commands that set radio parameters are in upper case for the receiver or lower case for the transmitter.  e.g. ~F434.250 sets the receive frequency to 434.250, and ~f434/450 sets the transmit frequency to 434.450.

The radio commands (upper case for receiver, lower case for transmitter) are:

	- F<frequency in MHz>
	- B<bandwidth>
	- E<error coding from 5 to 8>
	- S<spreading factor from 6 to 11>
	- I<1=implicit mode, 0=explicit mode>
	- L(low data rate optimisation: 1=enable, 0=disable)

The other configuration commands are:

	- P<PPM>
	- R<1=repeating on, 0=repeating off>

Finally, "~!" stores the current settings in EEPROM, "~^" resets all settings to those in EEPROM, and then lists those settings, "~*" resets all settings to defaults, stores those in EEPROM and lists them, and "~?" lists the current values of all settings.

Bandwidth value strings can be 7K8, 10K4, 15K6, 20K8, 31K25, 41K7, 62K5, 125K, 250K, 500K

History
=======

02/03/2020	V1.0



