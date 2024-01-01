# ATTINY_tracker
Tx and Rx module for tracking lost rockets, based on Attinys MCUs
The funtion is almost as simple as possible.

# Tx unit
The Attiny85 MCU is programmed to send out a beacon signal with 2 seconds through a standard 433.92Mhz TX module using OOK modulation.
The beacon is a small manchester encoded 1200baud 22byte databurst basically containing unitID, packet session number, CPU temperature and CPU voltage plus some more information. The Attiny85 power off the standard 433.92Mhz module and then disables all internal devices except the watch dog and goes in to sleep mode between the burst. The unitID is generated at first boot if there is none in the EEPROM. A new unidID can be set by pressing reset button when powering on the device. Beacon interval and baudrate can currently only be set when flashing the unit.
Furthermore, the MCU is powered by a button cell(3V) or small lipo package(3.6V) and the TX module is powered by a step up switch for higher voltage nad hence higher TX power. The MCU uses its builtin analog comparator and PWM to step up the 3-3.6V using switch transistor and transformer.

# Rx unit
The Attiny85 MCU is programmed to recieve the manchester encoded data through a standard 433.92Mhz RX module using OOK modulation.
The 433.92Mhz RX module has beside digital output also analog output so the beacon signal can be heard far before the signal strength is high enough in order to recieve the data correctly. The RX module is connected to a 433MHz YAGI antenna in order to find the direction of incoming beacon signal.
If data is correctly recieved, checked by crc16, the data is presented on a small oled display connected to the MCU so one can see what unit was transmitting, temperature and voltage.

# Schemas
Not yet

# How to program ATTiny MCU
This variant uses Arduino IDE and Arduino UNO board for programming the Attiny85 MCU. 
Attiny85 uses a bootloader in order to get the sketches uploaded to flash memory.
Schema of wiring will be added later.

As option we use also Atmel ICE as programmer but it requires building a small additional board with crystal and 6 pin ISP socket. Schema will be added later.
Using Atmel ICE one can also work without bootloader and use Atmel Studion instead of Arduino IDE.

ATTinyCore by Spence Konde is used for these projects.

Additional boards managers used, they are added Arduino IDE -> Preferences -> Additional bords manager URLs:

http://drazzy.com/package_drazzy.com_index.json

https://adafruit.github.io/arduino-board-index/package_adafruit_index.json

https://github.com/mchr3k/arduino-libs-manchester/blob/master/library.json

To select ATTINY85, Arduino IDE -> Tools -> Board -> Boards manager -> Attiny Core -> Attiny 24/45/85 (no bootloader)

Select programmer, Arduino IDE -> Tools -> Programmer -> Arduino UNO ISP (or Atmel ICE if you use that)

To burn bootloader:
Arduino IDE -> File -> Examples -> Arduino ISP
Arduino IDE -> Tools -> Burn Bootloader
