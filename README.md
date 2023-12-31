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
