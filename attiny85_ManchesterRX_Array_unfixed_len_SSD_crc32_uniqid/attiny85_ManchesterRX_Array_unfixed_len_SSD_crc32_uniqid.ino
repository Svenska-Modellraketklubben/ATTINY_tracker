// Simple attiny beacon
// Attiny85 manchester rx array unfixed length with crc and unique id and presentation on oled display

// Manchester code used with radio transmitter
// https://github.com/mchr3k/arduino-libs-manchester
#include "Manchester.h"

// CRC check of payload
// https://github.com/RobTillaart/CRC
//#include "CRC32.h"
#include "CRC.h"
#include "CRC16.h"

// my library own for timestamp
#include "timestamp.h"

// Read and Write EEPROM
#include <EEPROM.h>

// Simple I2C test for ebay 128x64 oled.
// To communicate with the oled display, i2c
#include <Wire.h>

// OLED driver
// https://github.com/greiman/SSD1306Ascii
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// oled display i2c address
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;

// 11494 bytes built

/*
Pinlayout of the ATTINY 85
                                         +-------+
             (PCINT5/RESET/ADC0/dW) PB5 -| 1   8 |- VCC
PIN 3 (PCINT3/XTAL1/CLKI/OC1B/ADC3) PB3 -| 2   7 |- PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2) PIN 2 
PIN 4 (PCINT4/XTAL2/CLKO/OC1B/ADC2) PB4 -| 3   6 |- PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1) PIN 1 PWM
                                    GND -| 4   5 |- PB0 (MOSI/DI/SDA/AIN0/OC0A/OC1A/AREF/PCINT0) PIN 0 PWM
                                         +-------+
*/

/*
    OLED    attiny85
    SCL  -  PB2
    SDA  -  PB0
    PB3  -  RxD
    PB4  -  Data recieved green led
*/

/*

  Manchester Receiver example
  
  In this example receiver will receive array of 10 bytes per transmittion

  try different speeds using this constants, your maximum possible speed will 
  depend on various factors like transmitter type, distance, microcontroller speed, ...

  MAN_300 0
  MAN_600 1
  MAN_1200 2
  MAN_2400 3
  MAN_4800 4
  MAN_9600 5
  MAN_19200 6
  MAN_38400 7

*/

#define RX_PIN PB3  //RxD
#define LED_PIN PB4 //Data recieved
//#define LED_PIN_ERROR PB1
#define AVR_SIGNATURE 3 // 3 byte avr signature from device signature imprint table
#define AVR_OSCCAL 2 // 2 byte factor osccal value from device signature imprint table

//first we define our structure data type
//customize this to whatever variable types and names you need
typedef struct settings_t {
  uint8_t size; // packet size
  long unitID; // set when button is pressed during startup
  uint8_t AVRsignature[AVR_SIGNATURE]; // AVR signature
  uint8_t AVRosccal[AVR_OSCCAL]; // AVR osccal
  uint32_t session; // packet session
  int CPUtemp;      // internal cpu temp
  float CPUvolts;   // cpu power voltage
  uint16_t CRC;  // CRC of this package
};

//the packet size will be the number of bytes (1byte = 8bits) of all the variables we defined in the structure above
//in our case it's: 4 bytes (the first 4 variables in the struct) + 2 bytes (the uint16_t is 2*8 bytes) + 4bytes (the float) + 1 byte (the last variable)
const int union_size = sizeof(settings_t);
//NOTE: the actual sizes of the variables MIGHT differ based on your platform, so be very careful when using this
//for cross-platform code (having it run the same on an ESP32 and an Arduino Nano for example)

/* Now we define a union, basically the ways we want to write or read this data
 * in our case we want one way to be the structure above
 * and another way to be a byte array of appropriate size.
 * I named this 'btPacket_t' because I use it for bluetooth, name it whatever you want.
 * You can define as many types inside as you need, just make sure the types you define are all the same size in bytes
 */
typedef union btPacket_t {
 settings_t structure;
 byte byteArray[union_size]; /* you can use other variable types if you want. Like: a 32bit integer if you have 4 8bit variables in your struct */
};

//create a variable using this new union we defined
btPacket_t settings;

const char compile_date[] = __DATE__ " " __TIME__;
const int32_t unix_timestamp = UNIX_TIMESTAMP;
uint16_t crc_calc;

//CRC32 crc;
CRC16 crc;

char *buffer;

void PrintHex83(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
  char tmp[length*2+1];
  byte first ;
  int j=0;
  for (uint8_t i=0; i<length; i++) 
  {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39; 
    else tmp[j] = first;
    j++;
  }
  tmp[length*2] = 0;
  oled.print(tmp);
};

void printHex(long val) {
  //oled.print(F("0x"));
  for (int8_t shift = 8 * sizeof(val) - 4; shift >= 0; shift -= 4) {
    uint8_t hexDigit = (val >> shift) & 0xF;
    oled.print(hexDigit, HEX);
    //if (((shift & 0xF) == 0) && (shift > 0)) {
    //  oled.print(F(":"));
    //}
  }
};

void draw(void) {
  oled.clearToEOL();

  //oled.print(settings.structure.unitID,HEX);
  printHex(settings.structure.unitID);
  oled.print(F(" "));

/*
   for (int addr = 0; addr < sizeof(settings.structure.AVRsignature); addr += 1) {
    //hexstring += String(settings.structure.AVRsignature[addr], HEX);
    oled.print(String(settings.structure.AVRsignature[addr], HEX));
   };
  oled.print(F(" "));

  for (int addr = 0; addr < sizeof(settings.structure.AVRosccal); addr += 1) { 
    //hexstring += String(settings.structure.AVRosccal[addr], HEX);
    oled.print(String(settings.structure.AVRosccal[addr], HEX));
  };
  //oled.print(F("\n"));
*/

  PrintHex83(settings.structure.AVRsignature,sizeof(settings.structure.AVRsignature));
  oled.print(F(" "));
  PrintHex83(settings.structure.AVRosccal,sizeof(settings.structure.AVRosccal));
  oled.print(F("\n"));

  buffer = new char[sizeof(settings.structure.session)*8+1];
  //char buffer[sizeof(settings.structure.session)*8+1];
  itoa(settings.structure.session,buffer,10);
  //oled.print(F(" "));
  oled.print(buffer);
  //oled.print(F("\n"));
  oled.print(F(" "));
  delete[] buffer;

  buffer = new char[sizeof(settings.structure.CPUvolts)*8+1];
  //char buffer1[sizeof(settings.structure.CPUvolts)*8+1];
  dtostrf(settings.structure.CPUvolts,4,2,buffer);
  oled.print(buffer);oled.print(F("V "));
  delete[] buffer;

  buffer = new char[sizeof(settings.structure.CPUtemp)*8+1];
  //char buffer2[sizeof(settings.structure.CPUtemp)*8+1];
  itoa(settings.structure.CPUtemp,buffer,10);
  oled.print(buffer);oled.println(F("C"));
  delete[] buffer;

  //oled.println(String(settings.structure.CRC, HEX));
  //oled.println(String(crc_calc, HEX));
}

void setup() 
{
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, LOW);
//  pinMode(LED_PIN_ERROR, OUTPUT);  
//  digitalWrite(LED_PIN_ERROR, LOW);

  Wire.begin();
  Wire.setClock(400000L);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  #if INCLUDE_SCROLLING == 0
  #error INCLUDE_SCROLLING must be non-zero.  Edit SSD1306Ascii.h
  #endif //  INCLUDE_SCROLLING

  oled.setFont(Adafruit5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);

  oled.clear();

  //oled.println(unix_timestamp);
  oled.println(compile_date);
  //oled.print(F(__DATE__" "));
  //oled.println(F(__TIME__));
  //oled.println(F("Waiting for data!"));
  //oled.println(EEPROM.length());

  //for (int index = 0 ; index < EEPROM.length() ; index++) {
  //  oled.print(index);oled.print(":");oled.println(String(EEPROM.read(index),HEX));
  //}

  //Serial.println(F("Starting"));
  man.setupReceive(RX_PIN, MAN_1200);
  man.beginReceiveArray(union_size, settings.byteArray);
}

void loop() 
{
  //digitalWrite(LED_PIN_ERROR, HIGH);
  if (man.receiveComplete()) 
  {
    //man.stopReceive();

    // Calculate the checksum of payload part
    crc.restart();
    for (int i=0;i < union_size - sizeof(settings.structure.CRC);i++) {
      crc.add(settings.byteArray[i]);
    };
    crc_calc = crc.calc();

    // show result if CRC was okay
    if ((settings.structure.CRC == crc_calc )) {
      // turn on LED to show that data has been received
      digitalWrite(LED_PIN, HIGH);  
      draw();
      digitalWrite(LED_PIN, LOW);
    };
    //digitalWrite(LED_PIN_ERROR, LOW);
    man.beginReceiveArray(union_size, settings.byteArray);
  }
}
