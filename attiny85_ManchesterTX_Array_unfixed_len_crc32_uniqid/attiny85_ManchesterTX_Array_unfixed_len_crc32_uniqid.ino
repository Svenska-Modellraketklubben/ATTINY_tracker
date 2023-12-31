// Manchester code used with radio transmitter
#include <Manchester.h>
// Get CPU chip id
//#include <ArduinoUniqueID.h>
// sleep functions
#include <avr/sleep.h>
// power modes
#include <avr/power.h>
// watchdog timer
#include <avr/wdt.h>
// analog comparator
#include "analogComp.h" // causes conflict with ArduinoUniqueID.h
// various macros like signature id is defined here
#include <avr/io.h>
// variuos low lever functions like boot_signature_byte_get() and read fuses etc.
#include <avr/boot.h>
// 
//#include "CRC32.h"
#include "CRC16.h"
#include "CRC.h"

//my own for timestamp
#include "timestamp.h"
#include "random.h"

// Read and Write EEPROM
#include <EEPROM.h>

/*
Pinlayout of the ATTINY 85
                                         +-------+
             (PCINT5/RESET/ADC0/dW) PB5 -| 1   8 |- VCC
PIN 3 (PCINT3/XTAL1/CLKI/OC1B/ADC3) PB3 -| 2   7 |- PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2) PIN 2 
PIN 4 (PCINT4/XTAL2/CLKO/OC1B/ADC2) PB4 -| 3   6 |- PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1) PIN 1 PWM
                                    GND -| 4   5 |- PB0 (MOSI/DI/SDA/AIN0/OC0A/OC1A/AREF/PCINT0) PIN 0 PWM
                                         +-------+
*/

// PB3 pin where your transmitter is connected
// PB2 pin powering up transmitter
// PB0 unused
// PB1 AIN1 analog comparator
// PB4 PWM

/*

  Manchester Transmitter example
  
  In this example transmitter will send 10 bytes array  per transmittion

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

#define SETTLE    100 // 100ms voltage reference mux switching settle time
#define BEACON_T  7   // beacon interval, deep sleep in between, see table below
#define TX_PIN    PB3   //pin where your transmitter is connected
#define POWER_RFD PB2   //pin powering up transmitter
#define AVR_SIGNATURE 3 // 3 byte avr signature from device signature imprint table
#define AVR_OSCCAL 2 // 2 byte factor osccal value from device signature imprint table

// device id, build date in epoch + some random value

//  *** Define the time interval in milliseconds ***
//const unsigned long TIMER0_COUNT = 100;    // 100 msec timer interval

// https://github.com/jordan-public/Thermometer-Attiny85/blob/master/Thermometer-Attiny85.ino
// http://21stdigitalhome.blogspot.com/2014/10/trinket-attiny85-internal-temperature.html
// https://andrey.mikhalchuk.com/2011/06/20/reading-attiny854525-internal-temperature-sensor.html
// http://www.technoblogy.com/show?2G9S

/* Arduino data manipulation and concatenation with structures and unions
 * 
 * This simple (ish) sketch shows you how to organize your data into a struct
 * and then access the whole thing as a byte array.
 *
 * Useful if you want to concatenate several variables into a single byte array
 * to send over bluetooth, i2c, lora or any other protocol that works with arrays. 
 * In other words you have a fixed byte array, and we squeeze in variables of different
 * data types and lengths into it, while still using it as a byte array afterwards.
 * 
 * The way we use a union here (in plain terms) is: the union is a fixed region of memory that we
 * can write to in one format (as a structure) and then read from in another format (a byte array).
 * You can do it in reverse as well, write to it as a byte array, and read it as a structure.
 * 
 * Thanks to this thread https://forum.arduino.cc/index.php?topic=271048.0 for helping me figure it out.
 * NOTE: be careful (ab)using structures for cross-platform code, variable sizes in bytes might vary causing havoc !!!!
 */

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

// https://www.re-innovation.co.uk/docs/sleep-modes-on-attiny85/
// Variables for the Sleep/power down modes:
volatile boolean f_wdt = 1;

//CRC32 crc;
CRC16 crc;

// for unitID
typedef struct unitID_t {
  long unitID;
  long notunitID;
  uint32_t timestamp;
};

const int unitID_size = sizeof(unitID_t);

typedef union unit_ID_t {
 unitID_t unit_ID;
 byte byteArray[unitID_size]; /* you can use other variable types if you want. Like: a 32bit integer if you have 4 8bit variables in your struct */
};

unit_ID_t myunitID;

//const char compile_date[] = __DATE__ " " __TIME__;
const int32_t unix_timestamp = UNIX_TIMESTAMP;

void setup() {
  // put your setup code here, to run once:

  // https://robotics.stackexchange.com/questions/1753/assigning-serial-number-and-guid-to-a-microcontroller
  // get/set unitID
  set_id();
  settings.structure.unitID = myunitID.unit_ID.unitID;

  // https://www.instructables.com/ATTiny-Port-Manipulation/
  // https://www.best-microcontroller-projects.com/attiny-ultra-low-power.html
  // https://www.electronics-lab.com/project/attiny85-push-button-power-switching-software-solution/
  //pinMode(PB0, INPUT_PULLUP);          // lower power consumption unused
  //pinMode(PB1, INPUT_PULLUP);          // lower power consumption later AIN1 analog comparator
  //pinMode(PB2, INPUT_PULLUP);          // lower power consumption later POWER_RFD
  //pinMode(PB3, INPUT_PULLUP);          // lower power consumption later TX_PIN
  //pinMode(PB4, INPUT_PULLUP);          // lower power consumption later PWM
  //pinMode(PB5, INPUT_PULLUP);          // lower power consumption normally cpu reset button
  
  for (int PBn; PBn < 6 ; PBn++) {  // PB0 to PB5
    pinMode(PBn, INPUT_PULLUP);     // lower power consumption
  };

 // Bootloader configured to run ATtiny85@1MHz internal clock,lower clock yields less power consumption   
  sleep_adc_disable();                 // saves power since ADC not used thus disabled
  sleep_bod_disable();                 // saves more power and provides longer 
                                      // operation by draining battery deep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down everything, except WDT
  sleep_enable();                      // Short naps are allowed between blinks

  rf_433mhz_init(); // enable 433 MHz Tx driving GPIO pin as Output

  settings.structure.size = union_size; // packet size
  settings.structure.session = (uint32_t) 0; // at power on packets begin at session zero

  // https://www.avrfreaks.net/s/topic/a5C3l000000UZmoEAG/t151356
  // https://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html#gaf375d2543ba38dc56697b4f4bc37a717
  // https://electronics.stackexchange.com/questions/31048/can-an-atmega-or-attiny-device-signature-be-read-while-running
  // uint8_t first =  boot_signature_byte_get( 0x00 ); // read device id 
  // https://forum.arduino.cc/t/get-cpu-id/110665/11
  // https://forum.arduino.cc/t/did-you-know-fuses-and-signature-bytes-can-be-read-out-live/304773
  // https://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html#gaf375d2543ba38dc56697b4f4bc37a717
  // https://electronics.stackexchange.com/questions/31048/can-an-atmega-or-attiny-device-signature-be-read-while-running

  //UniqueIDdump(Serial); // Read CPU chip id
  //memcpy(settings.structure.CPUid, UniqueID, UniqueIDsize); // set cpuid, same all the time

  //settings.structure.CPUid[0] = boot_signature_byte_get (0x00); //  Signature byte 0 
  //settings.structure.CPUid[1] = boot_signature_byte_get (0x01); //  Calibration data for internal oscillator at 8.0 MHz
  //settings.structure.CPUid[2] = boot_signature_byte_get (0x02); //  Signature byte 1 
  //settings.structure.CPUid[3] = boot_signature_byte_get (0x03); //  Calibration data for internal oscillator at 6.4 MHz
  //settings.structure.CPUid[4] = boot_signature_byte_get (0x04); //  Signature byte 2 
  // get AVR signature
	for (int addr = 0; addr < 3; addr += 1) {
		// signature bytes are at 0, 2 and 4. OSCCAL is at 1
		settings.structure.AVRsignature[addr] = boot_signature_byte_get(addr * 2);
	}
  // get OSCCAL values is at 1, 3
  int i = 0;
  for (int addr = 1; addr < 4; addr += 2) {
    settings.structure.AVRosccal[i++] = boot_signature_byte_get(addr);
  }

  // https://electronics.stackexchange.com/questions/31048/can-an-atmega-or-attiny-device-signature-be-read-while-running
  //byte x = SIGNATURE_0; // same as above 0,2,4!
  //byte y = SIGNATURE_1;
  //byte z = SIGNATURE_2;

  analogComparator.setOn(INTERNAL_REFERENCE, AIN1); //we instruct the lib to use voltages on the pins
  //analogComparator.enableInterrupt(changeStatus, CHANGE); //we set the interrupt and when it has to be raised

  //pinMode(PB0, OUTPUT); // ISR period test
  //digitalWrite(PB0,LOW);

  // enable periodic interrupts on timer 0, default about 500Hz
  setup_isr(); // timer 0 periodic interrupts enabled to adjust pwm if needed
 
  // setup fast pwm on timer 1, about 30Khz
  setup_pwm(); // timer 1 enable pwm for step up regulator switch transistor

  //man.workAround1MhzTinyCore(); //add this in order for transmitter to work with 1Mhz Attiny85/84
  man.setupTransmit(TX_PIN, MAN_1200); // Define manchester TX pin and baudrate.
}

void loop() {
  // put your main code here, to run repeatedly:
  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag  
    // transmit the array settings.byteArray
    //rf_short_blink(union_size, settings);
    rf_short_blink();

    // all ports input to lower power consumption
    pinMode(PB4, INPUT);          // lower power consumption later PWM, if not PB4 used pullup and is high during sleep.
    
    // deep sleep mode
    no_blink_long(BEACON_T);
    
     // all ports activated again
    pinMode(PB4, OUTPUT);          // lower power consumption later PWM
  }
}
//////// Void Loop Ends Here ///////

//////////////////////////////////////////
/////////// Functions Body ///////////////
//////////////////////////////////////////

// setup fast pwm on timer 1, about 30Khz
void setup_pwm(void) {
  pinMode (PB4, OUTPUT);  // chip pin 3  // OC1B - Timer 1 "B"
  noInterrupts();
  // Timer 1
  TCCR1 = bit (CS10);           // no prescaler, otherwise 500hz
  GTCCR = bit (COM1B1) | bit (PWM1B);  //  clear OC1B on compare
  OCR1B = 127;                   // duty cycle (25%) 31
  OCR1C = 255;
  //TIMSK = 1 << OCIE1B; // enable interrupt, not used to fast using timer 0 instead
  interrupts();
}

// enable interrupts on timer 0, default about 500Hz
void setup_isr(void) {
  // Timer 0
  //TCCR0A=0x00;
  //TCCR0B=0x00;
  //TCCR0B |= (1<<CS00)|(1<<CS02);   //prescaling with 1024
  //TCCR0A|=(1<<WGM01);//toggle mode and compare match  mode
  OCR0A= 161; //compare value
  //TCNT0=0;
  sei();   //enabling global interrupt
  TIMSK|=(1<<OCIE0A);
}

void rf_433mhz_init(void)
{
   pinMode(POWER_RFD,OUTPUT);
   digitalWrite(POWER_RFD,LOW);
}

void powerup_rf_433mhz(void)
{
   digitalWrite(POWER_RFD,HIGH);  
   delay(25);
}

void powerdown_rf_433mhz(void)
{
   digitalWrite(POWER_RFD,LOW);  
   delay(6);
}

// not used
//void rf_tx_data(int union_size, btPacket_t settings)
//{
//  // transmit the array settings.byteArray
//  powerup_rf_433mhz(); // Power on transmitter
//  man.transmitArray(union_size, settings.byteArray); // transmit
//  powerdown_rf_433mhz(); // Power off transmitter
//  //setup_watchdog(4);     // 250 ms length for watchdot timer
//  //sleep_mode();          // sleep until watchdog barks
//  //wdt_disable();         // hush the dog after waking up from sleep
//}

//void rf_short_blink(int union_size, btPacket_t settings)
void rf_short_blink(void)
{
    wdt_disable();

    //store some data in our union.
    //we're treating it as a structure in this case for easy writing
    //we could also do something like "settings.byteArray[1] = 22;" 
    //you can treat this memory region as any data type we define in our union
    //adc_enable();

    settings.structure.CPUvolts = getChip_V(); // cpu chip voltage
    settings.structure.CPUtemp = getChipTemperatureCelsius(getChip_T(),settings.structure.CPUvolts); // cpu chip temperature
    //sleep_adc_disable();

    // Calculate the checksum of payload part
    crc.restart();
    for (int i=0;i < union_size - sizeof(settings.structure.CRC);i++) {
      crc.add(settings.byteArray[i]);
    };
    settings.structure.CRC = crc.calc();

    powerup_rf_433mhz();
    man.transmitArray(union_size, settings.byteArray); // transmit
    powerdown_rf_433mhz();
    settings.structure.session++; // Count up session
}

void sleep_adc_disable(void)
{
   ADCSRA &= ~(1<<ADEN); //Disable ADC, saves power
}

// not used, instead enabled and disabled inside function need ADC
//void adc_enable (void)
//{
//  ADCSRA |= (1<<ADEN);  //Enable ADC if needed, not used here
//}

void no_blink_long(int wdt_time)
{
// 1 second sleeping
    setup_watchdog(wdt_time);   // (6) 1000 ms length for watchdog timer
    // https://gist.github.com/AlainGourves/1da7c19385f4fb52e76ce467b8147db5#file-attiny85_wdt-ino-L28
    // https://forum.arduino.cc/t/power-down-consumption-on-custom-board/53907/5
    // https://www.best-microcontroller-projects.com/attiny-ultra-low-power.html
    // https://www.avrfreaks.net/s/topic/a5C3l000000Uln3EAC/t193460
    // https://forum.arduino.cc/t/power-down-consumption-on-custom-board/53907/6
    //PRR = 0xFF;          // Power Reduction Register
    power_all_disable();
    int sPRR=PRR;
    PRR=0xFF;
    sleep_mode();        // sleep until watchdog barks
    PRR=sPRR;
    power_all_enable();
    wdt_disable();       // hush the dog after waking up from sleep
}

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec

// http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/

void setup_watchdog(int timerPrescaler) 
{
//Limit incoming amount to legal settings
  if (timerPrescaler > 9 ) timerPrescaler = 9;   
  byte bb = timerPrescaler & 7; 
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary

/////////// This order of commands is important and cannot be combined ////////
  MCUSR &= ~(1<<WDRF);             //Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE);   //Set WD_change enable, set WD enable
  WDTCR = bb;                      //Set new watchdog timeout value
  WDTCR |= _BV(WDIE);              //Set the interrupt enable, this will keep unit from resetting after each int
}


// From: http://21stdigitalhome.blogspot.com/2014/10/trinket-attiny85-internal-temperature.html

float getChip_T() {
  int i;
  int t_celsius; 
  uint8_t vccIndex;
  float rawTemp, rawVcc;
  
  // Measure temperature
  ADCSRA |= _BV(ADEN);           // Enable AD and start conversion
  ADMUX = 0xF | _BV( REFS1 );    // ADC4 (Temp Sensor) and Ref voltage = 1.1V;
  delay(SETTLE);                 // Settling time min 1 ms, wait 100 ms

  rawTemp = (float)getADC();     // use next sample as initial average
  for (int i=2; i<32; i++) {   // calculate running average for 2000 measurements
    rawTemp += ((float)getADC() - rawTemp) / float(i); 
  }
  ADCSRA &= ~(_BV(ADEN));        // disable ADC  
  return rawTemp;
}

float getChip_V() {
  int i;
  int t_celsius; 
  uint8_t vccIndex;
  float rawTemp, rawVcc;

// Measure chip voltage (Vcc)
  ADCSRA |= _BV(ADEN);   // Enable ADC
  ADMUX  = 0x0c | _BV(REFS2);    // Use Vcc as voltage reference, 
                                 //    bandgap reference as ADC input
  delay(SETTLE);                    // Settling time min 1 ms, there is 
                                 //    time so wait 100 ms
  rawVcc = (float)getADC();      // use next sample as initial average
  for (int i=2; i<32; i++) {   // calculate running average for 2000 measurements
    rawVcc += ((float)getADC() - rawVcc) / float(i);
  }
    rawVcc = 1024 * 1.1f / rawVcc;
    ADCSRA &= ~(_BV(ADEN));        // disable ADC  
    return rawVcc;
}

int getChipTemperatureCelsius(float rawTemp, float rawVcc) {
  //int i;
  int t_celsius; 
  uint8_t vccIndex;
  //float rawTemp, rawVcc;
  
  //index 0..13 for vcc 1.7 ... 3.0
  vccIndex = min(max(17,(uint8_t)(rawVcc * 10)),30) - 17;   

  // Temperature compensation using the chip voltage 
  // with 3.0 V VCC is 1 lower than measured with 1.7 V VCC 
  t_celsius = (int)(chipTemp(rawTemp) + (float)vccIndex / 13);  
                                                                                   
  return t_celsius;
}

// Calibration of the temperature sensor has to be changed for your own ATtiny85
// per tech note: http://www.atmel.com/Images/doc8108.pdf
float chipTemp(float raw) {
  //const float chipTempOffset = 272.9;           // Your value here, it may vary 
  const float chipTempOffset = 252.0;
  const float chipTempCoeff = 1.075;            // Your value here, it may vary
  return((raw - chipTempOffset) / chipTempCoeff);
}
 
// Common code for both sources of an ADC conversion
int getADC() {
  ADCSRA  |=_BV(ADSC);           // Start conversion
  while((ADCSRA & _BV(ADSC)));    // Wait until conversion is finished
  return ADC;
}

// check if unitID is in eeprom, if not try to generate it
void set_id() {
  int eeAddress = 0;
  EEPROM.get(eeAddress, myunitID);

  if ((bitRead(MCUSR, EXTRF) and (bitRead(MCUSR, BORF) or bitRead(MCUSR, PORF))) and not bitRead(MCUSR, WDRF)) {
    // 1. after reprogramming the device
    // 2. reset logical low during power on
    if ((myunitID.unit_ID.timestamp != unix_timestamp) and (myunitID.unit_ID.unitID != 0xFFFFFFFF)) {
        // uploaded program timestamp differs from stored in eeprom and eeprom contains data -> keep unitID + update timestamp
        myunitID.unit_ID.timestamp = unix_timestamp;
        EEPROM.put(eeAddress, myunitID);
    } else {
        // uploaded program timestamp differs from stored in eeprom and eeprom empty -> set/update unitID + timestamp
        myunitID.unit_ID.unitID = myunitID.unit_ID.notunitID = 4711;
    };
  };
  MCUSR = 0;

  // generate new unitId if triggered by 4711 and save it to EEPROM
  if (!(myunitID.unit_ID.unitID == -myunitID.unit_ID.notunitID)) {
    myunitID.unit_ID.timestamp = unix_timestamp;
    randomSeed(generateRandomSeed()+unix_timestamp);
    myunitID.unit_ID.unitID = random();
    myunitID.unit_ID.notunitID = -myunitID.unit_ID.unitID;
    EEPROM.put(eeAddress, myunitID);
  };
};


//////////Interrupt Service Routines ///////////////////
/////////////////////// ISR ////////////////////////////

//// ISR runs when Watchdog expires ////

ISR(WDT_vect)
{
            // need to do nothing here  
  f_wdt=1;  // set global flag
}

// https://www.randseq.org/2017/02/attiny85-timer-programming-using-timer1.html
// https://arduino.stackexchange.com/questions/60130/does-enabling-the-timer1-comparea-interrupt-instantly-triggers-an-interrupt

// handle interrupts from timer 1, currently not used, to fast
/*
ISR(TIMER1_COMPB_vect)
{
  OCR1B++;
  if (OCR1B>255) {OCR1B=5;};
  //digitalWrite(PB0, !digitalRead(PB0)); // test
}
*/

// handle interrupts from timer 0
ISR(TIMER0_COMPA_vect)
{
  //static unsigned long count = 0;
  //if ( ++count > TIMER0_COUNT )
  //{
    //digitalWrite(PB0, !digitalRead(PB0)); // test toggle
    //count = 0;
     // min max pwm values for duty cycle
    OCR1B = constrain(OCR1B,5,200);
    // Check analog comparator if pwm need to be adjusted
    if ( ACSR & (1<<ACO) ) { OCR1B++;} else { OCR1B--;};
  //}
  //digitalWrite(PB0, !digitalRead(PB0)); // test toggle
}