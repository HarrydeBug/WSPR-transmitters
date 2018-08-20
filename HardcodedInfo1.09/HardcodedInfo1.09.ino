//Software for the Zachtek WSPR-TX_LP1 product. Product number 1011.  This board is also known as "WSPR Transmitter Experimenters version"
//There are two different flawors of firmware for this board.
//1: This code with hardcoded Callsign and Power data that just transmitt on one single band. It is smaller in size and simpler in many ways so for
//   experimenters that want to modify the code, expand, add sensors etc etc, this might be easier to use.
//2: The regular version that is designed to be used with a PC software to set all the data like callsign, power level, band hopping etc
//   To acheive this it uses a Serial API and stores settings in EEPROM. It can do more but it is much bigger and can be hard to understand for the beginner.
//The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
//
//To compile you need to install several libraries, se below in the #include section what they are and download them using The Library manager in the Arduino IDE or use the direct URL
//For Arduino Pro Mini 3.3V 8MHz.
//
//  Version History
//  1.05 initial Release
//  1.06 Disabled transmission if callsing was not changed (variable "call[]" on row 115 sets callsign), added TXPause variable to set duty cycle of transmission.
//  1.07 Added more predefined frequency bands every band from 2190m to 4m are now defined
//  1.08 Moved the define for Reference Frequency to line 124 to make it easier to find
//  1.09 Added Bandhoping and a better RandomSeed
//
//The WSPR coding is based on Jason Mildrums example code. See his orgiginal Copyright text below the line.
//Harry "deBug" Zachrisson of ZachTek 2018
//-----------------------------------------------------------------
// WSPR beacon for Arduino, by Jason Milldrum NT7S.
// Original code based on Feld Hell beacon for Arduino by Mark
// Vandewettering K6HX, adapted for the Si5351A by Robert
// Liesenfeld AK6L <ak6l@ak6l.org>.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.

#include <TinyGPS++.h>       //TinyGPS++ library by Mikal Hart https://github.com/mikalhart/TinyGPSPlus
#include <JTEncode.h>        //JTEncode  by NT7S https://github.com/etherkit/JTEncode
#include <SoftwareSerial.h>  //Arduino SoftSerial 


// Data structures
enum E_Band
{
  LF2190m = 0,
  LF630m = 1,
  HF160m = 2,
  HF80m =  3,
  HF40m =  4,
  HF30m =  5,
  HF20m =  6,
  HF17m =  7,
  HF15m =  8,
  HF12m =  9,
  HF10m =  10,
  HF6m =   11,
  VHF4m =  12
};


void i2cInit();
uint8_t i2cSendRegister(uint8_t reg, uint8_t data);
uint8_t i2cReadRegister(uint8_t reg, uint8_t *data);

#define I2C_START 0x08
#define I2C_START_RPT 0x10
#define I2C_SLA_W_ACK 0x18
#define I2C_SLA_R_ACK 0x40
#define I2C_DATA_ACK 0x28
#define I2C_WRITE 0b11000000
#define I2C_READ 0b11000001
#define SI5351A_H

#define SI_CLK0_CONTROL 16 // Register definitions
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_PLL_RESET 177

#define SI_R_DIV_1 0b00000000 // R-division ratio definitions
#define SI_R_DIV_2 0b00010000
#define SI_R_DIV_4 0b00100000
#define SI_R_DIV_8 0b00110000
#define SI_R_DIV_16 0b01000000
#define SI_R_DIV_32 0b01010000
#define SI_R_DIV_64 0b01100000
#define SI_R_DIV_128 0b01110000

#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000




//defines for TX Frequeny of WSPR on the HAM bands
#define WSPR_TONE_SPACING       146           // ~1.46 Hz  Tone spacng in centiHz
#define WSPR_DELAY              683          // Delay value for WSPR  delay in milliseconds

#define WSPR_FREQ4m             7009250000ULL   //4m     70.092,500MHz 
#define WSPR_FREQ6m             5029450000ULL   //6m     50.294,500MHz 
#define WSPR_FREQ10m            2812620000ULL   //10m    28.126,200MHz
#define WSPR_FREQ12m            2492620000ULL   //12m    24.926,200MHz 
#define WSPR_FREQ15m            2109620000ULL   //15m    21.096.200MHz  
#define WSPR_FREQ17m            1810610000ULL   //17m    18.106,100MHz
#define WSPR_FREQ20m            1409710000ULL   //20m    14.097,100MHz 
#define WSPR_FREQ30m            1014020000ULL   //30m    10.140,200MHz  
#define WSPR_FREQ40m             704010000ULL   //40m     7.040,100MHz 
#define WSPR_FREQ80m             359410000ULL   //80m     3.394,100MHz
#define WSPR_FREQ160m            183810000ULL   //160m    1.838,100MHz
#define WSPR_FREQ630m             47570000ULL   //630m      475.700kHz
#define WSPR_FREQ2190m            13750000ULL   //2190m     137.500kHz

bool TXOnBand [13];    //Arraycount corresponds to the Enum E_Band, True =Transmitt Enabled, False = Transmitt disabled on this band  E.g to transmitt on 40m and 20m set TXOnBand[4] and TXOnBand[6] to true.
int CurrentBand = 0;   //Keeps track on what band we are currently tranmitting on
// Hardware defines
#define LEDIndicator1 4  //Yellow LED to indicator that blinks at different rates depending on GPS Lock, waiting for time to go to top of minute, Transmitting  etc
#define TransmitLED 8    //Red LED next to RF out SMA that will turn on when Transmitting

const char softwareversion[] = "1.07" ; //Version of this program, sent to serialport at startup


// Class instantiation
JTEncode jtencode;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial GPSSerial(2, 3); //GPS Serial port, RX on pin 2, TX on pin 3

void si5351aOutputOff(uint8_t clk);
void si5351aSetFrequency(uint32_t frequency);

//*********************************************************************************************
//----------------------- Set Transmit Frequency and Amateur call sign here--------------------
//*********************************************************************************************
char call[] = "AA0AAA";                          //**********Set your call sign here **************
uint8_t dbm = 23;                                //Output power in dBm. The WSPR-TX gives out about 24dBm after Low Pass filtering so 23 is closest value in WSPR coding, change this if you have an attenuater or amplifier connected
unsigned long TXPause = 120;                      //numer of seconds to pause after a transmission cycle (after all enabled bands have been transmitted on) if you TX on a single band then I recomend a value of at least 120 here.  
char loc[] =  "AB01";                            //If you dont want to use the GPS provided locator you can hard code a locator here, 4 chars only. You will still need the GPS for timing - Value is used if LocatorByGPS=false. Otherwise it will be owerwritten by GPS routine
boolean LocatorByGPS = true;                     //Set to false to disable automatic location by GPS and switch to use the "loc" value
#define XTAL_FREQ 26000000                       // Crystal frequency for a certain board in Hertz, change to fit your induvidual TCXO 

//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
char MHLocator[] = "AA00BB";// Dont change, this is only here to allocate memory, will be changed by the Maidenhead calculator.
uint8_t tx_buffer[180];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
uint64_t freq;

void setup()
{
  //***********************************************************************************************
  TXOnBand [0] = false; //LF2190m                  // Set to true on each band you want to transmit on
  TXOnBand [1] = false; //LF630m
  TXOnBand [2] = false; //HF160m
  TXOnBand [3] = false; //HF80m
  TXOnBand [4] = false; //HF40m
  TXOnBand [5] = false; //HF30m
  TXOnBand [6] = true; //HF20m
  TXOnBand [7] = true; //HF17m
  TXOnBand [8] = false; //HF15m
  TXOnBand [9] = false; //HF12m
  TXOnBand [10] = false; //HF10m
  TXOnBand [11] = false; //HF6m
  TXOnBand [12] = false; //VHF4m
//************************************************************************************************
  // Set the proper frequency, tone spacing, symbol count, and
  // tone delay depending on mode

  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;

  bool i2c_found;
  //Initialize Arduinos Serial port
  Serial.begin (9600);
  delay(500);//Wait for Serialport to be initialized properly
  GPSSerial.begin(9600); // //The GPS Serial port
  //Output Initialization text on the serial port

  Serial.print(F("Zachtek WSPR-TX_LP1 -Hard Codeed Info- Software version: "));
  Serial.println(softwareversion);
  Serial.println(F("Initializing.."));
  Serial.print(F("Callsign set to "));
  Serial.println(call);
  Serial.println(F("Maidenhead postion will be set by GPS"));
  Serial.print(F("Power Data set to "));
  Serial.print(dbm);
  Serial.println(F("dBm"));
  if (NoBandEnabled ()) {
    Serial.println (F("Warning, configuration error, tranmission is not enabled on any band!"));
  }
  else
  {
    Serial.print (F("Transmission enabled on the following bands:"));
    for (int FreqLoop = 0; FreqLoop < 13; FreqLoop++) {
      if (TXOnBand[FreqLoop]) {
        switch (FreqLoop) {
          case 0:
            Serial.print (F(" 2190m,"));
            break;
          case 1:
            Serial.print (F(" 630m,"));
            break;
          case 2:
            Serial.print (F(" 160m,"));
            break;
          case 3:
            Serial.print (F(" 80m,"));
            break;
          case 4:
            Serial.print (F(" 40m,"));
            break;
          case 5:
            Serial.print (F(" 30m,"));
            break;
          case 6:
            Serial.print (F(" 20m,"));
            break;
          case 7:
            Serial.print (F(" 17m,"));
            break;
          case 8:
            Serial.print (F(" 15m,"));
            break;
          case 9:
            Serial.print (F(" 12m,"));
            break;
          case 10:
            Serial.print (F(" 10m,"));
            break;
          case 11:
            Serial.print (F(" 6m,"));
            break;
          case 12:
            Serial.print (F(" 4m,"));
        }
      }//If
    }//For
    Serial.println("");

    Serial.println(F("Initializing.."));
    pinMode(TransmitLED, OUTPUT);
    pinMode(LEDIndicator1, OUTPUT);
    Serial.println(F("Blinking Yellow LED indicator"));
    for (int i = 0; i <= 30; i++) {
      digitalWrite(LEDIndicator1, HIGH);
      delay (20);
      digitalWrite(LEDIndicator1, LOW);
      delay (20);;
    }


    Serial.println(F("Initializing I2C"));
    i2cInit();
    Serial.println(F("Initializing Si5351"));
    si5351aSetFrequency(2000000000ULL);
    si5351aOutputOff(SI_CLK0_CONTROL);

    Serial.println(F("Initialization is complete."));
    Serial.println();
    NextFreq();
    freq = freq + (100ULL * random (-100, 100)); //modify TX frequency with a random value beween -100 and +100 Hz 
    random(RandomSeed());
   
  }//Else

}

void loop()
{
  //static double Lat;
  //static double Lon;
  static int GPSH;
  static int GPSM;
  static int GPSS;
  //encode ();

  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated())
    {
      GPSH = gps.time.hour();
      GPSM = gps.time.minute();
      GPSS = gps.time.second();
      calcLocator (gps.location.lat(), gps.location.lng());
      //Serial.print (F("GPS Fix accuired - Maidenhead locator is "));
      //Serial.print(MHLocator);
      loc[0] = MHLocator[0]; loc[1] = MHLocator[1]; loc[2] = MHLocator[2]; loc[3] = MHLocator[3];
      //Serial.print (F(" (using "));
      //Serial.print(loc);

      //UpdatePos();
      //  long seed = (long) (((long)Lat * 10000) + ((long)Lon * 10000));
      //  randomSeed(seed);



      if ((GPSS == 0) && ((GPSM % 2) == 0))//If second is zero at even minute then start WSPR transmission
      {
        // Encode the message in the transmit buffer
        set_tx_buffer();
        Serial.println(F("Top of even minute."));
        Serial.print(F("Transmitting for 1 Minute and 50 seconds at frequency :"));
        Serial.print(uint64ToStr(freq / 100, false));
        Serial.println(F("Hz"));
        encode ();

        if (LastFreq ())
        {
          Serial.print(F("Pausing transmission for "));
          Serial.print(TXPause);
          Serial.println(F(" seconds"));
          smartdelay(TXPause*1000UL);//Pause for some time to give a duty cycle on the transmit. 2000=100%, 20000=50%, 130000=33%
        }
        NextFreq();
        freq = freq + (100ULL * random (-100, 100)); //modify TX frequency with a random value beween -100 and +100 Hz 

        smartdelay(3000);
        Serial.println (F("Waiting for start of even minute to start a new WSPR transmission block"));
      }
      else //Dubble-blink to indicate waiting for top of minute
      {
        if (GPSH<10) Serial.print (F("0"));
        Serial.print (GPSH);
        Serial.print (F(":"));
        if (GPSM<10) Serial.print (F("0"));
        Serial.print (GPSM);
        Serial.print (F(":"));
        if (GPSS<10) Serial.print (F("0"));
        Serial.print (GPSS);
        Serial.print (F(" - "));
        Serial.print(loc);
        Serial.println( );
        //Serial.println (F("Waiting for WSPR timeslot."));
        digitalWrite(LEDIndicator1, HIGH);
        smartdelay (100);
        digitalWrite(LEDIndicator1, LOW);
        smartdelay (100);
        digitalWrite(LEDIndicator1, HIGH);
        smartdelay (100);
        digitalWrite(LEDIndicator1, LOW);
        smartdelay(300);
      }
    }
    if (gps.location.isValid())
    {

    }
    else
    {
      // //singelblink to indicate waiting for GPS Lock
      digitalWrite(LEDIndicator1, HIGH);   // turn the LED on
      smartdelay(100);
      digitalWrite(LEDIndicator1, LOW);    // turn the LED off
      Serial.println(F("Waiting for GPS fix. "));
      smartdelay(500);
    }
  }
}



// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;
  unsigned long startmillis;
  unsigned long endmillis;
  boolean TXEnabled = true;

  // Send WSPR for two minutes
  digitalWrite(LEDIndicator1, HIGH);
  if ((call[0] == 'A') && (call[1] == 'A') && (call[2] == '0') && (call[3] == 'A') && (call[4] == 'A') && (call[5] == 'A')) //Do not actually key the transmitter if the callsign has not been changed from the default one AA0AAA
  {
    Serial.println(F("Callsign is not changed from the default one, Transmit is disabled"));
    Serial.println(F("Recompile this software with your Callsign to enable transmission"));
    TXEnabled = false;
  }

  startmillis = millis();
  for (i = 0; i < symbol_count; i++)
  {
    endmillis = startmillis + ((i + 1) * (unsigned long) tone_delay) ;
    uint64_t tonefreq;
    tonefreq = freq + ((tx_buffer[i] * tone_spacing));
    if (TXEnabled) si5351aSetFrequency(tonefreq);
    //wait untill tone is transmitted for the correct amount of time
    while (millis() < endmillis) {
      //just wait
    }
  }

  // Switches off Si5351a output
  si5351aOutputOff(SI_CLK0_CONTROL);
  digitalWrite(LEDIndicator1, LOW);
  Serial.println(F("TX Off"));

}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, sizeof(tx_buffer));
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
}



//Maidenhead code from Ossi Väänänen https://ham.stackexchange.com/questions/221/how-can-one-convert-from-lat-long-to-grid-square
void calcLocator(double lat, double lon) {
  int o1, o2, o3;
  int a1, a2, a3;
  double remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  MHLocator[0] = (char)o1 + 'A';
  MHLocator[1] = (char)a1 + 'A';
  MHLocator[2] = (char)o2 + '0';
  MHLocator[3] = (char)a2 + '0';
  MHLocator[4] = (char)o3 + 'A';
  MHLocator[5] = (char)a3 + 'A';
}



// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPSSerial.available())
      gps.encode(GPSSerial.read());
  } while (millis() - start < ms);
}


uint8_t i2cStart()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) ;

  return (TWSR & 0xF8);
}

void i2cStop()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

  while ((TWCR & (1 << TWSTO))) ;
}

uint8_t i2cByteSend(uint8_t data)
{
  TWDR = data;

  TWCR = (1 << TWINT) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) ;

  return (TWSR & 0xF8);
}

uint8_t i2cByteRead()
{
  TWCR = (1 << TWINT) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) ;

  return (TWDR);
}

uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
  uint8_t stts;

  stts = i2cStart();
  if (stts != I2C_START) return 1;

  stts = i2cByteSend(I2C_WRITE);
  if (stts != I2C_SLA_W_ACK) return 2;

  stts = i2cByteSend(reg);
  if (stts != I2C_DATA_ACK) return 3;

  stts = i2cByteSend(data);
  if (stts != I2C_DATA_ACK) return 4;

  i2cStop();

  return 0;
}

uint8_t i2cReadRegister(uint8_t reg, uint8_t *data)
{
  uint8_t stts;

  stts = i2cStart();
  if (stts != I2C_START) return 1;

  stts = i2cByteSend(I2C_WRITE);
  if (stts != I2C_SLA_W_ACK) return 2;

  stts = i2cByteSend(reg);
  if (stts != I2C_DATA_ACK) return 3;

  stts = i2cStart();
  if (stts != I2C_START_RPT) return 4;

  stts = i2cByteSend(I2C_READ);
  if (stts != I2C_SLA_R_ACK) return 5;

  *data = i2cByteRead();

  i2cStop();

  return 0;
}

// Init TWI (I2C)
//
void i2cInit()
{
  TWBR = 92;
  TWSR = 0;
  TWDR = 0xFF;
  PRR = 0;
}

//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1; // PLL config register P1
  uint32_t P2; // PLL config register P2
  uint32_t P3; // PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 &
                  0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

//
// Set up MultiSynth with integer Divider and R Divider
// R Divider is the bit value which is OR'ed onto the appropriate
// register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t Divider, uint8_t rDiv)
{
  uint32_t P1; // Synth config register P1
  uint32_t P2; // Synth config register P2
  uint32_t P3; // Synth config register P3

  P1 = 128 * Divider - 512;
  P2 = 0; // P2 = 0, P3 = 1 forces an integer value for the Divider
  P3 = 1;

  i2cSendRegister(synth + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 1, (P3 & 0x000000FF));
  i2cSendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv);
  i2cSendRegister(synth + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 4, (P1 & 0x000000FF));
  i2cSendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 &
                  0x000F0000) >> 16));
  i2cSendRegister(synth + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 7, (P2 & 0x000000FF));
}

//
// Switches off Si5351a output
// Example: si5351aOutputOff(SI_CLK0_CONTROL);
// will switch off output CLK0
//
void si5351aOutputOff(uint8_t clk)
{
  digitalWrite(TransmitLED, LOW);
  i2cSendRegister(clk, 0x80); // Refer to SiLabs AN619 to see
  //bit values - 0x80 turns off the output stage
}




//
// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 10kHz to 150MHz and given in centiHertz (hundreds of Hertz)
// Example: si5351aSetFrequency(1000000200);
// will set output CLK0 to 10.000,002MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
//
void si5351aSetFrequency(uint64_t frequency) //Frequency is in centiHz
{
  static uint64_t oldFreq;
  int32_t FreqChange;
  uint64_t pllFreq;
  uint32_t xtalFreq = XTAL_FREQ;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t Divider;
  uint8_t rDiv;

  digitalWrite(TransmitLED, HIGH);
  // Serial.print (F("Freq is="));
  // Serial.println (uint64ToStr(frequency,false));
  if (frequency > 100000000) { //If higher than 1MHz then set output divider to 1
    rDiv = SI_R_DIV_1;
    Divider = 90000000000ULL / frequency;// Calculate the division ratio. 900MHz is the maximum internal (expressed as deciHz)
    //Serial.print (F("Divider="));
    //Serial.println (Divider);
    pllFreq = Divider * frequency; // Calculate the pllFrequency:
    //the Divider * desired output frequency
    //Serial.print (F("pllFreq="));
    //Serial.println (uint64ToStr(pllFreq,false));
    mult = pllFreq / (xtalFreq * 100UL); // Determine the multiplier to
    //Serial.print (F("mult="));
    //Serial.println (mult);

    //get to the required pllFrequency
    l = pllFreq % (xtalFreq * 100UL); // It has three parts:
    //Serial.print (F("l="));
    //Serial.println (l);
    f = l; // mult is an integer that must be in the range 15..90
    //Serial.print (F("f="));
    //Serial.println (f);
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    //Serial.print (F("f="));
    //Serial.println (f);
    f /= xtalFreq; // each is 20 bits (range 0..1048575)
    //Serial.print (F("f="));
    //Serial.println (f);

    num = f; // the actual multiplier is mult + num / denom
    //Serial.print (F("num="));
    //Serial.println (num);
    denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
    num = num / 100;
  }
  else // lower freq than 1MHz - use output Divider set to 128
  {
    rDiv = SI_R_DIV_128;
    frequency = frequency * 128ULL; //Set base freq 128 times higher as we are dividing with 128 in the last output stage
    Divider = 90000000000ULL / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal

    pllFreq = Divider * frequency; // Calculate the pllFrequency:
    //the Divider * desired output frequency
    mult = pllFreq / (xtalFreq * 100UL); // Determine the multiplier to
    //get to the required pllFrequency
    l = pllFreq % (xtalFreq * 100UL); // It has three parts:
    //Serial.print (F("l="));
    //Serial.println (l);
    f = l; // mult is an integer that must be in the range 15..90
    //Serial.print (F("f="));
    //Serial.println (f);
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    //Serial.print (F("f="));
    //Serial.println (f);
    f /= xtalFreq; // each is 20 bits (range 0..1048575)
    //Serial.print (F("f="));
    //Serial.println (f);

    num = f; // the actual multiplier is mult + num / denom
    //Serial.print (F("num="));
    //Serial.println (num);
    denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
    num = num / 100;
  }


  // Set up PLL A with the calculated  multiplication ratio
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);

  // Set up MultiSynth Divider 0, with the calculated Divider.
  // The final R division stage can divide by a power of two, from 1..128.
  // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
  // If you want to output frequencies below 1MHz, you have to use the
  // final R division stage
  setupMultisynth(SI_SYNTH_MS_0, Divider, rDiv);

  // Reset the PLL. This causes a glitch in the output. For small changes to
  // the parameters, you don't need to reset the PLL, and there is no glitch
  FreqChange = frequency - oldFreq;

  if ( abs(FreqChange) > 100000) //If changed more than 1kHz then reset PLL (completely arbitrary choosen)
  {
    i2cSendRegister(SI_PLL_RESET, 0xA0);
  }

  // Finally switch on the CLK0 output (0x4F)
  // and set the MultiSynth0 input to be PLL A
  i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
  oldFreq = frequency;
}

/*
  uint64_t  StrTouint64_t (String InString)
  {
  uint64_t y = 0;

  for (int i = 0; i < InString.length(); i++) {
    char c = InString.charAt(i);
    if (c < '0' || c > '9') break;
    y *= 10;
    y += (c - '0');
  }
  return y;
  }
*/

String  uint64ToStr (uint64_t p_InNumber, boolean p_LeadingZeros)
{
  char l_HighBuffer[7]; //6 digits + null terminator char
  char l_LowBuffer[7]; //6 digits + null terminator char
  char l_ResultBuffer [13]; //12 digits + null terminator char
  String l_ResultString = "";
  uint8_t l_Digit;

  sprintf(l_HighBuffer, "%06lu", p_InNumber / 1000000L); //Convert high part of 64bit unsigned integer to char array
  sprintf(l_LowBuffer, "%06lu", p_InNumber % 1000000L); //Convert low part of 64bit unsigned integer to char array
  l_ResultString = l_HighBuffer;
  l_ResultString = l_ResultString + l_LowBuffer; //Copy the 2 part result to a string

  if (!p_LeadingZeros) //If leading zeros should be romeved
  {
    l_ResultString.toCharArray(l_ResultBuffer, 13);
    for (l_Digit = 0; l_Digit < 12; l_Digit++ )
    {
      if (l_ResultBuffer[l_Digit] == '0')
      {
        l_ResultBuffer[l_Digit] = ' '; // replace zero with a space character
      }
      else
      {
        break; //We have found all the leading Zeros, exit loop
      }
    }
    l_ResultString = l_ResultBuffer;
    l_ResultString.trim();//Remove all leading spaces
  }
  return l_ResultString;
}


boolean NoBandEnabled(void)
{
  boolean NoOne = true;
  for (int FreqLoop = 0; FreqLoop < 13; FreqLoop++) {
    if (TXOnBand[FreqLoop]) NoOne = false;
  }
  return NoOne;
}

uint64_t NextFreq (void)
{
  if (NoBandEnabled())
  {
    freq = 0;
  }
  else
  {
    do
    {
      CurrentBand++;
      if (CurrentBand > 12) CurrentBand = 0;
    } while (TXOnBand[CurrentBand] == false);

    switch (CurrentBand) {
      case 0:
        freq = WSPR_FREQ2190m;
        break;
      case 1:
        freq = WSPR_FREQ630m;
        break;
      case 2:
        freq = WSPR_FREQ160m ;
        break;
      case 3:
        freq = WSPR_FREQ80m ;
        break;
      case 4:
        freq = WSPR_FREQ40m ;
        break;
      case 5:
        freq = WSPR_FREQ30m ;
        break;
      case 6:
        freq = WSPR_FREQ20m ;
        break;
      case 7:
        freq = WSPR_FREQ17m ;
        break;
      case 8:
        freq = WSPR_FREQ15m ;
        break;
      case 9:
        freq = WSPR_FREQ12m ;
        break;
      case 10:
        freq = WSPR_FREQ10m ;
        break;
      case 11:
        freq = WSPR_FREQ6m ;
        break;
      case 12:
        freq = WSPR_FREQ4m ;
    }
  }
}


boolean LastFreq (void)
{
  boolean Last = true;
  int TestBand;

  TestBand = CurrentBand;
  if (TestBand == 12)
  {
    Last = true;
  }
  else
  {
    do
    {
      TestBand++;
      if (TXOnBand[TestBand]) Last = false;
    } while (TestBand < 12);
  }
  return Last;
}

/*
  void UpdatePos (void)
  {
  double Lat;
  double Lon;
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
    if (gps.location.isUpdated())
    {

     //Lat = gps.location.lat();
      //Lon = gps.location.lng();
      calcLocator (gps.location.lat(), gps.location.lng());
      //Serial.print (F("GPS Fix accuired - Maidenhead locator is "));
      //Serial.print(MHLocator);
      loc[0] = MHLocator[0]; loc[1] = MHLocator[1]; loc[2] = MHLocator[2]; loc[3] = MHLocator[3];
      //Serial.print (F(" (using "));
      //Serial.print(loc);
      //Serial.println (F(")"));
      break;
    }
  }
  }
*/

//Create a random seed by doing CRC32 on 100 analog values from port A0
unsigned long RandomSeed(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  uint8_t ByteVal;
  unsigned long crc = ~0L;

  for (int index = 0 ; index<100 ; ++index) {
    ByteVal= analogRead(A0);
    crc = crc_table[(crc ^ ByteVal) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (ByteVal >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}
