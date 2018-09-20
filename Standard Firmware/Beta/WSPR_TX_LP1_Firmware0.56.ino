/*
  Software for Zachtek "WSPR-TX Version 1"
  The Version of this software is stored in the constant "softwareversion" and is displayed on the Serialport att startup
  For Arduino Pro Mini ATMega 328 8MHz

  Hardware connections:
  --------------------------
  pin 2 and 3 is Sofware serial port to GPS module
  pin 4 is Yellow Status LED. Used as StatusIndicator to display what state the software is currently in.
  pin 8 is Red TX LED next to RF out SMA connecotr. Used as StatusIndicator to display when transmission occurs.
  For Arduino Pro Mini 328 8MHz

  Version History:
  -------------------------------------
  0.51 First Beta for Serial API
  0.51 Expanded the Serial API and changed all information messages to {MIN} messages
  0.52 [DGF] API updates will change SignalGen output freq if running.
  0.54 Added TX status updates when sending  CCM status to improve the PC client display.
  0.55 Added support for factory data in EEPROM, stores TCXO and hardware info. 
  0.56 Changed 80 TX freq to new standard of 3.570,100MHz 
*/

#include <EEPROM.h>
#include <TinyGPS++.h>  //TinyGPS++ library by Mikal Hart https://github.com/mikalhart/TinyGPSPlus
#include <JTEncode.h>   //JTEncode  by NT7S https://github.com/etherkit/JTEncode
#include <SoftwareSerial.h>


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
  VHF4m =  12,
  VHF2m =  13,
  UHF70cm = 14,
  UHF23cm = 15
};

enum E_Mode
{
  WSPRBeacon ,
  SignalGen,
  Idle
};

enum E_LocatorOption {
  Manual,
  GPS
};

struct S_WSPRData
{
  char CallSign[7];                //Radio amateur Call Sign, zero terminated string can be four to six char in length + zero termination
  E_LocatorOption LocatorOption;   //If transmitted Maidenhead locator is based of GPS location or if it is using MaidneHead4 variable.
  char MaidenHead4[5];             //Maidenhead locator, must be 4 chars and a zero termination
  uint8_t TXPowerdBm;              //Power data in dBm min=0 max=60
};

struct S_GadgetData
{
  char Name[40];         //Optional Name of the device.
  E_Mode StartMode;      //What mode the Gadget should go to after boot.
  S_WSPRData WSPRData;   //Data needed to transmit a WSPR packet.
  bool TXOnBand [16];    //Arraycount corresponds to the Enum E_Band, True =Transmitt Enabled, False = Transmitt disabled on this band
  unsigned long TXPause;           //Number of seconds to pause after having transmitted on all enabled bands.
  uint64_t GeneratorFreq;//Frequency for when in signal Generator mode. Freq in centiHertz.
};

struct S_FactoryData
{
  uint32_t Product_Model; // Product number. E.g WSPR-TX_LP1=1011
  uint8_t  HW_Version;    // Hardware version
  uint8_t  HW_Revision;   // Hardware revision
  uint32_t RefFreq;       //The frequency of the Reference Oscillator in Hz, usually 26000000
};

//Constants
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

#define WSPR_FREQ23cm         129650150000ULL   //23cm 1296.501,500MHz (Overtone, not implemented)
#define WSPR_FREQ70cm          43230150000ULL   //70cm  432.301,500MHz (Overtone, not implemented)
#define WSPR_FREQ2m            14449000000ULL   //2m    144.490,000MHz //Not working. No decode in bench test with WSJT-X decoding Software
#define WSPR_FREQ4m             7009250000ULL   //4m     70.092,500MHz //Low Output power due to attenuation by last 60MHz Low pass filter
#define WSPR_FREQ6m             5029450000ULL   //6m     50.294,500MHz //Slightly lower output power
#define WSPR_FREQ10m            2812620000ULL   //10m    28.126,200MHz
#define WSPR_FREQ12m            2492620000ULL   //12m    24.926,200MHz 
#define WSPR_FREQ15m            2109620000ULL   //15m    21.096.200MHz  
#define WSPR_FREQ17m            1810610000ULL   //17m    18.106,100MHz
#define WSPR_FREQ20m            1409710000ULL   //20m    14.097,100MHz 
#define WSPR_FREQ30m            1014020000ULL   //30m    10.140,200MHz  
#define WSPR_FREQ40m             704010000ULL   //40m     7.040,100MHz 
#define WSPR_FREQ80m             357010000ULL   //80m     3.570,100MHz
#define WSPR_FREQ160m            183810000ULL   //160m    1.838,100MHz
#define WSPR_FREQ630m             47570000ULL   //630m      475.700kHz
#define WSPR_FREQ2190m            13750000ULL   //2190m     137.500kHz

#define FactorySpace true
#define UserSpace    false

#define UMesCurrentMode 1
#define UMesLocator 2
#define UMesTime 3
#define UMesGPSLock 4
#define UMesNoGPSLock 5
#define UMesFreq 6
#define UMesTXOn 7
#define UMesTXOff 8

// Hardware defines
#define StatusLED 4
#define Relay1 5
#define Relay2 6
#define Relay3 7
#define TransmitLED 8

const char softwareversion[] = "Beta 0.55" ; //Version of this program, sent to serialport at startup

//Global Variables
S_GadgetData GadgetData;   //Create a datastructure that holds all relevant data for a WSPR Beacon
S_FactoryData FactoryData; //Create a datastructure that holds information of the hardware
E_Mode CurrentMode;        //What mode are we in, WSPR, signal-gen or nothing

//bool TXOnBand [13];    //Arraycount corresponds to the Enum E_Band, True =Transmitt Enabled, False = Transmitt disabled on this band  E.g to transmitt on 40m and 20m set TXOnBand[4] and TXOnBand[6] to true.
int CurrentBand = 0;   //Keeps track on what band we are currently tranmitting on
const uint8_t SerCMDLength = 50; //Max number of char on a command in the SerialAPI
void i2cInit();
uint8_t i2cSendRegister(uint8_t reg, uint8_t data);
uint8_t i2cReadRegister(uint8_t reg, uint8_t *data);
//uint8_t WSPRBandHopCount = 0; //Keep tracks on what band we transmitted on
uint8_t tx_buffer[171];
uint64_t freq;   //Holds the Output frequency when we are in signal generator mode or in WSPR mode
int GPSH; //GPS Hours
int GPSM; //GPS Minutes
int GPSS; //GPS Seconds
// Class instantiation
JTEncode jtencode;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial GPSSerial(2, 3); //GPS Serial port, RX on pin 2, TX on pin 3
void si5351aOutputOff(uint8_t clk);
void si5351aSetFrequency(uint32_t frequency);


void setup()
{

  //Initialize the serial ports, The hardware port is used for communicating with a PC.
  //The Soft Serial is for communcating with the GPS
  Serial.begin (9600);  //USB Serial port
  Serial.setTimeout(2000);
  GPSSerial.begin(9600); //Internal GPS Serial port
  bool i2c_found;

  // Use the Red LED as a Transmitt indicator and the Yellow LED as Status indicator
  Serial.print(F("{MIN} ZachTek WSPR-TX_LP1 standard firmware version "));
  Serial.println(softwareversion);
  pinMode(StatusLED, OUTPUT);
  pinMode(Relay1, INPUT);//Set Relay1 as Input to deactivate the relay
  pinMode(Relay2, INPUT);//Set Relay1 as Input to deactivate the relay
  pinMode(Relay3, INPUT);//Set Relay1 as Input to deactivate the relay
  for (int i = 0; i <= 15; i++) {
    digitalWrite(StatusLED, HIGH);
    delay (50);
    digitalWrite(StatusLED, LOW);
    delay (50);
  }

  //Read all the Factory data from EEPROM at position 400
  if (LoadFromEPROM(FactorySpace)) //Read all Factory data from EEPROM
  {
    
  }
  else  //No data was found in EEPROM, set some defaults
  {
    FactoryData.Product_Model=1011; // Product number. E.g WSPR-TX_LP1=1011
    FactoryData.HW_Version=1;    // Hardware version
    FactoryData.HW_Revision=5;   // Hardware revision
    FactoryData.RefFreq = 26000000;//Reference Oscillator frequency
    Serial.println(F("{MIN} No factory data found for Model # and Reference frequency, guessing on values"));  
  }

  if (LoadFromEPROM(UserSpace)) //Read all UserSpace data from EEPROM at position 0
  {
    CurrentMode = GadgetData.StartMode;
    GadgetData.WSPRData.CallSign[6] = 0;//make sure Call sign is null terminated in case of incomplete data saved
    GadgetData.WSPRData.MaidenHead4[4] = 0; //make sure Maidenhead locator is null terminated in case of incomplete data saved
  }
  else //No data was found in EEPROM, set some defaults
  {
    CurrentMode = Idle;
    GadgetData.Name[0] = 'W'; GadgetData.Name[1] = 'S'; GadgetData.Name[2] = 'P'; GadgetData.Name[3] = 'R';
    GadgetData.Name[4] = ' '; GadgetData.Name[5] = 'T'; GadgetData.Name[6] = 'X'; GadgetData.Name[7] = 0;
    GadgetData.StartMode = Idle;
    GadgetData.WSPRData.CallSign[0] = 'A'; GadgetData.WSPRData.CallSign[1] = 'A'; GadgetData.WSPRData.CallSign[2] = '0';
    GadgetData.WSPRData.CallSign[3] = 'A'; GadgetData.WSPRData.CallSign[4] = 'A'; GadgetData.WSPRData.CallSign[5] = 'A';
    GadgetData.WSPRData.CallSign[6] = 0;
    GadgetData.WSPRData.LocatorOption = GPS;
    GadgetData.WSPRData.MaidenHead4[0] = 'A'; GadgetData.WSPRData.MaidenHead4[1] = 'A';
    GadgetData.WSPRData.MaidenHead4[2] = '0'; GadgetData.WSPRData.MaidenHead4[3] = '0';
    GadgetData.WSPRData.MaidenHead4[4] = 0;
    GadgetData.WSPRData.TXPowerdBm = 23;
    for (int i = 0; i <= 16; i++)
    {
      GadgetData.TXOnBand [i] = false;
    }
    GadgetData.TXPause = 120;  //Number of minutes to pause after transmisson
    GadgetData.GeneratorFreq = 1000000000;
    Serial.println(F("{MIN} No user data was found, setting default values"));
  }

  i2cInit();
  si5351aSetFrequency(2000000000ULL);
  si5351aOutputOff(SI_CLK0_CONTROL);

  random(RandomSeed());
  //if (CurrentMode == SignalGen)  DoSignalGen();
  //if (CurrentMode == WSPRBeacon) DoWSPR();
  //if (CurrentMode == Idle) DoIdle();

    switch (CurrentMode) {
      case SignalGen :
        DoSignalGen();
        break;
    
      case WSPRBeacon:
        DoWSPR();
        break;   

      case Idle:
        DoIdle();
        break;
   }  
}

void loop()
{
  DoSerialHandling();
  delay (100);
}


//Serial API commands and data decoding
void DecodeSerialCMD(const char * InputCMD) {
  char CharInt[13];
  bool EnabDisab;
  if ((InputCMD[0] == '[') && (InputCMD[4] == ']')) { //A Command,Option or Data input

    if (InputCMD[1] == 'C') {  //Commmand

      //Current Mode
      if ((InputCMD[2] == 'C') && (InputCMD[3] == 'M')) {
        if (InputCMD[6] == 'S') { //Set option
          if (InputCMD[8] == 'S') {
            DoSignalGen();
          }
          if (InputCMD[8] == 'W') {
            DoWSPR();
          }
          if (InputCMD[8] == 'N') {
            DoIdle ();
          }
        }//Set Current Mode
        else //Get
        {
          SendAPIUpdate (UMesCurrentMode);
        }//Get Current Mode
      }//CurrentMode


      //Store Current configuration data to EEPROM
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E')) {
        if (InputCMD[6] == 'S') { //Set option
          SaveToEEPROM(UserSpace);
           Serial.println(F("{MIN} User data saved"));  
        }
      }

      exit;
    }

    if (InputCMD[1] == 'O') {//Option

      //TX Pause
      if ((InputCMD[2] == 'T') && (InputCMD[3] == 'P')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = InputCMD[10];
          CharInt[3] = InputCMD[11]; CharInt[4] = InputCMD[12]; CharInt[5] = 0;
          GadgetData.TXPause = atoi(CharInt);
        }
        else //Get Option
        {
          Serial.print (F("{OTP} "));
          if (GadgetData.TXPause < 10000) Serial.print ("0");
          if (GadgetData.TXPause < 1000) Serial.print ("0");
          if (GadgetData.TXPause < 100) Serial.print ("0");
          if (GadgetData.TXPause < 10) Serial.print ("0");
          Serial.println (GadgetData.TXPause);
        }
      }//TX Pause


      //StartMode
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'M')) {
        if (InputCMD[6] == 'S') { //Set option
          if (InputCMD[8] == 'S') {
            GadgetData.StartMode = SignalGen;
          }
          if (InputCMD[8] == 'W') {
            GadgetData.StartMode = WSPRBeacon;
          }
          if (InputCMD[8] == 'N') {
            GadgetData.StartMode = Idle;
          }
        }//Set Start Mode
        else //Get
        {
          Serial.print ("{OSM} ");
          switch (GadgetData.StartMode) {
            case Idle:
              Serial.println ("N");
              break;
            case WSPRBeacon:
              Serial.println ("W");
              break;
            case SignalGen:
              Serial.println ("S");
              break;
          }
        }//Get Start Mode
      }//StartMode

      //Band TX enable
      if ((InputCMD[2] == 'B') && (InputCMD[3] == 'D')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = 0; CharInt[3] = 0;
          EnabDisab = false;
          if (InputCMD[11] == 'E') EnabDisab = true;
          GadgetData.TXOnBand [atoi(CharInt)] = EnabDisab ; //Enable or disable on this band
        }//Set Band TX enable
        else //Get
        {
          //Send out 16 lines, one for each band
          for (int i = 0; i <= 15; i++) {
            Serial.print (F("{OBD} "));
            if (i < 10) Serial.print (F("0"));
            Serial.print (i);
            if (GadgetData.TXOnBand[i]) {
              Serial.println (F(" E"));
            }
            else
            {
              Serial.println (F(" D"));
            }
          }//for
        }//Get Band TX enable
      }//Band TX enable

      //Location Option
      if ((InputCMD[2] == 'L') && (InputCMD[3] == 'C')) {
        if (InputCMD[6] == 'S') { //Set Location Option
          if (InputCMD[8] == 'G') {
            GadgetData.WSPRData.LocatorOption = GPS;
          }
          if (InputCMD[8] == 'M') {
            GadgetData.WSPRData.LocatorOption = Manual;
          }
        }//Set Location Option
        else //Get Location Option
        {
          Serial.print ("{OLC} ");
          if (GadgetData.WSPRData.LocatorOption == GPS)
          {
            Serial.println ("G");
          }
          else
          {
            Serial.println ("M");
          }
        }//Get Location Option
      }//Location Option
      exit;
    }//All Options

    //Data
    if (InputCMD[1] == 'D') {
      //Callsign
      if ((InputCMD[2] == 'C') && (InputCMD[3] == 'S')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 5; i++) {
            GadgetData.WSPRData.CallSign[i] = InputCMD[i + 8];
          }
          GadgetData.WSPRData.CallSign[6] = 0;
        }
        else //Get
        {
          Serial.print (F("{DCS} "));
          Serial.println (GadgetData.WSPRData.CallSign);
        }
      }//Callsign

      //Locator
      if ((InputCMD[2] == 'L') && (InputCMD[3] == '4')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 3; i++) {
            GadgetData.WSPRData.MaidenHead4[i] = InputCMD[i + 8];
          }
          GadgetData.WSPRData.MaidenHead4[4] = 0;
        }
        else //Get
        {
          Serial.print (F("{DL4} "));
          Serial.println (GadgetData.WSPRData.MaidenHead4);
        }
      }//Locator

      //Name
      if ((InputCMD[2] == 'N') && (InputCMD[3] == 'M')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 38; i++) {
            GadgetData.Name[i] = InputCMD[i + 8];
          }
          GadgetData.Name[39] = 0;
        }
        else //Get
        {
          Serial.print (F("{DNM} "));
          Serial.println (GadgetData.Name);
        }
      }//Name

      //Power data
      if ((InputCMD[2] == 'P') && (InputCMD[3] == 'D')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = 0; CharInt[3] = 0;
          GadgetData.WSPRData.TXPowerdBm = atoi(CharInt);
        }
        else //Get
        {
          Serial.print (F("{DPD} "));
          if (GadgetData.WSPRData.TXPowerdBm < 10) Serial.print ("0");
          Serial.println (GadgetData.WSPRData.TXPowerdBm);
        }
      }//Power Data


      //Generator Frequency
      if ((InputCMD[2] == 'G') && (InputCMD[3] == 'F')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 11; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[12] = 0;
          GadgetData.GeneratorFreq = StrTouint64_t(CharInt);
          if (CurrentMode == SignalGen) DoSignalGen();
        }
        else //Get
        {
          Serial.print (F("{DGF} "));
          Serial.println (uint64ToStr(GadgetData.GeneratorFreq, true));
        }
      }//Generator Frequency
      
      exit;
    }//Data

    //Factory
    if (InputCMD[1] == 'F') {
      
       //Product model Number
      if ((InputCMD[2] == 'P') && (InputCMD[3] == 'N')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = InputCMD[10];
          CharInt[3] = InputCMD[11]; CharInt[4] = InputCMD[12]; CharInt[5] = 0;
          FactoryData.Product_Model = atoi(CharInt);
        }//Set
        else //Get Option
        { 
          Serial.print (F("{FPN} "));
          if (FactoryData.Product_Model < 10000) Serial.print ("0");
          Serial.println (FactoryData.Product_Model);
        }
      }//Product model Number

      //Hardware Version
      if ((InputCMD[2] == 'H') && (InputCMD[3] == 'V')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = InputCMD[10];
          CharInt[3]  = 0;
          FactoryData.HW_Version = atoi(CharInt);
        }//Set
        else //Get Option
        { 
          Serial.print (F("{FHV} "));
          if (FactoryData.HW_Version < 100) Serial.print ("0");
          if (FactoryData.HW_Version < 10) Serial.print ("0");
          Serial.println (FactoryData.HW_Version);
        }
      }//Hardware Version

      //Hardware Revision
      if ((InputCMD[2] == 'H') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = InputCMD[10];
          CharInt[3]  = 0;
          FactoryData.HW_Revision = atoi(CharInt);
        }//Set
        else //Get Option
        { 
          Serial.print (F("{FHR} "));
          if (FactoryData.HW_Revision < 100) Serial.print ("0");
          if (FactoryData.HW_Revision < 10) Serial.print ("0");
          Serial.println (FactoryData.HW_Revision);
        }
      }//Hardware Revision

      //Reference Oscillator Frequency
      if ((InputCMD[2] == 'R') && (InputCMD[3] == 'F')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 8; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[9] = 0;
          FactoryData.RefFreq = StrTouint64_t(CharInt);
        }
        else //Get
        {
          Serial.print (F("{FRF} "));
          Serial.println (uint64ToStr(FactoryData.RefFreq, true));
        }
      }//Reference Oscillator Frequency

      //Store Current Factory configuration data to EEPROM
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E')) {
        if (InputCMD[6] == 'S') { //Set option
          SaveToEEPROM(FactorySpace);
          Serial.println(F("{MIN} Factory data saved"));
        }
      }
      
       exit;
    }//Factory
  }
}


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

  if (!p_LeadingZeros) //If leading zeros should be removed
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

//Parts from NickGammon Serial Input example
//http://www.gammon.com.au/serial
void DoSerialHandling()
{
  static char SerialLine[SerCMDLength]; //A single line of incoming serial command and data
  static uint8_t input_pos = 0;
  char InChar;

  while (Serial.available () > 0)
  {
    //Serial.println ("Serial availabe");
    InChar = Serial.read ();
    switch (InChar)
    {
      case '\n':   // end of text
        SerialLine [input_pos] = 0;  // terminating null byte

        // terminator reached, process Command
        DecodeSerialCMD (SerialLine);
        //Serial.println("New Line");
        // reset buffer for next time
        input_pos = 0;
        break;

      case '\r':   // discard carriage return
        break;

      default:
        // keep adding if not full ... allow for terminating null byte
        if (input_pos < (SerCMDLength - 1))
          SerialLine [input_pos++] = InChar;
        break;

    }  // end of switch

  } // end of processIncomingByte
}


void DoSignalGen ()
{
  CurrentMode=SignalGen;
  freq = GadgetData.GeneratorFreq;
  si5351aSetFrequency(freq);
  digitalWrite(StatusLED, HIGH);
  SendAPIUpdate (UMesCurrentMode);
  SendAPIUpdate (UMesFreq);
}


void DoIdle ()
{
  CurrentMode = Idle;
  digitalWrite(StatusLED, LOW);
  si5351aOutputOff(SI_CLK0_CONTROL);
  SendAPIUpdate (UMesCurrentMode);
}


void DoWSPR ()
{
  // static double Lat;
  // static double Lon;
  CurrentMode = WSPRBeacon;
  if (NoBandEnabled ()) {
    Serial.println (F("{MIN} Warning, configuration error, tranmission is not enabled on any band!"));
  }
  else
  {
    NextFreq();
    freq = freq + (100ULL * random (-100, 100)); //modify TX frequency with a random value beween -100 and +100 Hz
    si5351aOutputOff(SI_CLK0_CONTROL);
    SendAPIUpdate (UMesCurrentMode);

    //LOOP HERE FOREVER OR UNTIL INTERRUPTED BY A SERIAL COMMAND
    while (!Serial.available()) { //Do until incoming serial command
      while (GPSSerial.available()) {
        if (Serial.available()) {// If serialdata was received on control port then abort and handle command
          DoIdle(); //Return to ideling
          return;
        }
        gps.encode(GPSSerial.read());
        if (gps.location.isUpdated())
        {
          GPSH = gps.time.hour();
          GPSM = gps.time.minute();
          GPSS = gps.time.second();

          //Lat = gps.location.lat();
          //Lon = gps.location.lng();

          if (GadgetData.WSPRData.LocatorOption == GPS) { //If GPS should update the Maidenhead locator
            //calcLocator (Lat, Lon);
            calcLocator (gps.location.lat(), gps.location.lng());
            //Serial.print (F("GPS calculated Maidenhead locator is "));
          }
          else
          {
            // Serial.print (F("Maidenhead locator manually set to "));
          }
          // if (SetWSPRFreq())
          //Serial.println(GadgetData.WSPRData.MaidenHead4);
          //Serial.println (F("Waiting for start of even minute to begin a new WSPR transmission block"));
          if ((GPSS == 0) && ((GPSM % 2) == 0))//If second is zero at even minute then start WSPR transmission
          {
            set_tx_buffer();// Encode the message in the transmit buffer
           
            SendWSPRBlock ();
            if (LastFreq ())
            {
      
              smartdelay(GadgetData.TXPause * 1000UL); //Pause for some time to give a duty cycle on the transmit. 2000=100%, 20000=50%, 130000=33%
            }
            NextFreq();
            freq = freq + (100ULL * random (-100, 100)); //modify TX frequency with a random value beween -100 and +100 Hz

            smartdelay(3000);
            // Serial.println (F("Waiting for start of even minute to start a new WSPR transmission block"));

          }
          else //Dubble-blink to indicate waiting for top of minute
          {
            SendAPIUpdate(UMesGPSLock);
            SendAPIUpdate(UMesTime);
            SendAPIUpdate(UMesLocator);
            LEDBlink(); 
            LEDBlink(); 
            smartdelay(200);
          }
        }
        if (gps.location.isValid())
        {
         //
       
        }
        else
        {
          //singelblink to indicate waiting for GPS Lock
          LEDBlink(); 
          SendAPIUpdate(UMesNoGPSLock);
          smartdelay(400);
        }
      }
    }
  }//As this routine will run for ever if not interuppted by serial port, indicate end of routine by letting DoIdle routine send status message
  DoIdle(); //Return to ideling
}


// Loop through the string, transmitting one character at a time.
void SendWSPRBlock()
{
  uint8_t i;
  unsigned long startmillis;
  unsigned long endmillis;
  boolean TXEnabled = true;

  // Send WSPR for two minutes
  digitalWrite(StatusLED, HIGH);
  if ((GadgetData.WSPRData.CallSign[0] == 'A') && (GadgetData.WSPRData.CallSign[1] == 'A') && (GadgetData.WSPRData.CallSign[2] == '0') && (GadgetData.WSPRData.CallSign[3] == 'A') && (GadgetData.WSPRData.CallSign[4] == 'A') && (GadgetData.WSPRData.CallSign[5] == 'A')) //Do not actually key the transmitter if the callsign has not been changed from the default one AA0AAA
  {
    Serial.println(F("{MIN} Callsign is not changed from the default one, Transmit is disabled"));
    Serial.println(F("{MIN} Set your Callsign to enable transmission"));
    TXEnabled = false;
  }

  startmillis = millis();
  for (i = 0; i < 162; i++)  //162 WSPR symbols to transmitt
  {
    endmillis = startmillis + ((i + 1) * (unsigned long) 683) ;   // Delay value for WSPR  delay is 683 milliseconds
    uint64_t tonefreq;
    tonefreq = freq + ((tx_buffer[i] * 146));  //~1.46 Hz  Tone spacing in centiHz
    if (TXEnabled) si5351aSetFrequency(tonefreq);
    //wait untill tone is transmitted for the correct amount of time
    while ((millis() < endmillis) && (!Serial.available())) ;//Until time is up or there is serial data received on the controll Serial port
    {
      //Do nothing, just wait
    }
    if (Serial.available()) {// If serialdata was received on Controll port then abort and handle command
      break;
    }
  }

  // Switches off Si5351a output
  si5351aOutputOff(SI_CLK0_CONTROL);
  digitalWrite(StatusLED, LOW);
  Serial.println(F("TX Off"));
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, sizeof(tx_buffer));
  jtencode.wspr_encode(GadgetData.WSPRData.CallSign, GadgetData.WSPRData.MaidenHead4, GadgetData.WSPRData.TXPowerdBm, tx_buffer);
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
  //remainder = remainder - 2.0 * (double)o2;
  //o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  //remainder = remainder - (double)a2;
  //a3 = (int)(24.0 * remainder);
  GadgetData.WSPRData.MaidenHead4[0] = (char)o1 + 'A';
  GadgetData.WSPRData.MaidenHead4[1] = (char)a1 + 'A';
  GadgetData.WSPRData.MaidenHead4[2] = (char)o2 + '0';
  GadgetData.WSPRData.MaidenHead4[3] = (char)a2 + '0';
  GadgetData.WSPRData.MaidenHead4[4] = 0;
}




// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartdelay(unsigned long ms)
{
long TimeLeft;
unsigned long EndTime = ms+millis();

  do
  {
    while (GPSSerial.available()) gps.encode(GPSSerial.read()); //If GPS data available - process it
    TimeLeft=EndTime - millis();
    if ((TimeLeft>1000) && ((TimeLeft % 1000) < 20)) {
    //Send API update every  second
      Serial.print (F("{MPS} "));
      Serial.println (TimeLeft/1000);
    }
  } while ((TimeLeft> 0) && (!Serial.available())) ;//Until time is up or there is serial data received 
    //Serial.println (F("{MPS} 0"));
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
  i2cSendRegister(clk, 0x80); // Refer to SiLabs AN619 to see
  //bit values - 0x80 turns off the output stage
  digitalWrite(TransmitLED, LOW);
  SendAPIUpdate(UMesTXOff);
  
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
  //uint32_t xtalFreq = XTAL_FREQ;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t Divider;
  uint8_t rDiv;


  if (frequency > 100000000) { //If higher than 1MHz then set output divider to 1
    rDiv = SI_R_DIV_1;
    Divider = 90000000000ULL / frequency;// Calculate the division ratio. 900MHz is the maximum internal (expressed as deciHz)
    pllFreq = Divider * frequency; // Calculate the pllFrequency:
    mult = pllFreq / (FactoryData.RefFreq * 100UL); // Determine the multiplier to
    l = pllFreq % (FactoryData.RefFreq * 100UL); // It has three parts:
    f = l; // mult is an integer that must be in the range 15..90
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    f /= FactoryData.RefFreq; // each is 20 bits (range 0..1048575)
    num = f; // the actual multiplier is mult + num / denom
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
    mult = pllFreq / (FactoryData.RefFreq * 100UL); // Determine the multiplier to
    //get to the required pllFrequency
    l = pllFreq % (FactoryData.RefFreq * 100UL); // It has three parts:
    f = l; // mult is an integer that must be in the range 15..90
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    f /= FactoryData.RefFreq; // each is 20 bits (range 0..1048575)
    num = f; // the actual multiplier is mult + num / denom
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
  digitalWrite(TransmitLED, HIGH);
  Serial.print (F("{TFQ} "));
  Serial.println (uint64ToStr(frequency,false));
  SendAPIUpdate(UMesTXOn);
}





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

  for (int index = 0 ; index < 100 ; ++index) {
    ByteVal = analogRead(A0);
    crc = crc_table[(crc ^ ByteVal) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (ByteVal >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}


boolean NoBandEnabled(void)
{
  boolean NoOne = true;
  for (int FreqLoop = 0; FreqLoop < 13; FreqLoop++) {
    if (GadgetData.TXOnBand [FreqLoop]) NoOne = false;
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
    } while (GadgetData.TXOnBand [CurrentBand] == false);

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
      if (GadgetData.TXOnBand [TestBand]) Last = false;
    } while (TestBand < 12);
  }
  return Last;
}


//Calculate CRC on either Factory data or Userspace data
unsigned long GetEEPROM_CRC(boolean EEPROMSpace) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;
  int Start;
  int Length;

  if (EEPROMSpace == FactorySpace)
  {
    Start = 400;
    Length = sizeof(FactoryData);
  }
  else
  {
    Start = 0;
    Length = sizeof(GadgetData);
  }
  for (int index = Start; index < (Start + Length) ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}


//Load FactoryData or UserSpace Data from Arduino EEPROM
bool LoadFromEPROM (boolean EEPROMSpace)
{

  int Start;
  int Length;
  unsigned long CRCFromEEPROM, CalculatedCRC;

  if (EEPROMSpace == FactorySpace) //Factory data
  {
    Start = 400;
    Length = sizeof(FactoryData);
    EEPROM.get(Start, FactoryData);                    //Load all the data from EEPROM
    CalculatedCRC = GetEEPROM_CRC(FactorySpace);       //Calculate the CRC of the data
  }
  else   //User data
  {
    Start = 0;
    Length = sizeof(GadgetData);
    EEPROM.get(Start, GadgetData);                     //Load all the data from EEPROM
    CalculatedCRC = GetEEPROM_CRC(UserSpace);          //Calculate the CRC of the data
  }
  EEPROM.get(Start + Length, CRCFromEEPROM);           //Load the saved CRC at the end of the data
  return (CRCFromEEPROM == CalculatedCRC);             //If  Stored and Calculated CRC are the same return true

}


//Save FactoryData or UserSpace Data to Arduino EEPROM
void SaveToEEPROM (boolean EEPROMSpace)
{
  int Start;
  int Length;
  unsigned long CRCFromEEPROM;
  if (EEPROMSpace == FactorySpace)
  {
    Start = 400;
    Length = sizeof(FactoryData);
    EEPROM.put(Start, FactoryData);         //Save all the Factory data to EEPROM at adress400
  }
  else //UserSpace
  {
    Start = 0;
    Length = sizeof(GadgetData);
    EEPROM.put(Start, GadgetData);          //Save all the User data to EEPROM at adress0
  }
  CRCFromEEPROM = GetEEPROM_CRC (EEPROMSpace);  //Calculate CRC on the saved data
  EEPROM.put(Start + Length, CRCFromEEPROM);    //Save the CRC after the data
}


void SendAPIUpdate (uint8_t UpdateType)
{


  switch (UpdateType) {
    case UMesCurrentMode :

      Serial.print (F("{CCM} "));
      switch (CurrentMode) {
        case Idle:
          Serial.println (F("N"));
          Serial.println (F("{TON} F")); //Send TX Off info
          break;
        case WSPRBeacon:
          Serial.println (F("W"));
          Serial.println (F("{TON} F"));  //Send TX Off info, only true if WSPR is in pause mode between transmission, if not it will be changed quickly by WSPR routine
          break;
        case SignalGen:
          Serial.println (F("S"));
          Serial.println (F("{TON} T"));  //Send TX ON info
          break;
      }
      break;

    case UMesLocator:
      Serial.print (F("{GL4} "));
      Serial.println (GadgetData.WSPRData.MaidenHead4);;
      break;

    case UMesTime:
      Serial.print (F("{GTM} "));
      if (GPSH < 10) Serial.print (F("0"));
      Serial.print (GPSH);
      Serial.print (F(":"));
      if (GPSM < 10) Serial.print (F("0"));
      Serial.print (GPSM);
      Serial.print (F(":"));
      if (GPSS < 10) Serial.print (F("0"));
      Serial.println (GPSS);
      break;

    case UMesGPSLock:
      Serial.println (F("{GLC} T"));
      break;  

    case UMesNoGPSLock:
      Serial.println (F("{GLC} F"));
      break;   
      
    case UMesFreq:
      Serial.print (F("{TFQ} "));
      Serial.println (uint64ToStr(freq,false));
      break;   

    case UMesTXOn:
      Serial.println (F("{TON} T"));
      break;  
       
    case UMesTXOff:
      Serial.println (F("{TON} F"));
      break; 
         
  }
}

//Brief flash on the Status LED
void LEDBlink()
{
  digitalWrite(StatusLED, HIGH);
  smartdelay (100);
  digitalWrite(StatusLED, LOW);
  smartdelay (100);
}

