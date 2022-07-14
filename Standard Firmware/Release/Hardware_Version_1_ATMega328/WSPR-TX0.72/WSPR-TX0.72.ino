/*
  Software for Zachtek "WSPR Transmitter" products
  For Arduino Pro Mini ATMega 328 8MHz

  Hardware connections:
  --------------------------
  pin 2 and 3 is Sofware serial port to GPS module
  pin 4 is Yellow Status LED. Used as StatusIndicator to display what state the software is currently in.
  pin 5,6 and 7 are Relay controll pins.
  pin 8 is Red TX LED next to RF out SMA connector. Used as StatusIndicator to display when transmission occurs.


  Version History:
  -------------------------------------
  0.51 First Beta for Serial API
  0.51 Expanded the Serial API and changed all information messages to {MIN} messages
  0.52 [DGF] API updates will change SignalGen output freq if running.
  0.54 Added TX status updates when sending  CCM status to improve the PC client display.
  0.55 Added support for factory data in EEPROM, stores TCXO and hardware info.
  0.56 Changed 80 TX freq to new standard of 3.570,100MHz
  0.57 Split Firmware in to two separate version for the WSPR-TX_ LP1 and Desktop products and changed the "Product_Model" from Factory EEPROM data to constant
  0.58 Fixed Frequency information output calculation errors when Freq=<1MHz
  0.59 Fixed dim Red TX LED, (Pin was not set to output by setup routine)
  0.60 Fixed wrong TX Freq on 10,12 and 15 Band
  0.61 Added routines to Set and Get LP filters with factory data API [FLP]
  0.62 Added functionality to automatically use one of the Low Pass filter in WSPR and SignalGen routines (New rutines - PickLP,SetLPFilter)
  0.63 Changed Software Version and Revision to constants that can be read by the Serial API [FSV] and [FSR] and merged Firmware for LP1 and Desktop in one version again as they were before v0.57
  0.64 Added function BandNumOfHigestLP to find the bandnumber of higest fitted LP filter, expanded on the PickLP filter routine
  0.65 Fixed bug that forced Hardware Revision to 4
  0.66 Fixed relay driving bug that affected Desktop transmitter with hardware revision higher than 4
  0.67 Added support for relay driving the WSPR-TX_LP1 with the Mezzanine LP4 card that contains relays
  0.68 Added support for manual override relay control over the Serial API and Relay update messages. ([CSL] command and {LPI} status message, shortened Start LED Blink
  0.69 Added support for hardware WSPR-TX Mini 1021, added funtion readVcc() that returs Arduino Voltage, added PowerSave funtions save current on the Mini
  0.70 Added power saving for the Mini, GPS turned off if TX pause is longer than a minute.
  0.71 Corrected 2m Frequency, not in use but nice to have correct. Enabled PLL power saving for the Mini if TX pause is longer than a minute. Current draw for mini is: 40mA waiting to TX,  60mA TX, 
  0.72 Sends GPS updates when in Idle and Signal gen mode and when pausing in WSRP Beacon mode. Improved Serial port respons when pausing in WSPR Beacon mode, 
       On the Mini the GPS and the Si5351 is put to power save during long TX Pauses. Current draw is now arround 40 mA regardless if tranmitting or not.
*/

const uint8_t  SoftwareVersion = 0; //0 to 255. 0=Beta
const uint8_t  SoftwareRevision = 72; //0 to 255

// Product model. WSPR-TX_LP1                             =1011
// Product model. WSPR-TX Desktop                         =1012
// Product model. WSPR-TX Mini                            =1017
// Product model. WSPR-TX_LP1 with Mezzanine LP4 card     =1020
const uint16_t Product_Model                              =1017;



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
  unsigned long TXPause; //Number of seconds to pause after having transmitted on all enabled bands.
  uint64_t GeneratorFreq;//Frequency for when in signal Generator mode. Freq in centiHertz.
};



struct S_FactoryData
{
  uint8_t  HW_Version;    // Hardware version
  uint8_t  HW_Revision;   // Hardware revision
  uint32_t RefFreq;       //The frequency of the Reference Oscillator in Hz, usually 26000000
  uint8_t  LP_A_BandNum;  //Low Pass filter A Band number (0-15) Ham Band as defined by E_Band  Eg. if a 20m LowPass filter is fitted on LP_A then LP_A_BandNum will be set to 6 by factory config software
  uint8_t  LP_B_BandNum;  //Low Pass filter B Band number (0-15)
  uint8_t  LP_C_BandNum;  //Low Pass filter C Band number (0-15)
  uint8_t  LP_D_BandNum;  //Low Pass filter D Band number (0-15)
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
#define WSPR_FREQ2m            14449500000ULL   //2m    144.490,000MHz //Not working. No decode in bench test with WSJT-X decoding Software
#define WSPR_FREQ4m             7009250000ULL   //4m     70.092,500MHz //Slightly lower output power
#define WSPR_FREQ6m             5029450000ULL   //6m     50.294,500MHz //Slightly lower output power
#define WSPR_FREQ10m            2812610000ULL   //10m    28.126,100MHz
#define WSPR_FREQ12m            2492610000ULL   //12m    24.926,100MHz 
#define WSPR_FREQ15m            2109610000ULL   //15m    21.096.100MHz  
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
#define UMesLPF 9

const uint8_t  LP_A = 0;
const uint8_t  LP_B = 1;
const uint8_t  LP_C = 2;
const uint8_t  LP_D = 3;

// Hardware defines

int     StatusLED;    //Yellow LED next to Green Power LED
#define Relay1 5
#define Relay2 6
#define Relay3 7
#define TransmitLED 8  //Red LED next to RF out SMA that will turn on when Transmitting
#define SiPower A3     //Power the Si5351 from this pin on the WSPR-TX Mini


//Global Variables
S_GadgetData GadgetData;   //Create a datastructure that holds all relevant data for a WSPR Beacon
S_FactoryData FactoryData; //Create a datastructure that holds information of the hardware
E_Mode CurrentMode;        //What mode are we in, WSPR, signal-gen or nothing

uint8_t CurrentBand = 0;   //Keeps track on what band we are currently tranmitting on
uint8_t CurrentLP =  0; //Keep track on what Low Pass filter is currently switched in
const uint8_t SerCMDLength = 50; //Max number of char on a command in the SerialAPI
void i2cInit();
uint8_t i2cSendRegister(uint8_t reg, uint8_t data);
uint8_t i2cReadRegister(uint8_t reg, uint8_t *data);
uint8_t tx_buffer[171];
uint64_t freq;   //Holds the Output frequency when we are in signal generator mode or in WSPR mode
int GPSH; //GPS Hours
int GPSM; //GPS Minutes
int GPSS; //GPS Seconds
bool GPSSleep;
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
  GPSSerial.begin(9600); //Init software serial port to communicate with the on-board GPS module
  bool i2c_found;


  switch (Product_Model) {
    case 1011:
      Serial.println(F("{MIN} ZachTek WSPR-TX_LP1 transmitter"));
      break;

    case 1012:
      Serial.println(F("{MIN} ZachTek WSPR Desktop transmitter"));
      break;

    case 1017:
      Serial.println(F("{MIN} ZachTek WSPR Mini transmitter"));
      break;

    case 1020:
      Serial.println(F("{MIN} ZachTek WSPR-TX_LP1 transmitter with Mezzanine LP4 board"));
      break;
  }

  Serial.print(F("{MIN} Firmware version "));
  if (SoftwareVersion == 0) {
    Serial.print(F("Beta "));
  }
  Serial.print(SoftwareVersion);
  Serial.print(F("."));
  Serial.println(SoftwareRevision);




  //Read all the Factory data from EEPROM at position 400
  if (LoadFromEPROM(FactorySpace)) //Read all Factory data from EEPROM
  {

  }
  else  //No factory data was found in EEPROM, set some defaults
  {
    FactoryData.HW_Version = 1;  // Hardware version
    FactoryData.HW_Revision = 6; // Hardware revision
    FactoryData.RefFreq = 27000000;//Reference Oscillator frequency
    FactoryData.LP_A_BandNum = 99; //Low Pass filter A is Nothing
    FactoryData.LP_B_BandNum = 99; //Low Pass filter B is Nothing
    FactoryData.LP_C_BandNum = 99; //Low Pass filter C is Nothing
    FactoryData.LP_D_BandNum = 99; //Low Pass filter D is Nothing
    Serial.println(F("{MIN} No factory data found for Model # and Reference frequency, guessing on values"));
  }

  if (LoadFromEPROM(UserSpace)) //Read all UserSpace data from EEPROM at position 0
  {
    CurrentMode = GadgetData.StartMode;
    GadgetData.WSPRData.CallSign[6] = 0;//make sure Call sign is null terminated in case of incomplete data saved
    GadgetData.WSPRData.MaidenHead4[4] = 0; //make sure Maidenhead locator is null terminated in case of incomplete data saved
  }
  else //No user data was found in EEPROM, set some defaults
  {
    CurrentMode = SignalGen ;
    GadgetData.Name[0] = 'W'; GadgetData.Name[1] = 'S'; GadgetData.Name[2] = 'P'; GadgetData.Name[3] = 'R';
    GadgetData.Name[4] = ' '; GadgetData.Name[5] = 'T'; GadgetData.Name[6] = 'X'; GadgetData.Name[7] = 0;
    GadgetData.StartMode = Idle;
    GadgetData.WSPRData.CallSign[0] = 'A'; GadgetData.WSPRData.CallSign[1] = 'A'; GadgetData.WSPRData.CallSign[2] = '0';
    GadgetData.WSPRData.CallSign[3] = 'A'; GadgetData.WSPRData.CallSign[4] = 'A'; GadgetData.WSPRData.CallSign[5] = 'A';
    GadgetData.WSPRData.CallSign[6] = 0;
    GadgetData.WSPRData.LocatorOption  = GPS;
    GadgetData.WSPRData.MaidenHead4[0] = 'A'; GadgetData.WSPRData.MaidenHead4[1] = 'A';
    GadgetData.WSPRData.MaidenHead4[2] = '0'; GadgetData.WSPRData.MaidenHead4[3] = '0';
    GadgetData.WSPRData.MaidenHead4[4] = 0;//Null termination
    GadgetData.WSPRData.TXPowerdBm = 23;   //Set deafult power to 0.2W
    if (Product_Model == 1017) //The WSPR mini
    {
      GadgetData.WSPRData.TXPowerdBm = 13; // WSPR Mini has 20mW output power
    }
    for (int i = 0; i <= 16; i++)
    {
      GadgetData.TXOnBand [i] = false; //Disable TX on all bands.
    }
    GadgetData.TXOnBand [6] = true; //enable TX on 20m
    GadgetData.TXPause = 120;  //Number of minutes to pause after transmisson
    GadgetData.GeneratorFreq = 2000000000;
    Serial.println(F("{MIN} No user data was found, setting default values"));
  }
  if (Product_Model == 1017) //The WSPR mini dont have any relays, it's Status LED is on a different pin and it has a pin to power the Si5351
  {
    StatusLED = A2;
    pinMode(SiPower, OUTPUT);
    digitalWrite(SiPower, LOW);  //Turn on power to the Si5351
  }
  else  //Other models than the WSPR TX Mini
  {
    StatusLED = 4;
    if ((FactoryData.HW_Version == 1) & (FactoryData.HW_Revision == 4)) // Early WSPR Desktop hardware had different Relay driving electronics
    {
      //De-energize all relays
      pinMode(Relay1, INPUT);
      pinMode(Relay2, INPUT);
      pinMode(Relay3, INPUT);
    }
    else
    {
      //De-energize all relays
      pinMode(Relay1, OUTPUT);
      pinMode(Relay2, OUTPUT);
      pinMode(Relay3, OUTPUT);
      digitalWrite(Relay1, LOW);
      digitalWrite(Relay2, LOW);
      digitalWrite(Relay3, LOW);
    }
  }
  // i2cInit();
  // si5351aOutputOff(SI_CLK0_CONTROL);



  // Use the Red LED as a Transmitt indicator and the Yellow LED as Status indicator
  pinMode(StatusLED, OUTPUT);
  pinMode(TransmitLED, OUTPUT);

  //Blink StatusLED to indicate Reboot
  for (int i = 0; i <= 7; i++) {
    digitalWrite(StatusLED, HIGH);
    delay (30);
    digitalWrite(StatusLED, LOW);
    delay (30);
  }
  random(RandomSeed());
  PowerSaveOFF();

  
  switch (CurrentMode) {
    case SignalGen :
      DoSignalGen();
      break;

    case WSPRBeacon:
      CurrentBand = 0;
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
  gps.encode(GPSSerial.read());
  if (gps.location.isUpdated())
  {
    GPSH = gps.time.hour();
    GPSM = gps.time.minute();
    GPSS = gps.time.second();
    SendAPIUpdate(UMesTime);
  }

  if (gps.location.isValid())
  {
    SendAPIUpdate(UMesGPSLock);
    if (GadgetData.WSPRData.LocatorOption == GPS) { //If GPS should update the Maidenhead locator
      calcLocator (gps.location.lat(), gps.location.lng());
      SendAPIUpdate(UMesLocator);
    }
  }
  else
  {
    SendAPIUpdate(UMesNoGPSLock);
  }
  smartdelay(500);
  /*
  
  digitalWrite(StatusLED, HIGH);
  delay(10);
  digitalWrite(StatusLED, LOW);
  GoToSleep (8);
  */
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
            CurrentBand = 0;
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


      //Set Low Pass filter (LP filters are automatically set by the WSPR Beacon and Signal Gen. routines but can be temporarily overrided by this command for testing purposes)
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'L')) {
        if (InputCMD[6] == 'S') { //Set option
          if (InputCMD[8] == 'A') {
            CurrentLP = 0;
          }
          if (InputCMD[8] == 'B') {
            CurrentLP = 1;
          }
          if (InputCMD[8] == 'C') {
            CurrentLP = 2;
          }
          if (InputCMD[8] == 'D') {
            CurrentLP = 3;
          }
          DriveLPFilters ();
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

    //Factory data
    if (InputCMD[1] == 'F') {

      //Product model Number
      if ((InputCMD[2] == 'P') && (InputCMD[3] == 'N')) {
        if (InputCMD[6] == 'G')
        { //Get option
          Serial.print (F("{FPN} "));
          if (Product_Model < 10000) Serial.print ("0");
          Serial.println (Product_Model);
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

      //Software Version
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'V')) {
        if (InputCMD[6] == 'G') { //Get option
          Serial.print (F("{FSV} "));
          Serial.println (SoftwareVersion);
        }
      }//Software Version

      //Software Revision
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'G') { //Get option
          Serial.print (F("{FSR} "));
          Serial.println (SoftwareRevision);
        }
      }//Software Revision

      //Low pass filter config
      if ((InputCMD[2] == 'L') && (InputCMD[3] == 'P')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[10]; CharInt[1] = InputCMD[11];
          CharInt[2]  = 0;
          switch (InputCMD[8]) {
            case 'A':
              FactoryData.LP_A_BandNum = atoi(CharInt);
              break;
            case 'B':
              FactoryData.LP_B_BandNum = atoi(CharInt);
              break;
            case 'C':
              FactoryData.LP_C_BandNum = atoi(CharInt);
              break;
            case 'D':
              FactoryData.LP_D_BandNum = atoi(CharInt);
              break;
          }

        }//Set
        else //Get Option
        {
          Serial.print (F("{FLP} A "));
          if (FactoryData.LP_A_BandNum < 10) Serial.print ("0");
          Serial.println (FactoryData.LP_A_BandNum);
          Serial.print (F("{FLP} B "));
          if (FactoryData.LP_B_BandNum < 10) Serial.print ("0");
          Serial.println (FactoryData.LP_B_BandNum);
          Serial.print (F("{FLP} C "));
          if (FactoryData.LP_C_BandNum < 10) Serial.print ("0");
          Serial.println (FactoryData.LP_C_BandNum);
          Serial.print (F("{FLP} D "));
          if (FactoryData.LP_D_BandNum < 10) Serial.print ("0");
          Serial.println (FactoryData.LP_D_BandNum);
        }
      }//Low pas filter config

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


void DoSignalGen ()
{
  CurrentMode = SignalGen;
  PickLP(BandNumOfHigestLP()); //Use the Low Pass filter that has the highest freq or is just a link
  freq = GadgetData.GeneratorFreq;
  si5351aSetFrequency(freq);
  digitalWrite(StatusLED, HIGH);
  SendAPIUpdate (UMesCurrentMode);
  SendAPIUpdate (UMesFreq);
}


void DoIdle ()
{
  PowerSaveOFF();
  CurrentMode = Idle;
  digitalWrite(StatusLED, LOW);
  si5351aOutputOff(SI_CLK0_CONTROL);
  SendAPIUpdate (UMesCurrentMode);
}


void DoWSPR ()
{
  // static double Lat;
  // static double Lon;
  PowerSaveOFF();  //Make sure GPS is not in sleep mode
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
          
          if ((GPSS == 0) && ((GPSM % 2) == 0))//If second is zero at even minute then start WSPR transmission
          {
            set_tx_buffer();// Encode the message in the transmit buffer
            SendWSPRBlock (); //Send a WSPR "Packet" for 1 minute and 50 seconds
            if (LastFreq ())  //If al bands have been transmitted on then pause for user defined time and then start over on the first band again
            {
              if ((GadgetData.TXPause > 60) && (Product_Model == 1017)) //More than a 60 sec. of TX delay then do power saving (WSPR-TX Mini only)
              {
                PowerSaveON(); //Save power by putting the GPS to sleep and power off the Si5351 PLL during this long TX Paus
                //delay (1000UL * (GadgetData.TXPause - 30)); //Do noting until there is 30 seconds left of delay
                //GoToSleep (GadgetData.TXPause - 30);               //Set MCU in sleep mode until there is 30 seconds left of delay
                smartdelay (1000UL * (GadgetData.TXPause - 30)); //Do noting until there is 30 seconds left of delay, update GUI every second
                PowerSaveOFF();// Turn on GPS and PLL again
                smartdelay(2000); // let the smartdelay routine read a few GPS lines so we can get the new GPS time after our sleep
              }
              else
              { //Less than a minute of TX delay
                smartdelay(GadgetData.TXPause * 1000UL); //Pause for a second and then recheck if top of the minute
              }
            }
            NextFreq();// get the frequency for the next HAM band that we will transmit on
            freq = freq + (100ULL * random (-100, 100)); //modify the TX frequency with a random value beween -100 and +100 Hz to avoid lengthy colisions
            smartdelay(3000);
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
  GPSGoToSleep();//Put GPS to sleep to save power
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
  GPSWakeUp();
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
  unsigned long EndTime = ms + millis();

  do
  {
    while (GPSSerial.available()) gps.encode(GPSSerial.read()); //If GPS data available - process it
    TimeLeft = EndTime - millis();
    if ((TimeLeft > 1000) && ((TimeLeft % 1000) < 20)) {
      //Send API update every  second
      Serial.print (F("{MPS} "));
      Serial.println (TimeLeft / 1000);
    }
  } while ((TimeLeft > 0) && (!Serial.available())) ; //Until time is up or there is serial data received
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


// Switches off Si5351a output
void si5351aOutputOff(uint8_t clk)
{
  i2cSendRegister(clk, 0x80); // Refer to SiLabs AN619 to see
  //bit values - 0x80 turns off the output stage
  digitalWrite(TransmitLED, LOW);
  SendAPIUpdate(UMesTXOff);
}


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
    //frequency = frequency * 128ULL; //Set base freq 128 times higher as we are dividing with 128 in the last output stage
    Divider = 90000000000ULL / (frequency * 128ULL);// Calculate the division ratio. 900,000,000 is the maximum internal freq

    pllFreq = Divider * frequency * 128ULL; // Calculate the pllFrequency:
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
  Serial.println (uint64ToStr(frequency, false));
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

//Returns true if the user has not enabled any bands for TX
boolean NoBandEnabled(void)
{
  boolean NoOne = true;
  for (int FreqLoop = 0; FreqLoop < 13; FreqLoop++) {
    if (GadgetData.TXOnBand [FreqLoop]) NoOne = false;
  }
  return NoOne;
}

//Determine what band to transmitt on, cycles upward in the TX enabled bands, e.g if band 2,5,6 and 11 is enbled for TX then the cycle will be 2-5-6-11-2-5-6-11-...
void NextFreq (void)
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
    } while (!GadgetData.TXOnBand [CurrentBand]);
    //Serial.print(F("{MIN} CurrentBand="));
    //Serial.println(CurrentBand);
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
    //We have found what band to use, now pick the right low pass filter for this band
    PickLP (CurrentBand);
  }
}

//Function returns True if the band we just transmitted on was the highest band the user want to transmitt on.
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
      Serial.println (uint64ToStr(freq, false));
      break;

    case UMesTXOn:
      Serial.println (F("{TON} T"));
      break;

    case UMesTXOff:
      Serial.println (F("{TON} F"));
      break;

    case UMesLPF:
      if (CurrentLP == LP_A) Serial.println   (F("{LPI} A"));
      if (CurrentLP == LP_B) Serial.println   (F("{LPI} B"));
      if (CurrentLP == LP_C) Serial.println   (F("{LPI} C"));
      if (CurrentLP == LP_D) Serial.println   (F("{LPI} D"));
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

//Pulls the correct relays to choose LP filter A,B,C or D
void DriveLPFilters ()
{
  if (Product_Model == 1017)
  {
    //If its the WSPR-TX Mini then do nothing as it dont have any relays
  }
  else
  {
    SendAPIUpdate (UMesLPF);
    //Product model 1011 E.g WSPR-TX LP1, this will drive the relays on the optional Mezzanine LP4 card
    if ((Product_Model == 1011) || (Product_Model == 1020))
    {
      switch (CurrentLP) {
        case LP_A:
          //all relays are at rest
          digitalWrite(Relay2, LOW);
          digitalWrite(Relay3, LOW);
          //Serial.println(F("{MIN} Mezzanine Filter A in use"));
          break;
        case LP_B:
          digitalWrite(Relay2, HIGH);
          digitalWrite(Relay3, LOW);
          //Serial.println(F("{MIN} Mezzanine Filter B in Use"));
          break;
        case LP_C:
          digitalWrite(Relay2, LOW);
          digitalWrite(Relay3, HIGH);
          //Serial.println(F("{MIN} Mezzanine Filter C in Use"));
          break;
        case LP_D:
          digitalWrite(Relay2, HIGH);
          digitalWrite(Relay3, HIGH);
          //Serial.println(F("{MIN} Mezzanine Filter D in use"));
          break;
      }//Case
    }//If Product_Model == 1011
    else
    {
      //is not Product Model 1011 and is Hardware version 1.4 E.g en early model of the Desktop transmitter
      if ((FactoryData.HW_Version == 1) && (FactoryData.HW_Revision == 4)) // Early Hardware has different relay driving
      {
        switch (CurrentLP) {
          case LP_A:
            //all relays are at rest
            pinMode(Relay1, INPUT);//Set Relay1 as Input to deactivate the relay
            pinMode(Relay2, INPUT);//Set Relay2 as Input to deactivate the relay
            pinMode(Relay3, INPUT);//Set Relay3 as Input to deactivate the relay
            //Serial.println(F("{MIN} 1.4 Filter A"));
            break;
          case LP_B:
            pinMode(Relay1, OUTPUT);//Set Relay1 as Output so it can be pulled low
            digitalWrite(Relay1, LOW);
            pinMode(Relay2, INPUT);//Set Relay2 as Input to deactivate the relay
            pinMode(Relay3, INPUT);//Set Relay3 as Input to deactivate the relay
            //Serial.println(F("{MIN} 1.4 Filter B"));
            break;
          case LP_C:
            pinMode(Relay1, INPUT);//Set Relay1 as Input to deactivate the relay
            pinMode(Relay2, INPUT);//Set Relay2 as Input to deactivate the relay
            pinMode(Relay3, OUTPUT);//Set Relay3 as Output so it can be pulled low
            digitalWrite(Relay3, LOW);
            //Serial.println(F("{MIN} 1.4 Filter C"));
            break;
          case LP_D:
            pinMode(Relay1, INPUT);//Set Relay1 as Input to deactivate the relay
            pinMode(Relay2, OUTPUT);//Set Relay2 as Output so it can be pulled low
            digitalWrite(Relay2, LOW);
            pinMode(Relay3, OUTPUT);//Set Relay3 as Output so it can be pulled low
            digitalWrite(Relay3, LOW);
            //Serial.println(F("{MIN} 1.4 Filter D"));
            break;
        }
      }
      else
      {
        //Not Product Model 1011 and not Hardvare version 1.4 E.g later model of the Desktop transmitter
        switch (CurrentLP) {
          case LP_A:
            //all relays are at rest
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, LOW);
            digitalWrite(Relay3, LOW);
            //Serial.println(F("{MIN} Not 1.4 LP Filter A in use"));
            break;
          case LP_B:
            digitalWrite(Relay1, HIGH);
            digitalWrite(Relay2, LOW);
            digitalWrite(Relay3, LOW);
            //Serial.println(F("{MIN} Not 1.4 LP Filter B in Use"));
            break;
          case LP_C:
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, LOW);
            digitalWrite(Relay3, HIGH);
            //Serial.println(F("{MIN} Not 1.4LP Filter C in Use"));
            break;
          case LP_D:
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, HIGH);
            digitalWrite(Relay3, HIGH);
            //Serial.println(F("{MIN} Not 1.4 LP Filter D in use"));
            break;
        }
      }
    }
  }
}

//Out of the four possible LP filters fitted - find the one that is best for Transmission on TXBand
void PickLP (uint8_t TXBand)
{
  boolean ExactMatch = false;
  uint8_t BandLoop;

  //Check if some of the four low pass filters is an exact match for the TXBand
  if  (FactoryData.LP_A_BandNum == TXBand)
  {
    ExactMatch = true;
    CurrentLP = LP_A;
  }
  if  (FactoryData.LP_B_BandNum == TXBand)
  {
    ExactMatch = true;
    CurrentLP = LP_B;
  }
  if  (FactoryData.LP_C_BandNum == TXBand)
  {
    ExactMatch = true;
    CurrentLP = LP_C;
  }
  if  (FactoryData.LP_D_BandNum == TXBand)
  {
    ExactMatch = true;
    CurrentLP = LP_D;
  }

  //If we did not find a perfect match then use a low pass filter that is higher in frequency.
  if (!ExactMatch)
  {
    for (BandLoop = TXBand; BandLoop < 99; BandLoop ++)  //Test all higher bands to find a a possible LP filter in one of the four LP banks
    {
      if (FactoryData.LP_A_BandNum == BandLoop) //The LP filter in Bank A is a match for this band
      {
        CurrentLP = LP_A;
        break;
      }
      if (FactoryData.LP_B_BandNum == BandLoop) //The LP filter in Bank B is a match for this band
      {
        CurrentLP = LP_B;
        break;
      }
      if (FactoryData.LP_C_BandNum == BandLoop) //The LP filter in Bank C is a match for this band
      {
        CurrentLP = LP_C;
        break;
      }
      if (FactoryData.LP_D_BandNum == BandLoop) //The LP filter in Bank D is a match for this band
      {
        CurrentLP = LP_D;
        break;
      }
    }
    //If there is no LP that is higher than TXBand then use the highest one, (not ideal as output will be attenuated but best we can do)
    if (BandLoop == 99) {
      TXBand = BandNumOfHigestLP();
      if  (FactoryData.LP_A_BandNum == TXBand)
      {
        CurrentLP = LP_A;
      }
      if  (FactoryData.LP_B_BandNum == TXBand)
      {
        CurrentLP = LP_B;
      }
      if  (FactoryData.LP_C_BandNum == TXBand)
      {
        CurrentLP = LP_C;
      }
      if  (FactoryData.LP_D_BandNum == TXBand)
      {
        CurrentLP = LP_D;
      }
    }
  }
  DriveLPFilters ();
}

//Returns a band that is the highest band that has a LP filter fitted onboard.
//Low pass filter numbering corresponds to Bands or two special cases
//The special cases are: 98=just a link between input and output, 99=Nothing fitted (open circut) the firmware will never use this
//These numbers are set by the factory Configuration program and stored in EEPROM
uint8_t BandNumOfHigestLP () {
  uint8_t BandLoop, Result;
  Result = FactoryData.LP_A_BandNum ; //Use this filter if nothing else is a match.
  //Find the highest band that has a Low Pass filter fitted in one of the four LP banks
  for (BandLoop = 98; BandLoop > 0; BandLoop--) {
    if (FactoryData.LP_A_BandNum == BandLoop) //The LP filter in Bank A is a match for this band
    {
      Result = FactoryData.LP_A_BandNum ;
      break;
    }
    if (FactoryData.LP_B_BandNum == BandLoop) //The LP filter in Bank B is a match for this band
    {
      Result = FactoryData.LP_B_BandNum ;
      break;
    }
    if (FactoryData.LP_C_BandNum == BandLoop) //The LP filter in Bank C is a match for this band
    {
      Result = FactoryData.LP_C_BandNum ;
      break;
    }
    if (FactoryData.LP_D_BandNum == BandLoop) //The LP filter in Bank D is a match for this band
    {
      Result = FactoryData.LP_D_BandNum ;
      break;
    }
  }
  return Result;
}

//Retun VCC voltage measured in milliVolt
//So 5000 is 5V, 3300 is 3.3V.
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void PowerSaveOFF()
{
  GPSWakeUp ();
  Si5351PowerOn ();
}

void PowerSaveON()
{
  GPSGoToSleep();
  Si5351PowerOff ();
}

void GPSGoToSleep()
{
  if (Product_Model == 1017)
  { //If its the WSPR-TX Mini it has a Quectel L80-R GPS, send the correct string to it put it to sleep
    GPSSerial.println("$PMTK161,0*28");
    GPSSleep = true;
  }
}

void GPSWakeUp ()
{
  if (Product_Model == 1017)
  { //If its the WSPR-TX Mini it has a Quectel L80-R GPS,  Send something to wake it up
    GPSSerial.println(" ");
    GPSSleep = false;
  }
}

void Si5351PowerOff ()
{
  if (Product_Model == 1017)//If its the WSPR-TX Mini it has a control line that can cut power to the Si5351
  {
    //Power off the Si5351
    digitalWrite(SiPower, HIGH);
  }
}

void Si5351PowerOn ()
{
  if (Product_Model == 1017)//If its the WSPR-TX Mini it has a control line that can cut power to the Si5351
  {
    //Power on the Si5351
    digitalWrite(SiPower, LOW);
    //Give it some time to stabilize voltage before init
    delay (100);
    //re-initialize the Si5351
    i2cInit();
    si5351aOutputOff(SI_CLK0_CONTROL);
  }
}

  //Sleep code from Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M
void GoToSleep( int SleepTime)//Sleep time in seconds, accurate to the nearest 8 seconds
{
  int SleepLoop;
  SleepLoop = SleepTime / 8 ; // every sleep period is 8 seconds
  GPSGoToSleep();
  GPSSerial.end();//Must turn off software serialport or sleep will not work
  Serial.end(); //Turn off Hardware serail port as well as we will temporary change all ports to outputs
  AllIOtoLow ();  //Set all IO pins to outputs to save power
  DisableADC ();  //Turn off ADC to save power

  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1 << 6); //enable interrupt mode
   
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep
  for (int i = 0; i < SleepLoop; i++)
  {
    //BOD DISABLE - this must be called right before the __asm__ sleep instruction
    MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
    __asm__  __volatile__("sleep");//in line assembler to go to sleep
    digitalWrite(StatusLED, HIGH);
    delay (30);
    digitalWrite(StatusLED, LOW);
  }
  //Restore everything
   EnableADC ();
  GPSSerial.begin(9600); //Init software serial port to communicate with the on-board GPS module
  Serial.begin (9600);   //Init hardware Serial port
  Serial.setTimeout(2000);
  Si5351PowerOn ();
}
void AllIOtoLow ()
{
  //  Save Power by writing all Digital IO LOW - note that pins just need to be tied one way or another, do not damage devices!
   for (int i = 0; i < 20; i++) {
    pinMode(i, OUTPUT);
  }
  //The Pin that controlls power to the Si5351 must be an output and tied high to turn off power to Si5351
  pinMode(SiPower, OUTPUT);
  digitalWrite(SiPower, HIGH);
}

void DisableADC ()
{
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);
}

void EnableADC ()
{
  //Enable ADC again
  ADCSRA |= (1 << 7);
}

ISR(WDT_vect) {
  //DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
}// watchdog interrupt
