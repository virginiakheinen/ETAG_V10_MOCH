/*
  Data logging sketch for the ETAG RFID Reader
  ETAG READER Version 10.0
  PROCESSOR SAMD21J
  USE BOARD DEFINITION: ETAG RFID V2 D21
  Code by:
   Jay Wilhelm
   Tho Trinh
   Eli Bridge
   Alexander Moreno
   David Mitchell

  Sept 2019

  Licenced in the public domain

  REQUIREMENTS:
  Power supply for the board should come from the USB cable or a 5V battery or DV power source.
  A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
      and time when the board is disconnected from a primary power source.

  LIBRARIES:
  The RV3129 library is accessible on github here:
  https://github.com/OUIDEAS/RV-3129_Arduino_Library.git

  FLASH MEMORY STRUCTURE:
  The onboard flash memory is divided into pages of 528 bytes each. There are probably several thousand pages.
  Page 0 is reserved for parameters and counters
    //bytes 0-2 (0x400 - (0x402) - first three are the current backup memory address -- No longer used
    byte 3 - (0x403) Set to 0xAA once the memory address counter is intialized. (important mainly for the very first initialization process with a completely blank Flash memory)
    bytes 4-7 - (0x404-0x407) next four bytes are for the reader ID charaters
    byte 8 - Set to 0xAA once the Reader ID is established - if this byte is anything other than 0xAA then the Reader ID will be set to the default
    byte 0xA feeder mode (O A or T)
    bytes 0x0B and 0x0C are for GPS scheduling.
    byte 13 = Logging mode - log to SD in real time or just use Flash
  Pages 1  to 16 is reserved for RFID tag codes (list A).
  Page 17 is reserved for the list schedule.
  Backup data starts on page defined by the variable - dataStartPage

  Nov 8, 2019 - Added Memory address lookup - address pointer no longer used.
  Nov 10, 2019 - Added dual logging modes.
  Nov 10, 2019 - Fixed bug in sleepTimer function that stopped the clock.
  Dec 15, 2019 - Changed default mode to no-SD, added LED flash when logging starts

  Jan 2020 - incorporated chickadee feeder functions and routines
  Feb 6, 2020 - added door close failure routine.

  Aug 2020 - changed flash memory addressing to page and byte variables (instead of 4-byte fAddress)
  Aug 2020 - changed RFID storage so that the RF circuit number comes first, this eliminates an addressing problem that could occur if the RFID tag number starts with FF
  Aug 2020 - extensive revision to tag lists. Now tag list are specified by a page address. Byte 0 indicated the number of tags, the rest of the bytes store the tags.
*/


// ***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
#include "RV3129.h"          // include library for the real time clock - must be installed in libraries folder
#include <Wire.h>            // include the standard wire library - used for I2C communication with the clock
#include <SD.h>              // include the standard SD card library
#include <SPI.h>             // include standard SPI library
#include "Manchester.h"
#include <wiring_private.h> // pinPeripheral() function

// define hardware serial port
Uart Serial2 (&sercom1, 13, 10, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

#define serial SerialUSB       // Designate the USB connection as the primary serial comm port - note lowercase "serial"
//#define DEMOD_OUT_1      41  // (PB22) this is the target pin for the raw RFID data from RF circuit 1
//#define DEMOD_OUT_2      42  // (PB23) this is the target pin for the raw RFID data from RF circuit 2
//#define SHD_PINA         48  // (PB16) Setting this pin high activates RFID circuit 1
//#define SHD_PINB         49  // (PB17) Setting this pin high activates RFID circuit 2
#define SDselect           46  // (PA21) Chip select for SD card. Make this pin low to activate the SD card for SPI communication
#define SDon               45  // (PA06) Provides power to the SD via a high side mosfet. Make this pin low to power on the SD card
#define FlashCS            44  // (PA05) Chip select for flash memory. Make this pin low to activate the flash memory for SPI communication
#define LED_RFID           43  // (PA27) Pin to control the LED indicator.
#define INT1               47  // (PA20) Clock interrupt for alarms and timers on the RTC
//#define MOTR               2   // used for sleep function (need to investigate this).
RV3129 rtc;   //Initialize an instance for the RV3129 real time clock library.

//Motor controller pins
#define MOTPWMA         6  //motor pulse width
#define MOTL            5  //motor left
#define MOTR            4  //motor right
#define mStby           7  //motor controller standby
#define MOTPWMB         1  //motor controller pulse width B 
#define gpsPwr1         3  //GPS power control - connected to B_in_1 of motor controller
#define gpsPwr2         2  //GPS power control - connected to B_in_2 of motor controller 
#define mSwitch         12 //motor switch input
#define mSwitchGnd      11 //optional ground for the motor switch

// ************************* initialize variables******************************
char deviceID[5] = "RFID";            // User defined name of the device
String deviceIDstr;                   // String verion of device name.
String dataFile;                      // Stores text file name for SD card writing.
String logFile;                       // Stores text file name for SD card writing.
String feederNameStr;
String fileNameSD;
uint32_t fAddress;                    // Address pointer for logging to backup memory
uint32_t fAddr = 0x800;

String currRFID;                      // stores current RFID number and RF circuit - for use with delayTime
String pastRFID = "XXXXXXXXXX1";       // stores past RFID number and RF circuit - for use with delayTime
uint32_t pastHHMMSS;                  // stores the past Hour, Minute and Second - for use with delayTime

//byte tagCount;                        //keeps track of number of stored tags
char activeList = 'A';

unsigned int fMemPageAddr;         // page address for flash memory
unsigned int fMemByteAddr;         // byte address for flash memory
unsigned int dataStartPage = 18;
unsigned int timePage = 17;

byte RFcircuit = 1;                   // Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.

unsigned long curTime = 0;
unsigned long tickTime = 0;
unsigned long pastTime = 9999;
byte ss, mm, hh, da, mo, yr;          //Byte variables for storing date/time elements
String currentDate;                   //Get the current date in mm/dd/yyyy format (we're weird in the US)
String currentTime;                   //Get the time
String currentDateTime;
String timeString;                    //String for storing the whole date/time line of data
char incomingByte = 0;                 //Used for incoming serial data
unsigned int timeIn[12];              //Used for incoming serial data during clock setting
byte menu;
char feedMode;
char feederID[4];
uint16_t tagGrpSel;               // Group selection bits for indicating which pages to search for tags in target mode.
uint32_t nextTime;                // Unix time of next switch event
uint16_t nextGrpSel  = 0; 
char selBits[21];                 // character array for group select bits


// Global variable for tag codes

char RFIDstring[10];                  // Stores the TagID as a character array (10 character string)
byte RFIDtagUser = 0;                 // Stores the first (most significant) byte of a tag ID (user number)
unsigned long RFIDtagNumber = 0;      // Stores bytes 1 through 4 of a tag ID (user number)
byte RFIDtagArray[5];                 // Stores the five individual bytes of a tag ID.


// ********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const byte checkTime = 30;                          // How long in milliseconds to check to see if a tag is present (Tag is only partially read during this time -- This is just a quick way of detirmining if a tag is present or not
const unsigned int pollTime1 = 100;                 // How long in milliseconds to try to read a tag if a tag was initially detected (applies to both RF circuits, but that can be changed)
const unsigned int delayTime = 8;                   // Minimim time in seconds between recording the same tag twice in a row (only applies to data logging--other operations are unaffected)
const unsigned long pauseTime = 200;                // CRITICAL - This determines how long in milliseconds to wait between reading attempts. Make this wait time as long as you can and still maintain functionality (more pauseTime = more power saved)
uint16_t pauseCountDown = pauseTime / 31.25;        // Calculate pauseTime for 32 hertz timer
byte pauseRemainder = ((100 * pauseTime) % 3125) / 100; // Calculate a delay if the pause period must be accurate
//byte pauseRemainder = 0 ;                         // ...or set it to zero if accuracy does not matter

const byte slpH = 22;                            // When to go to sleep at night - hour
const byte slpM = 00;                            // When to go to sleep at night - minute
const byte wakH = 05;                            // When to wake up in the morning - hour
const byte wakM = 00;                            // When to wake up in the morning - minute
const unsigned int slpTime = slpH * 100 + slpM;  // Combined hours and minutes for sleep time
const unsigned int wakTime = wakH * 100 + wakM;  // Combined hours and minutes for wake time

const byte motorPresent = 1;

/* The reader will output Serial data for a certain number of read cycles;
   then it will start using a low power sleep mode during the pauseTime between read attempts.
   The variable stopCycleCount determines how many read cycles to go
   through before using the low-power sleep.
   Once low-power sleep is enabled, the reader will not be able to output
   serial data, but tag reading and data storage will still work.
*/
unsigned int cycleCount = 0;          // counts read cycles
unsigned int stopCycleCount = 50000;     // How many read cycles to maintain serial comminications
bool Debug = 1;                       // Use to stop serial messages once sleep mode is used.
byte SDOK = 1;
char logMode;

byte doorState = 1;                              //Status of the motorized door. 0 = open, 1 = closed.
byte birdFed = 0;

//GPS variables
uint32_t timeVal; //For GPS
uint32_t dateVal; //For GPS
uint32_t dateVal_clk = 0; //For GPS
uint32_t timeVal_old = 0; //For GPS
uint32_t dateVal_old = 0; //For GPS
byte GPSstatus = 0;  // status flag for GPS time updates. 0 = dormant, 1 = waiting for connection.
uint16_t GPScounter = 0; // used to count minutes between GPS clock calibrations
uint32_t GPSacquireA;  //GPS aquisition time variable.
int timeOffset = -7;                  //Used to convert from GPS time to local time
unsigned int timeCalFreq = 9999;      //How often to calibrate the clock with GPS time (minutes)
unsigned int calFreq = 9999;          //used for GPS time calibration
unsigned int calFreqInc = 5;          //used for GPS time calibration
String GPSTimeString;


//////SETUP//////////////SETUP//////////////SETUP//////////////SETUP////////

void setup() {  // setup code goes here, it is run once before anything else

  pinMode(LED_RFID, OUTPUT);      // pin for controlling the on-board LED
  digitalWrite(LED_RFID, HIGH);   // turn the LED off (LOW turns it on)
  pinMode(SDselect, OUTPUT);      // Chip select pin for SD card must be an output
  pinMode(SDon, OUTPUT);          // Chip select pin for SD card must be an output
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory
  digitalWrite(SDon, LOW);        // turns on the SD card.
  digitalWrite(SDselect, HIGH);   // Make both chip selects high (not selected)
  digitalWrite(FlashCS, HIGH);    // Make both chip selects high (not selected)
  pinMode(SHD_PINA, OUTPUT);      // Make the primary RFID shutdown pin an output.
  digitalWrite(SHD_PINA, HIGH);   // turn the primary RFID circuit off (LOW turns on the EM4095)
  pinMode(SHD_PINB, OUTPUT);      // Make the secondary RFID shutdown pin an output.
  digitalWrite(SHD_PINB, HIGH);   // turn the secondary RFID circuit off (LOW turns on the EM4095)
  pinMode(INT1, INPUT);           // Make the alarm pin an input

  //Motor control pin initializaiton
  pinMode(mStby, OUTPUT);         // pin that can put the motor controller in low-power standby mode
  digitalWrite(mStby, LOW);       // standby pin set low (motor off)
  pinMode(MOTR, OUTPUT);          // Right motor control pin
  digitalWrite(MOTR, LOW);        // turn motor off
  pinMode(MOTL, OUTPUT);          // Left motor control pin
  digitalWrite(MOTL, LOW);        // turn motor off
  pinMode(MOTPWMA, OUTPUT);       // pulse widtch (speed control) for motor controller circuit A
  digitalWrite(MOTPWMA, LOW);     // turn pulse off

  pinMode(gpsPwr1, OUTPUT);       // pin that powers the GPS via the motor controller
  digitalWrite(gpsPwr1, LOW);     // turn off
  pinMode(MOTPWMB, OUTPUT);       // pulse widtch (speed control) for motor controller circuit B
  digitalWrite(MOTPWMB, LOW);     // turn pulse off

  pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
  pinMode(mSwitchGnd, OUTPUT);    // pin that provides a ground reference for the motor switch (optional comment out if not used)
  digitalWrite(mSwitchGnd, LOW);     // make low always. (optional comment out if not used)


  blinkLED(LED_RFID, 4, 400);     // blink LED to provide a delay for serial comms to come online
  serial.begin(9600);
  serial.println("Start");

  //Check flash memory and initialize if needed
  serial.print("Is flash memory initialized? ");    //It is neccessary to initialize the flash memory on the first startup
  byte byte0 = readFlashByte(0, 3);                 //Read a particular byte from the flash memory
  if (byte0 != 0xAA) {                              //If the byte is 0xFF then the flash memory needs to be initialized
    serial.println("NO!");                          //Message
    serial.println("Initializing Flash Memory..."); //Message
    writeFlashByte(0, 3, 0xAA);                     //Write a different byte to this memory location
    serial.println("Setting default device ID - Please update this!!!");
    deviceID[0] = 'R'; deviceID[1] = 'F'; deviceID[2] = '0'; deviceID[3] = '1';
    writeFlashArray(0, 4, deviceID, 4);  //write to flash memory without updating flash address
  } else {
    serial.println("YES");                          //Confirm initialization
  }

  // Set up SD card communication
  serial.println("Initializing SD card...");       // message to user
  if (SDwriteString("Message", "DUMMY.TXT")) {   // Write a dummy file to the SD card
    serial.println("SD card OK");                // OK message
    SDremoveFile("DUMMY.TXT");                   // delete the dummy file
    if (logMode == 'F') {
      writeMem(0); //Write flash memory
    }
  } else {
    serial.println("SD card not detected.");       // error message
    if (logMode == 'S') {
      ;
      for (byte i = 0; i < 4; i++) {
        serial.println("SD card not detected.");       // error message
        blinkLED(LED_RFID, 1, 1000);                 // LED on for warning
        SDOK = 2;                       //indicates SD card not detected
      }
    }
  }


  // start clock functions and check time
  Wire.begin();  //Start I2C bus to communicate with clock.
  if (rtc.begin() == false) {  // Try initiation and report if something is wrong
    serial.println("Something wrong with clock");
  } else {
    rtc.set24Hour();
  }

  if (SDOK == 1) {
    String loadFile = checkForLoadFile();
    serial.println(loadFile);
    if (loadFile != "na") {
      serial.println("Loading new parameters...");
      loadParameters(loadFile);

      readFlashArray(0, 4, deviceID, 4);   //Read in device ID
      deviceIDstr = String(deviceID);

      String tagFile = String(deviceIDstr +  "TAGS.TXT");
      serial.print("Updating tag lists from ");
      serial.println(tagFile);
      transferTags(tagFile, 1);

      String timeFile = String(deviceIDstr +  "TIME.TXT");
      serial.print("Updating switch times from ");
      serial.println(timeFile);
      transferTimes(timeFile, 17);

      rtc.updateTime();
      updateTimeVars();
      readTimes(makeUnixTime(yr, mo, da, hh, mm, ss), 0);
      blinkLED(LED_RFID, 16, 50);
    }
  }



  //Read in parameters and display parameters from the flash memory
  //Get and display the device ID
  readFlashArray(0, 4, deviceID, 4);   //Read in device ID
  //serial.println(deviceID);             //Display device ID

  //Find the flash address
  fAddr = FlashGetAddr(12);
  //fMemPageAddr <- fAddr >> 10
  //fMemByteAddr <- fAddr & 0xFFFFF

  logMode = readFlashByte(0, 0x0D);
  if (logMode != 'S' & logMode != 'F') { //If log mode is not established then do so
    serial.println("Setting log mode to SD card");
    writeFlashByte(0, 0x0D, 0x53);    //hex 53 is "S"
    logMode = 'S';
    //serial.println("Setting log mode to Flash mode");
    //writeFlashByte(0, 0x0D, 0x46);    //hex 46 is "F"
    //logMode = 'F';
  }
  if (logMode == 'S') {
    serial.println("Logging mode S");
    serial.println("Data saved immediately SD card and Flash Mem");
  } else {
    serial.println("Logging mode F");
    serial.println("Data saved to Flash Mem only (no SD card needed)");
    SDOK = 0;     //Turns off SD logging
  }


  serial.println(F("Initializing motor...."));  //Message to user
  if (motorPresent) {
    motInit(); //Function that allows the door system to recognize its current position and move to the closed position
  }

  //////////////MENU////read1 = myfile.read()////////MENU////////////MENU////////////MENU////////
  //Display all of the following each time the main menu is called up.




  byte menu = 1;
  while (menu == 1) {
    serial.println();
    rtc.updateTime();
    serial.println(showTime());

    serial.println("Device ID: " + String(deviceID)); //Display device ID
    serial.println("Logging mode: " + String(logMode)); //Display logging mode

    serial.print("Flash memory address: Page ");      //Display flash memory address
    serial.print(fMemPageAddr, DEC);
    serial.print(" - Byte ");
    serial.println(fMemByteAddr, DEC);

    feedMode = getMode();                    //Get and show the feeder mode stored in the flash memory.
    //if(feedMode != 'A' & feedMode != 'T' & feedMode != 'O'){
    //  feedMode = 'A';
    //  feedMode = setMode('A');
    // }
    serial.println("Feeding mode: " + String(feedMode));

    //tagCount = readFlashByte(0, 09);       //Get tag count from flash memory
    //serial.println("Tags in memory: " + String(tagCount));

    timeCalFreq = (readFlashByte(0, 0x0B) << 8) + readFlashByte(0, 0x0C);
    if (timeCalFreq == 0xFFFF) { //in case this value has not been defined yet...
      timeCalFreq = 999;
    }
    if (timeCalFreq == 999) {
      serial.println("GPS time update: OFF");
    } else {
      serial.println("GPS time update: Every " + String(timeCalFreq) + " minutes");
    }
    //Define the log and data files
    deviceIDstr = String(deviceID);
    logFile = String(deviceIDstr +  "LOG.TXT");
    dataFile = String(deviceIDstr + "DATA.TXT");


    // Ask the user for instruction and display the options
    serial.println("What to do? (input capital letters)");
    serial.println("    B = Display backup memory");
    serial.println("    C = set clock");
    serial.println("    D = test Door");
    serial.println("    E = Erase (reset) backup memory");
    serial.println("    G = GPS functions");
    serial.println("    I = Set device ID");
    serial.println("    L = Load Tag IDs");                //Transfer tag IDs from a file on the SD card to the flash memory.
    serial.println("    M = Change logging mode");
    serial.println("    S = Show current Tag IDs");
    serial.println("    W = Write flash data to SD card");
    serial.println("    O, A, T = Feed modes: Open, All, Target");


    //Get input from user or wait for timeout
    char incomingByte = getInputByte(15000);
    String printThis = String("Value recieved: ") + incomingByte;
    serial.println(printThis);
    switch (incomingByte) {                                               // execute whatever option the user selected
      default:
        menu = 0;         //Any non-listed entries - set menu to 0 and break from switch function. Move on to logging data
        break;
      case 'A': {
          feedMode = setMode('A');  //set feeder mode to A: feed all tagged birds; break and show menu again
          birdFed = 0;
          break;
        }
      case 'B': {
          dumpMem(0);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'b': {
          dumpMem(1);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'C': {                                                         // option to set clock
          inputTime();                                                    // calls function to get time values from user
          rtc.updateTime();
          updateTimeVars();
          readTimes(makeUnixTime(yr, mo, da, hh, mm, ss), 0);
          serial.print("Current group selection ");
          printBits(tagGrpSel, 16, 1);
          serial.println();
          serial.print("Time for next tag group ");
          extractUnixTime(nextTime);
          char dateLine[20];
          sprintf(dateLine, "%02d/%02d/%02d %02d:%02d:%02d ",
                  mo, da, yr, hh, mm, ss);
          serial.print(dateLine);
          break;                                                        //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'D': {
          if (motorPresent) {
            doorOpen();
          }
          delay(1000);
          if (motorPresent) {
            doorClose();
          }
          break;
        }
      case 'E': {
          eraseBackup('s');
          break;   //  break out of this option, menu variable still equals 1 so the menu will display again
        }

      case 'e': {
          eraseBackup('a');
          break;   //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'G': {
          //byte timeset = 1;;
          char YorN = 'X';
          GPSon();
          timeVal = 99999;
          serial.println(F("GPS setup"));
          serial.println(F("How often should the Clock be calibrated (value in minutes)?"));
          serial.println(F("A value of 999 (16 hours, 39 minutes) turns GPS calibration off"));
          serial.println(F("Enter three digits (e.g. 036)"));  //Ask for user input
          while (serial.available() == 0) {}    //wait for 12 characters to accumulate
          for (int n = 0; n < 4; n++) {        //loop to read all the data from the serial buffer once it is ready
            timeIn[n] = serial.read();         //Read the characters from the buffer into an array of bytes one at a time
          }
          while (serial.available()) {       //Clear the buffer, in case there were extra characters
            serial.read();                  //Read in any extra characters (and then ignore them)
          }
          timeCalFreq = ((timeIn[0] - 48) * 100 + (timeIn[1] - 48) * 10 + (timeIn[2] - 48)) ; //Convert two ascii characters into a single decimal number
          setTimeCalFreq(timeCalFreq);

          serial.println(F("Update clock with GPS time? (enter Y or N)"));
          unsigned serDelay2 = 0;                                         //If there's no response then eventually move on and just start logging
          while (serial.available() == 0 && serDelay2 < 10000) { //wait about 10 seconds for a user response
            delay(1);                                           //delay 1 ms
            serDelay2++;                                         //add to the delay count
          }
          if (serial.available()) {                                             //If there is a response then perform the corresponding operation
            YorN = serial.read();
          }
          if (YorN == 'Y') {
            serial.println(F("Have to wait for satellite connection."));
            serial.println(F("This can take a couple of minutes."));
            byte breakout = 0;
            while (breakout == 0) {
              GPSon();
              timeVal = 0;
              dateVal = 0;
              parseGPS(millis() + 300);
              if (timeVal > 0 & dateVal > 0) {
                serial.println();
                serial.print(F("GPS time: "));
                printGPStime(timeVal, dateVal);  //Create a string for GPS time and set time byte variables (ss, mm, hh, da, mo, yr).
                serial.println(GPSTimeString);
                extractUnixTime(makeUnixTime(yr, mo, da, hh, mm, ss) + timeOffset * 3600);   // calculate the time offset and set new time byte variables
                serial.print(F("Old local time: "));
                serial.println(showTime());
                if (rtc.setTime(ss, mm, hh, da, mo, yr + 2000, 1) == false) {       //  attempt to set clock with input values
                  serial.println(F("Something went wrong setting the time"));
                }
                delay(20);
                serial.print(F("New local time: "));
                serial.println(showTime());
                breakout = 1;
              }
            }
          }
          GPSoff();
          rtc.updateTime();
          updateTimeVars();
          readTimes(makeUnixTime(yr, mo, da, hh, mm, ss), 0);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'I': {
          inputID();   //  calls function to get a new feeder ID
          writeFlashByte(0, 8, 0xAA);                           //Write a different byte to this memory location
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'L': {
          String tagFile = String(deviceIDstr +  "TAGS.TXT");
          transferTags(tagFile, 1);                                          //Transfer tag data from the SD card to the flash memory
          String timeFile = String(deviceIDstr +  "TIME.TXT");
          transferTimes(timeFile, 17);
          rtc.updateTime();
          updateTimeVars();
          serial.println(showTime());
          readTimes(makeUnixTime(yr, mo, da, hh, mm, ss), 0);
          serial.print("Current group selection ");
          printBits(tagGrpSel, 16, 1);
          serial.println();
          serial.print("Time for next tag group ");
          extractUnixTime(nextTime);
          char dateLine[20];
          sprintf(dateLine, "%02d/%02d/%02d %02d:%02d:%02d ",
                  mo, da, yr, hh, mm, ss);
          serial.print(dateLine);
          break;
        }
      case 'M': {
          logMode = readFlashByte(0, 0x0D);
          if (logMode != 'S') {
            writeFlashByte(0, 0x0D, 0x53);    //hex 53 is "S"
            serial.println("Logging mode S");
            serial.println("Data saved immediately SD card and Flash Mem");
            if (SDOK == 0) {
              SDOK = 1;
            }
          } else {
            writeFlashByte(0, 0x0D, 0x46);    //hex 46 is "F"
            serial.println("Logging mode F");
            serial.println("Data saved to Flash Mem only (no SD card needed)");
            SDOK = 0;
          }
          logMode = readFlashByte(0, 0x0D);
          serial.println("log mode set to: ");
          serial.println(logMode);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'O': {
          feedMode = setMode('O');  //set feeder mode to O: feeder remains open at all times; break and show menu again
          birdFed = 1;
          break;
        }
      case 'S': {
          for (byte pg = 1; pg < 17; pg++) {
            showTags(pg);
          }

          serial.println("Switch times:");
          serial.println();         

          showReadTimes();

          rtc.updateTime();
          updateTimeVars();
          uint32_t tNow = makeUnixTime(yr, mo, da, hh, mm, ss);
          readTimes(tNow, 1);
          serial.println();
          serial.print("Current list selection ");
          printBits(tagGrpSel, 16, 1);
          serial.println();
          serial.print("Next switch at ");
          extractUnixTime(nextTime);
          char dateLine3[20];
          sprintf(dateLine3, "%02d/%02d/%02d %02d:%02d:%02d",
              mo, da, yr, hh, mm, ss);
          serial.print(dateLine3);
          serial.print(" to ");
          printBits(nextGrpSel, 16, 1);
          serial.println();
          break;
          
        }
      case 'T': {
          feedMode = setMode('T');  //set feeder mode to T: feed only targeted birds; break and show menu again
          birdFed = 0;
          break;
        }
      case 'W': {
          if (SDOK == 1) {
            writeMem(0);
          } else {
            serial.println("SD card missing");
          }
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
    } //end of switch
  } //end of while(menu = 1){

  rtc.updateTime();
  String logStr = "Logging started at " + showTime();
  if (SDOK == 1) {
    SDwriteString(logStr, logFile);
  }
  RFcircuit = 1;
  birdFed = 0;                 //birdFed default is 0
  blinkLED(LED_RFID, 3, 100);
  pastTime = 0;
  calFreq = timeCalFreq;
}

////////////////////////////////////////////////////////////////////////////////
////////MAIN////////////////MAIN////////////////MAIN////////////////MAIN////////
////////////////////////////////////////////////////////////////////////////////

void loop() { // Main code is here, it loops forever:

  //Variables used in main loop
  String SDsaveString;  //used later for SD card writing.
  String currentDate;   //usde for current date in mm/dd/yyyy format
  String currentTime;   //used for current time in hh:mm:ss format
  String currentDateTime;  //conbines other strings

  ///////GPS clock calibration///////////GPS clock calibration///////
  //Check if it is time to switch lists and/or recalibrate the clock

  if (rtc.updateTime() == false) {
    serial.print(F("RTC failed at beginning of loop ")); //Updates the time from the real time clock
  }
  updateTimeVars();             // Gets the time unit variables from the RTC (ss, mm, hh, etc),
  curTime = makeUnixTime(yr, mo, da, hh, mm, ss);     // returns a unix time value based on the RTC values
  //  if(Debug) {
  //    serial.println(showTime());   // print the rtc values
  //    serial.println(F("curTime   pastTime"));
  //    serial.print(curTime);
  //    serial.print("   ")  ;
  //    serial.println(pastTime);
  //  }
//
//
//
//
//
//
//  serial.print("curTime ");
//  serial.print(curTime);
//    serial.print("    nextTime ");
//  serial.print(nextTime);
//      serial.print("    tagGrpSel ");
//  serial.println(tagGrpSel, BIN);
//
//

  if (curTime >= nextTime) {
    readTimes(curTime, 1);  //Switch tag lists.
    printBits(tagGrpSel, 16, 0);
    serial.println();
    currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird in the US)
    currentTime = rtc.stringTime(); //Get the time
    SDsaveString = String("List selection updated to ") + 
      selBits + " at " + currentDate + " " + currentTime;    // Create a data string
    if (SDOK == 1) {
      if (Debug) {serial.println(SDsaveString);}
      SDwriteString(SDsaveString, logFile); //Write log to SD card
    }
    if (Debug) {
      serial.print("Next switch at ");
      extractUnixTime(nextTime);
      char dateLine2[20];
      sprintf(dateLine2, "%02d/%02d/%02d %02d:%02d:%02d",
              mo, da, yr, hh, mm, ss);
      serial.print(dateLine2);
      serial.print(" to ");
      printBits(nextGrpSel, 16, 1);
      serial.println();
    }
  }












  if ((curTime - pastTime) / 60 >= calFreq & GPSstatus == 0 & calFreq != 999) { //calculate difference in minutes between current time and past time to see if it is time to check the GPS for a time update
    //Try to aquire a GPS clock update
    //if(Debug) { serial.println(F("Time to check GPS"));}  //print a message
    GPSacquireA = curTime;                  //note the current time
    //if(Debug) {
    //serial.print(F("GPS timing started: ")); //indicate when aquisition started
    //serial.println(GPSacquireA, DEC);}
    GPSstatus = 1;                        //Set GPS status to 1 to indicate that aquisition is in progress
    timeVal = 0;
    dateVal = 0;
    dateVal_clk = 10000 * da + 100 * mo + yr;
    GPSon();                              //Power up the gps module
    //    while (Serial2.available() > 0){
    //       byte Byte0 = Serial2.read();}       // attempt to clear the Serial2 buffer
  }

  if (GPSstatus > 0) {   //Do this whenever GPS status is 1 or 2
    if (Debug) {
      uint32_t timeOn  = curTime - GPSacquireA;
      serial.print("GPS aquisition active for ");   //Serial message
      serial.print(timeOn, DEC);
      serial.println(" secs");
    }
    parseGPS(millis() + pauseTime);
    //      if(Debug) {
    //        serial.println(F("timeVal and dateVal"));
    //        serial.print(timeVal);
    //        serial.print(F(" "));
    //        serial.println(dateVal);
    //        serial.print(timeVal_old);
    //        serial.print(F(" "));
    //        serial.println(dateVal_old);
    //      }
    if (timeVal - timeVal_old > 0 & timeVal - timeVal_old < 50 & dateVal == dateVal_old & dateVal != 0) {
      GPSstatus = GPSstatus + 1;
    } else {
      if (timeVal != 0) {
        GPSstatus = 1;
      }
    }
    if (timeVal != 0) {
      timeVal_old = timeVal;
      dateVal_old = dateVal;
    }

    if ((curTime - GPSacquireA) >= 120) {            //Check to see if aquisition process has timed out.
      if (Debug) {
        serial.print(F("FAILED TO AQUIRE!!!!!!!!!")); //Failure message
      }
      GPSoff();                                     //Turn off the GPS
      GPSstatus = 0;                                //GPSstatus = 0 indicates no GPS activity
      calFreqInc = calFreqInc + 5;
      calFreq = min(calFreqInc, timeCalFreq);           //shorten calFreq to 5 minutes if there is a GPS failure
      GPSoff();                                     //Make sure GPS is off
      //         while (Serial2.available() > 0){
      //            char Byte0 = Serial2.read(); }             //read to dummy byte to clear buffer
      //         Serial2.end();                                //Stop Serial2 communication
      //if(SDOK==1) {saveLogSD("GPS acquisition failed ");}
      currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird in the US)
      currentTime = rtc.stringTime(); //Get the time
      SDsaveString = String("GPS acquisition failed ")  + currentDate + " " + currentTime;    // Create a data string
      if (SDOK == 1) {
        SDwriteString(SDsaveString, logFile); //Write log to SD card
      }
      char flashData[13] = {0, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA,              // Create an array representing an entire line of data
                            rtc.getMonth(), rtc.getDate(), rtc.getYear(), rtc.getHours(),
                            rtc.getMinutes(), rtc.getSeconds()
                           };
      writeFlashArray(fMemPageAddr, fMemByteAddr, flashData, 12);  //write array
      advanceFlashAddr2(12);
    }
  }

  if (GPSstatus >= 3) {
    String timeLogStr = "";
    GPSoff();
    GPSstatus = 0;
    calFreqInc = 0;
    calFreq = timeCalFreq;
    //      while (Serial2.available() > 0){
    //          char Byte0 = Serial2.read(); }    //read to dummy byte to clear buffer
    //      Serial2.end();
    if (Debug) {
      serial.println(F("GPS read obtained!!"));
      serial.println();
    }
    if (rtc.updateTime() == false) {
      if (Debug) {
        serial.print(F("RTC failed")); //Updates the time variables from RTC
      }
    }
    currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird in the US)
    currentTime = rtc.stringTime(); //Get the time
    currentDateTime = currentDate + " " + currentTime ;
    GPSoff();                             //Turn GPS off in case it is on
    GPSstatus = 0;                        //End GPS read attempt
    pastTime = 0;                         //This should ensure a GPS read on wake up

    //dateVal = 10000*da + 100*mo + yr ;               //Use date from RTC  - THIS DOES NOT WORK - time zone issue
    printGPStime(timeVal, dateVal);                 //Create a string for GPS time and set time byte variables (ss, mm, hh, da, mo, yr).
    extractUnixTime(makeUnixTime(yr, mo, da, hh, mm, ss) + timeOffset * 3600);
    //if (dateVal_clk == 10000*da + 100*mo + yr) {
    if (rtc.setTime(ss, mm, hh, da, mo, yr + 2000, 1) == false) {       //  attempt to set clock with input values
      if (Debug) {
        serial.println(F("Something went wrong setting the time"));
      }
    }
    if (Debug) {
      serial.print(F("Local time: "));
      serial.println(showTime());
      serial.println(F("Clock calibrated with GPS time."));
    }
    timeLogStr = "GPS calibration: Time updated from " + currentDateTime + " to ";
    //      } else {
    //          timeLogStr = "Bad date recieved from GPS. Time not updated.";
    //      }
    curTime = makeUnixTime(yr, mo, da, hh, mm, ss);                  // returns a unix time value based on the RTC values
    //if(SDOK==1) {saveLogSD(timeLogStr);}
    currentDate = rtc.stringDateUSA();                               // Get the current date in mm/dd/yyyy format (we're weird in the US)
    currentTime = rtc.stringTime(); //Get the time
    SDsaveString = timeLogStr  + currentDate + " " + currentTime;    // Create a data string
    if (SDOK == 1) {
      SDwriteString(SDsaveString, logFile); // Write log to SD card
    }
    pastTime = curTime;
    char flashData[13] = {0, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,           // Create an array representing an entire line of data
                          rtc.getMonth(), rtc.getDate(), rtc.getYear(),
                          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()
                         };
    writeFlashArray(fMemPageAddr, fMemByteAddr, flashData, 12);      // write array to note GPS update
    advanceFlashAddr2(12);
  }



  ///////Check Sleep//////////////Check Sleep///////////
  //Check to see if it is time to execute nightime sleep mode.

  rtc.updateTime();                                            // Get an update from the real time clock
  if (Debug) serial.println(showTime());                       // Show the current time
  int curTimeHHMM = rtc.getHours() * 100 + rtc.getMinutes();   // Combine hours and minutes into one variable
  if (curTimeHHMM == slpTime) {                                // Check to see if it is sleep time
    String SlpStr =  "Entering sleep mode at " + showTime();  // if it's time to sleep make a log message
    GPSoff();                                                 // Turn GPS off in case it is on
    GPSstatus = 0;                                            // End GPS read attempt
    pastTime = 0;                                             // This should ensure a GPS read on wake up
    if (Debug) serial.println(SlpStr);                        // print log message
    if (SDOK == 1) {
      SDwriteString(SlpStr, logFile); // save log message if SD writes are enabled
    }
    char flashData[13] = {0, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,    // Create an array representing an entire line of data
                          rtc.getMonth(), rtc.getDate(), rtc.getYear(),
                          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()
                         };
    writeFlashArray(fMemPageAddr, fMemByteAddr, flashData, 12);    // write array to flash memory
    advanceFlashAddr2(12);                                         // update flash memory
    sleepAlarm();                                                  // sleep using clock alarm for wakeup
    rtc.updateTime();                                              // get time from clock
    SlpStr =  "Wake up from sleep mode at " + showTime();          // log message
    if (SDOK == 1) {
      SDwriteString(SlpStr, logFile); // save log message if SD writes are enabled
    }
    char flashData2[13] = {0, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,        // Create an array representing an entire line of data
                           rtc.getMonth(), rtc.getDate(), rtc.getYear(),
                           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()
                          };
    writeFlashArray(fMemPageAddr, fMemByteAddr, flashData2, 12);   // write array to note wake-up from sleep
    advanceFlashAddr2(12);
  }

  //////Read Tags//////////////Read Tags//////////

  //Try to read tags - if a tag is read and it is not a recent repeat, write the data to the SD card and the backup memory.


  if (FastRead(RFcircuit, checkTime, pollTime1) == 1) {
    processTag(RFIDtagArray, RFIDstring, RFIDtagUser, &RFIDtagNumber);    // Parse tag data into string and hexidecimal formats
    rtc.updateTime();                                                     // Update time from clock
    if (feedMode == 'A' & doorState == 1) {           // Do the following in feed-All mode
      if (motorPresent) {
        doorOpen();                                   // Just open the door if the motor is active
        birdFed = 1;                                  // set birdFed varible to 1 to indicate that the bird was given food access.
      }
    }
    if (feedMode == 'T'  & doorState == 1) {          // Do the following in Target mode
      byte tagFound = 0;                              // Initialize tag found varible - indicates if tag read matches something in a list
      byte checkPage = 1;                             // first page of tag lists
      uint16_t andInt = 0xFFFF;                       // value used for bitwise and calculation
      for (byte c = 16; c > 0; c--) {                 // Start a for loop to look through all the tags lists indicated in tagGrpSel
        if ((tagGrpSel & andInt) == 0) {
          break; // Exit loop if no more 1s remain in tagGrpSel.
        }
        uint16_t testBit = (1 << (c - 1)) & tagGrpSel; // See if the current list is indicated in tagGrpSel
        if (testBit > 0) {
          tagFound = checkTag(RFIDtagNumber, checkPage); // If the list is indicated check the corresponding page for a match
        }
        if (tagFound) {
          if (Debug) serial.println("tag found!!!");  // If a match is found, print a message
          break;
        }                                     // If a match is found exit the loop
        checkPage++;                                  // advance to next page
        andInt = andInt >> 1;                         // update and mask variable -- shift down a bit
      }
      if (tagFound) {
        if (motorPresent) {
          doorOpen();                                 // if a match is found and the motor is working, open the door.
          birdFed = 1;                                // set birdFed varible to 1 to indicate that the bird was given food access.
        }
      }
    }
    uint32_t tagNo = RFIDtagNumber;                                                              // Set a temporary value for tag number
    //SDsaveString = String(RFIDstring) + ", " + RFcircuit + ", " + showTime();                  // Create a data string for SD write
    SDsaveString = String(RFIDstring) + " " + showTime() + " " + birdFed;                        // Create a data string for SD write
    if (Debug) serial.println(SDsaveString);                                                     // Print the data string to serial output
    currRFID = String(RFIDstring) + String(RFcircuit);                                           // Combine RFID and RFcircuit into a string to compare with future reads.
    uint32_t HHMMSS = rtc.getHours() * 10000 + rtc.getMinutes() * 100 + rtc.getSeconds();        // Creat a human readable time value
    if (currRFID != pastRFID  | (HHMMSS - pastHHMMSS >= delayTime)) {                            // See if the tag read is a recent repeat
      if (SDOK == 1) {
        SDwriteString(SDsaveString, dataFile); // If it's a new read, save to the SD card
      }
      if (feedMode == 'O') {
        birdFed = 1; // If the feed mode is Open, then birdFed needs to be set to 1
      }
      byte circuit_fed = (RFcircuit << 4) + birdFed;
      char flashData[13] = {circuit_fed, RFIDtagArray[0], RFIDtagArray[1], RFIDtagArray[2],      // Create an array representing an entire line of data
                            RFIDtagArray[3], RFIDtagArray[4], rtc.getMonth(), rtc.getDate(),
                            rtc.getYear(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()
                           };
      writeFlashArray(fMemPageAddr, fMemByteAddr, flashData, 12);  //write array
      advanceFlashAddr2(12);
      pastRFID = currRFID;                                                              // Update past RFID to check for repeats
      pastHHMMSS = HHMMSS;                                                              // Update pastHHMMSS to check for repeats
      if (Debug) {
        serial.println("Data logged.");      //Output message
        serial.println(SDsaveString);        //Output message
        serial.println();                    //Output message
      }
    } else {
      if (Debug) serial.println("Repeat - Data not logged");                            // Output message to indicate repeats (no data logged)
    }

    uint32_t tagNo2 = tagNo;                       // make a copy of current RFID number and antenna no.

    //Wait for tag to leave
    while (tagNo2 == tagNo) {                      // Continually check to see if tag number changes (or if a tag is not read)
      if (FastRead(RFcircuit, 100, 200) == 0) {    // Use slightly longer check and poll times used here...
        tagNo2 = 0xFFFFFFFF;                       // clear tagNo if no tag is read - this will exit the while loop when no tag is read
        if (Debug) serial.println(F("tag gone ")); // Output message
      } else {
        processTag(RFIDtagArray, RFIDstring, RFIDtagUser, &RFIDtagNumber);   // Tag read (probably same as before) - process to parse data
        tagNo = RFIDtagNumber;                                               // Reset current tag ID
        if (Debug) {
          serial.print(F("tag still present "));     //Output message
          serial.println(tagNo, HEX);                //Output message
        }
      }
      delay(100);
    }

    if (feedMode != 'O') {     // close the door if it needs closing
      if (motorPresent) {
        doorClose();           // doorClose function checks to make sure door is open first.
        birdFed = 0;           // set birdFed value back to zero
      }
    }
    blinkLED(LED_RFID, 2, 5);  // quick LED flash to indicate that everything worked.
  }

  //////////Pause//////////////////Pause//////////
  //After each read attempt execute a pause using either a simple delay or low power sleep mode.

  if (cycleCount < stopCycleCount | GPSstatus != 0) {   //Just to a simple pause between read attempts uneil cycle count reaches its threshold. Also do a simple pause if the GPS is engaged
    cycleCount++;
    //blipLED(30);
    if (GPSstatus == 0) {
      delay(pauseTime); //pause between polling attempts if GPS is not active (When GPS is active the pause happens in that code).
    }
  } else {
    sleepTimer(pauseCountDown, pauseRemainder);
  }

  //Alternate between circuits (comment out to stay on one cicuit).
  //RFcircuit == 1 ? RFcircuit = 2 : RFcircuit = 1; //if-else statement to alternate between RFID circuits

}





/////////////////////////////////////////////////////////////////////////////
/////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////
/////////////////////////////////////////////////////////////////////////////

//repeated LED blinking function
void blinkLED(uint8_t ledPin, uint8_t repeats, uint16_t duration) { //Flash an LED or toggle a pin
  pinMode(ledPin, OUTPUT);             // make pin an output
  for (int i = 0; i < repeats; i++) {  // loop to flash LED x number of times
    digitalWrite(ledPin, LOW);         // turn the LED on (LOW turns it on)
    delay(duration);                   // pause again
    digitalWrite(ledPin, HIGH);        // turn the LED off (HIGH turns it off)
    delay(duration);                   // pause for a while
  }                                    // end loop
}                                      // End function


//Recieve a byte (charaacter) of data from the user- this times out if nothing is entered
char getInputByte(uint32_t timeOut) {                 // Get a single character
  char readChar = '?';                                // Variable for reading in character
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  if (serial.available()) {                           //If there is a response then perform the corresponding operation
    readChar = serial.read();                         //read the entry from the user
  }
  return readChar;                                    //Return the value. It will be "?" if nothing is recieved.
}

//Recieve a string of data from the user- this times out if nothing is entered
String getInputString(uint32_t timeOut) {             // Get a string from the user
  String readStr = "";                                // Define an empty string to work with
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  if (serial.available()) {                           // If there is a response then read in the data
    delay(40);                                        // long delay to let all the data sink into the buffer
    readStr = serial.readString();                    // read the entry from the user
  }
  return readStr;                                     //Return the string. It will be "" if nothing is recieved.
}



//CLOCK FUNCTIONS///////////

//Get date and time strings and cobine into one string
String showTime() { //Get date and time strings and cobine into one string
  currentDateTime = String(rtc.stringDateUSA()) + " " + String(rtc.stringTime());
  return currentDateTime;
}

//Function to get user data for setting the clock
void inputTime() {                               // Function to set the clock
  serial.println("Enter mmddyyhhmmss");          // Ask for user input
  String timeStr = getInputString(20000);        // Get a string of data and supply time out value
  if (timeStr.length() == 12) {                  // If the input string is the right length, then process it
    //serial.println(timeStr);                   // Show the string as entered
    byte mo = timeStr.substring(0, 2).toInt();   //Convert two ascii characters into a single decimal number
    byte da = timeStr.substring(2, 4).toInt();   //Convert two ascii characters into a single decimal number
    byte yr = timeStr.substring(4, 6).toInt();   //Convert two ascii characters into a single decimal number
    byte hh = timeStr.substring(6, 8).toInt();   //Convert two ascii characters into a single decimal number
    byte mm = timeStr.substring(8, 10).toInt();  //Convert two ascii characters into a single decimal number
    byte ss = timeStr.substring(10, 12).toInt(); //Convert two ascii characters into a single decimal number
    if (rtc.setTime(ss, mm, hh, da, mo, yr + 2000, 1) == false) {     // attempt to set clock with input values
      serial.println("Something went wrong setting the time");        // error message
    }
  } else {
    serial.println("Time entry error");           // error message if string is the wrong lenth
  }
}

// sleep and wake up using the alarm  on the real time clock
void sleepAlarm() {                           // sleep and wake up using the alarm  on the real time clock
  rtc.setAlarm(0, wakM, wakH, 1, 1, 1, 19);  // set alarm: sec, min, hr, date, mo, wkday, yr (only min and hr matter)
  rtc.writeRegister(0x02, 0);                // write a zero to the flags register to clear all flags.
  rtc.enableDisableAlarm(B00000111);         // Enable daily alarm - responds to hour, minute, and second (set last three enable bits)
  rtc.enableAlarmINT(1);                     // Enable the clock interrupt output.
  lpSleep();                                 // sleep funciton - sleep until alarm interrupt
  blinkLED(LED_RFID, 5, 200);                // blink to indicate wakeup
  rtc.writeRegister(0x02, 0);                // clear clock flags to turn off alarm.
}

// Sleep and wake up using a 32-hertz timer on the real time clock
void sleepTimer(uint16_t pCount, byte pRemainder) { // Sleep and wake up using a 32-hertz timer on the real time clock
  rtc.writeRegister(0x02, 0);       // write a zero to the flags register to clear all flags.
  rtc.setTimer(pCount);             // set timer countdown (32-hertz timer)
  rtc.enableTimerINT(1);            // enable the clock interrupt output
  rtc.setCTRL1Register(B10011011);  // set control register to enable a 32 Hertz timer.
  lpSleep();                        // call sleep funciton (you lose USB communicaiton here)
  //blinkLED(LED_RFID, 1, 30);     // blink indicator - processor reawakened
  delay(pRemainder);                // additional delay for accuracy
  rtc.enableTimerINT(0);            // disable the clock interrupt output
  rtc.writeRegister(0x02, 0);       // write a zero to clear all interrupt flags.
}


void updateTimeVars() {
  yr = rtc.getYear();
  mo = rtc.getMonth();
  da = rtc.getDate();
  hh = rtc.getHours();
  mm = rtc.getMinutes();
  ss = rtc.getSeconds();
}

uint32_t makeUnixTime(byte yrU, byte moU, byte daU, byte hhU, byte mmU, byte ssU) {  // assemble time elements into a unix time value (stolen from time.h library)
  static const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // Number of days in each month
  uint16_t yrx = yrU ;                // get the year value into an integer variable
  yrx > 70 ? yrx = yrx + 1900 : yrx = yrx + 2000; // Set year to 1900s or 2000s
  uint32_t seconds = (yrx - 1970) * 86400 * 365;         // seconds since 1/1/1970
  for (int i = 1970; i < yrx; i++) {
    if (((i) > 0) && !((i) % 4) && ( (((i)) % 100) || !(((i)) % 400))) {
      seconds +=  86400; //add extra days for leap years
    }
  }
  // add days for this year, months start from 1
  for (int i = 1; i < moU; i++) {
    if ((i == 2) && (((yrx) > 0) && !((yrx) % 4) && (((yrx) % 100) || !((yrx) % 400)))) { //add an extra day on leap years
      seconds += 86400 * 29;
    } else {
      seconds += 86400 * monthDays[i - 1]; //monthDay array starts from 0
    }
  }
  seconds = seconds + ((daU - 1) * 86400) + (hhU * 3600) + (mmU * 60) + ssU;
  return seconds;
}



void extractUnixTime(uint32_t seconds) {
  // break the given time_t into time components
  // this is a more compact version of the C library localtime function
  // note that year is offset from 1970 !!!
  static const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // Number of days in each month

  //uint8_t yearX;
  uint8_t monthX, monthLength;
  uint32_t timeX;

  timeX = seconds;
  ss = timeX % 60;
  timeX /= 60; // now timeX is minutes
  mm = timeX % 60;
  timeX /= 60; // now timeX is hours
  hh = timeX % 24;
  timeX /= 24; // now timeX is days
  //tm.Wday = ((timeX + 4) % 7) + 1;  // Sunday is day 1 - don't use weekdays

  uint32_t yrx = 0;
  uint32_t days = 0;
  while ( (unsigned) (days += LEAP_YEAR(yrx) ? 366 : 365) <= timeX) {    //Sums up days from Jan 1 1970 to end of current year.
    yrx++;
  }
  yr = (yrx + 1970) % 100;  // calculate year

  days -= LEAP_YEAR(yrx) ? 366 : 365;  //subtract out the days of the last year

  timeX  -= days; // now it is days in this year, starting at 0

  days = 0;
  monthX = 0;
  monthLength = 0;
  for (monthX = 0; monthX < 12; monthX++) {
    if (monthX == 1) { // february
      if (LEAP_YEAR(yrx)) {
        monthLength = 29;
      } else {
        monthLength = 28;
      }
    } else {
      monthLength = monthDays[monthX];
    }

    if (timeX >= monthLength) {
      timeX -= monthLength;
    } else {
      break;
    }
  }
  mo = monthX + 1;  // jan is month 1
  da = timeX + 1;     // day of month
}

bool LEAP_YEAR(uint32_t Y) {
  bool ly = ( ((1970+(Y))>0) && !((1970+(Y))%4) && ( ((1970+(Y))%100) || !((1970+(Y))%400) ) );
  return ly;
}



////FLASH MEMORY FUNCTIONS////////////////////

//Enable the flash chip
void flashOn(void) {              // Enable the flash chip
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory set to output
  digitalWrite(SDselect, HIGH);   // make sure the SD card select is off
  digitalWrite(FlashCS, LOW);     // activate Flash memory
  SPI.begin();                    // Enable SPI communication for Flash Memory
  SPI.setClockDivider(SPI_CLOCK_DIV16); // slow down the SPI for noise and signal quality reasons.
}

//Disable the flash chip
void flashOff(void) {           // Disable the flash chip
  SPI.end();                    // turn off SPI
  digitalWrite(FlashCS, HIGH);  // deactivate Flash memory
}

// Read a single byte from flash memory
byte readFlashByte(unsigned int pageAddr, unsigned int byteAddr) {   // read a single byte from flash memory
  unsigned long rAddr = (pageAddr << 10) + byteAddr;
  flashOn();                                // activate flash chip
  SPI.transfer(0x03);                       // opcode for low freq read
  SPI.transfer((rAddr >> 16) & 0xFF);       // first of three address bytes
  SPI.transfer((rAddr >> 8) & 0xFF);        // second address byte
  SPI.transfer(rAddr & 0xFF);               // third address byte
  byte fByte = SPI.transfer(0);             // finally, read the byte
  flashOff();                               // deactivate flash chip
  return fByte;                             // return the byte that was read
}

void writeFlashByte(unsigned int pageAddr, unsigned int byteAddr, byte wByte) { // write a single byte from flash memory
  unsigned long wAddr = (pageAddr << 10) + byteAddr;
  flashOn();                                      // activate flash chip
  SPI.transfer(0x58);                             // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);             // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);              // second address byte
  SPI.transfer(wAddr & 0xFF);                     // third address byte
  SPI.transfer(wByte);                            // finally, write the byte
  flashOff();                                     // deactivate flash chip
  delay(20);                                      // delay to allow processing
}

//Write a data array to a specified memory address on the flash chip
void writeFlashArray(unsigned int pageAddr, unsigned int byteAddr, char *cArr, byte nchar) {
  //  serial.println("Writing Flash Array!!");
  //  serial.print("Flash Page: ");
  //  serial.print(pageAddr, DEC);
  //  serial.print("    Flash Byte: ");
  //  serial.println(byteAddr, DEC);
  unsigned long wAddr = (pageAddr << 10) + byteAddr;
  flashOn();                           // activate flash chip
  SPI.transfer(0x58);                  // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);  // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);   // second address byte
  SPI.transfer(wAddr & 0xFF);          // third address byte
  for (int n = 0; n < nchar; n++) {    // loop throught the bytes
    SPI.transfer(cArr[n]);             // finally, write each byte
  }
  flashOff();                          // Shut down
  delay(20);                           // This delay allows writing to happen - maybe it can be removed if the chip is not read immediately after
}

void readFlashArray(unsigned int pageAddr, unsigned int byteAddr, char *carr, byte nchar) {  // read a several bytes from flash memory
  unsigned long rAddr = (pageAddr << 10) + byteAddr;
  flashOn();                              // activate flash chip
  SPI.transfer(0x03);                     // opcode for low freq read
  SPI.transfer((rAddr >> 16) & 0xFF);     // first of three address bytes
  SPI.transfer((rAddr >> 8) & 0xFF);      // second address byte
  SPI.transfer(rAddr & 0xFF);             // third address byte
  for (int n = 0; n < nchar; n++) {       // loop throught the bytes
    carr[n] = SPI.transfer(0);          // finally, write the byte
  }
  flashOff();                             // deactivate flash chip
}

void readFlashByteArray(unsigned int pageAddr, unsigned int byteAddr, byte *bArr, byte nchar) {  // read a several bytes from flash memory
  unsigned long rAddr = (pageAddr << 10) + byteAddr;
  flashOn();                              // activate flash chip
  SPI.transfer(0x03);                     // opcode for low freq read
  SPI.transfer((rAddr >> 16) & 0xFF);     // first of three address bytes
  SPI.transfer((rAddr >> 8) & 0xFF);      // second address byte
  SPI.transfer(rAddr & 0xFF);             // third address byte
  for (int n = 0; n < nchar; n++) {       // loop throught the bytes
    bArr[n] = SPI.transfer(0);          // finally, write the byte
  }
  flashOff();                             // deactivate flash chip
}

char getMode() {
  char fMode = readFlashByte(0, 0x0A);        //feeder mode is stored in page 1, byte 10
  if ((fMode != 'O') && (fMode != 'T')) {
    fMode = 'A';
    writeFlashByte(0, 0x0A, 'A');
  }
  //serial.print(F("Feeder mode: "));
  //serial.println(fMode);
  return fMode;
}

char setMode(char sMode) {
  writeFlashByte(0, 0x0A, sMode);
  serial.print(F("Feeder mode set to: "));
  serial.println(sMode);
  delay(20);
  return sMode;
}

uint32_t advanceFlashAddr(uint32_t Addr, byte stepsize) {
  uint16_t bAddress = (Addr & 0x03FF) + stepsize;      //and with 00000011 11111111 and add 12 for new byte address
  if (bAddress > 500) {                           //stop writing if beyond byte address 500 (this is kind of wasteful)
    Addr = (Addr & 0xFFFFC00) + 0x0400;      //set byte address to zero and add 1 to the page address
    //serial.println("Page boundary!!!!!!!!!");
  } else {
    Addr = Addr + stepsize;                        //just add to the byte address
  }
  return Addr;
}

uint32_t advanceFlashAddr2(byte stepsize) {
  uint16_t byteAddr = fMemByteAddr + stepsize;
  uint16_t pageAddr = fMemPageAddr;
  if (byteAddr > 527) {                           //stop writing if beyond byte address 528
    if (pageAddr < 8192) {
      pageAddr = pageAddr + 1;      //set byte address to zero and add 1 to the page address
      //serial.println("Page boundary!!!!!!!!!");
    }
    byteAddr = 0;
  }
  fMemPageAddr = pageAddr;
  fMemByteAddr = byteAddr;
  uint32_t newFlashAddress = (pageAddr << 10) + byteAddr;
  return newFlashAddress;
}

bool checkTag(unsigned long tag, unsigned int lPage) { //Check if a tag matches one in a list stored in a particular Flash Memory page.
  unsigned long tag2 = 0;
  byte tagLoop = 1;
  byte tagCount = readFlashByte(lPage, 0);
  bool tFound = false;
  //  serial.print("looking to match ");
  //  serial.println(tag, HEX);
  //  serial.print("tag count ");
  //  serial.print(tagCount, DEC);
  //  serial.print(" tagLoop ");
  //  serial.println(tagLoop, DEC);
  unsigned long tagAddr = (lPage << 10) + 1;
  //  digitalWrite(csFlash, HIGH);  // initially deactivate flash chip
  //  digitalWrite(SDselect, HIGH); // deactivate SD card
  //  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  //  digitalWrite(csFlash, LOW);   //activate flash chip
  if(tagCount != 255) {
    flashOn();
    SPI.transfer(0x03);           //opcode for low freq read
    SPI.transfer(tagAddr >> 16);           //first of three address bytes
    SPI.transfer((tagAddr >> 8) & 0xFF);   // second address byte
    SPI.transfer(tagAddr & 0xFF);          // third address byte
    while (tagLoop <= tagCount) {
      tag2 = SPI.transfer(0);               // ignore the first (most significant) byte
      tag2 = SPI.transfer(0);               // pile the next four bytes into a long variable
      tag2 = (tag2 << 8) + SPI.transfer(0);
      tag2 = (tag2 << 8) + SPI.transfer(0);
      tag2 = (tag2 << 8) + SPI.transfer(0);
      //    serial.print("Tag retrieved ");
      //    serial.println(tag2, HEX);
      if (tag2 == tag) {
        tFound = true;
        //serial.println("match found...");
        flashOff();
        return (true);
      }
      tagLoop = tagLoop + 1;
    }
    flashOff();
  }
  //serial.println("match not found.");
  return (false); //No match found, return false
}


// Input the device ID and write it to flash memory
void inputID() {                                         // Function to input and set feeder ID
  serial.println("Enter four alphanumeric characters");  // Ask for user input
  String IDStr = getInputString(10000);                  // Get input from user (specify timeout)
  if (IDStr.length() == 4) {                             // if the string is the right length...
    deviceID[0] = IDStr.charAt(0);                       // Parse the bytes in the string into a char array
    deviceID[1] = IDStr.charAt(1);
    deviceID[2] = IDStr.charAt(2);
    deviceID[3] = IDStr.charAt(3);
    writeFlashArray(0, 4, deviceID, 4);                 // Write the array to flash
  } else {
    serial.println("Invalid ID entered");                // error message if the string is the wrong lenth
  }
}


//Display backup memory
void dumpMem(byte doAll) {                        // Display backup memory; specify last data address
  uint16_t curPage = dataStartPage;              // first address for stored data
  uint16_t curByte = 0;
  char BA[12];                                    // byte array for holding data
  static char dataLine[40];                       // make an array for writing data lines
  serial.println("Displaying backup memory.");    // Message
  serial.println();                               // three blank lines
  serial.println();
  serial.println();

  serial.print(deviceIDstr);
  serial.println("DATA.txt");
  while (curPage < 8193) {                 // Escape long memory dump by entering a character
    if (serial.available()) {                     // If a charcter is entered....
      serial.println("User exit");                // ...print a message...
      break;                                      // ...and exit the loop
    }
    readFlashArray(curPage, curByte, BA, 12);              // read a line of data (12 bytes)
    if (BA[0] == 255 & doAll == 0) {
      break;
    }
    byte fed01 = (BA[0] & 0x01);
    sprintf(dataLine, "%02X%02X%02X%02X%02X %02d/%02d/%02d %02d:%02d:%02d %01d",
            BA[1], BA[2], BA[3], BA[4], BA[5], BA[6], BA[7], BA[8], BA[9], BA[10], BA[11], fed01);  //skipping BA[0] which is the RF circuit number
    //    serial.print("page ");
    //    serial.print(curPage);
    //    serial.print("  byte ");
    //    serial.print(curByte);
    //    serial.print(" ");
    serial.println(dataLine);                      // print the string
    curByte = curByte + 12;
    if (curByte >= 528) {
      curByte = 0;
      curPage++;
    }
  }
}

void eraseBackup(char eMode) {  //erase chip and replace stored info
  //first erase pages 2 through 7
  serial.print("Erasing flash with mode ");
  serial.println(eMode);
  //uint16_t pageAddr = fAddr >> 10;    // store current page address for later
  if (eMode == 'a') {
    serial.println("This will take 80 seconds");
    flashOn();
    SPI.transfer(0xC7);  // opcode for chip erase: C7h, 94h, 80h, and 9Ah
    SPI.transfer(0x94);
    SPI.transfer(0x80);
    SPI.transfer(0x9A);
    digitalWrite(FlashCS, HIGH);    //Deassert cs for process to start
    delay(80000);
    flashOff();                         // Turn off SPI
    serial.println("DONE!");
    serial.println("You must now reestablish all parameters");
    return;
  }
  if (eMode == 's') {
    serial.println("Seek and destroy!!");
    uint16_t pageAddr = dataStartPage;
    while (pageAddr < 8192) {                   //Max number of pages = 8192
      byte rByte = readFlashByte(pageAddr, 0);  //check first byte of each page
      if (rByte != 255) {
        pageAddr++;                             //move to next page
      } else {
        if (pageAddr == dataStartPage) {            //no data detected on first page - no erasing needed
          break;
        } else {
          pageAddr--;                           //go back to last page with data to start erasing here
          break;
        }
      }
    }
  }
  if (eMode == 'f') {
    serial.println("Fast erase.");
    //pageAddr = (fAddr >> 10) & 0x1FFF;
  }
  //serial.print("erasing through page ");
  //serial.println(pageAddr, DEC);

  for (uint16_t p = dataStartPage; p <= fMemPageAddr; p++) {
    //serial.print("Erasing page ");
    //serial.println(p);
    uint32_t ePage = p << 10;
    flashOn();
    SPI.transfer(0x81);                    // opcode for page erase: 0x81
    SPI.transfer((ePage >> 16) & 0xFF);    // first of three address bytes
    SPI.transfer((ePage >> 8) & 0xFF);     // second address byte
    SPI.transfer(ePage & 0xFF);            // third address byte
    digitalWrite(FlashCS, HIGH);           // Deassert cs for process to start
    delay(35);                             // delay for page erase
    flashOff();
  }
  fMemPageAddr = dataStartPage;
  fMemByteAddr = 0;
}

void pageErase(uint16_t pageAddr) {
  //serial.print("Erasing page ");
  //serial.println(pageAddr);
  uint32_t ePage = pageAddr << 10;
  flashOn();
  SPI.transfer(0x81);                    // opcode for page erase: 0x81
  SPI.transfer((ePage >> 16) & 0xFF);    // first of three address bytes
  SPI.transfer((ePage >> 8) & 0xFF);     // second address byte
  SPI.transfer(ePage & 0xFF);            // third address byte
  digitalWrite(FlashCS, HIGH);           // Deassert cs for process to start
  delay(35);                             // delay for page erase
  flashOff();
}



void writeMem(byte doAll) {       //write backup data to SD card, specify end of data
  String BakupFileName = String(deviceID) + "BKUP.TXT";
  File dataFile;
  SDstart();
  serial.print("Writing flash to SD card to ");
  serial.println(BakupFileName);

  if (!SD.exists(BakupFileName)) {
    dataFile = SD.open(BakupFileName, FILE_WRITE);         // Create file if it is not there.
    if (dataFile) {
      dataFile.println(BakupFileName);                           // ...write the filename in the new file
      dataFile.close();
    }
  }
  delay(20);
  SDstop();

  byte mBytes[528];
  uint16_t curPAddr = dataStartPage;

  while (curPAddr < 8192) {                           //read in full pages one at a time.
    digitalWrite(LED_RFID, LOW);
    serial.print(" transfering page ");
    serial.println(curPAddr, DEC);
    uint32_t fAddress = curPAddr << 10;    //Make the page address a full address with byte address as zero.
    flashOn();
    SPI.transfer(0x03);                           // opcode for low freq read
    SPI.transfer((fAddress >> 16) & 0xFF);       // write most significant byte of Flash address
    SPI.transfer((fAddress >> 8) & 0xFF);         // second address byte
    SPI.transfer(fAddress & 0xFF);                // third address byte
    for (int n = 0; n < 527; n++) {             // loop to read in an RFID code from the flash and send it out via serial comm
      mBytes[n] = SPI.transfer(0);          //read in 528 bytes
      //serial.print(mBytes[n]);
    }
    digitalWrite(LED_RFID, HIGH);
    flashOff();
    if (mBytes[0] = 255 & doAll == 0) {     //No data in next page - stop transfer.
      break;
      curPAddr = 9000;  //probably unnecessary.
    }


    //Now write to SD card
    String wStr;
    SDstart();
    dataFile = SD.open(BakupFileName, FILE_WRITE);        //Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
    if (dataFile) {
      for (int i = 0; i < 527; i = i + 12) {         // loop to read in an RFID code from the flash and send it out via serial comm
        if (i != 255 | doAll != 0) {                // only write data if it exists
          static char charRFID[10];
          sprintf(charRFID, "%02X%02X%02X%02X%02X", mBytes[i + 1], mBytes[i + 2], mBytes[i + 3], mBytes[i + 4], mBytes[i + 5]); //skip i (rf circuit)
          static char dateAndTime[17];
          sprintf(dateAndTime, "%02d/%02d/%02d %02d:%02d:%02d", mBytes[i + 6], mBytes[i + 7], mBytes[i + 8], mBytes[i + 9], mBytes[i + 10], mBytes[i + 11]); //note order switched....+8 then +7
          wStr = String(charRFID) + " " + String(dateAndTime);
          dataFile.println(wStr);
          //serial.println(wStr);
        }
      }
      dataFile.close();      //close the file
    }
    SDstop();
    curPAddr++;
  }
  blinkLED(LED_RFID, 6, 70);
}

void showTags(uint16_t pageAddr) {      //display tags in flash memory (specify page).

  byte tagCount = readFlashByte(pageAddr, 0);         //provide page and byte addresses
  if (tagCount != 255) {
    serial.println();
    serial.print(tagCount, DEC);
    serial.print(" tags currently in list ");
    serial.println(pageAddr, DEC);

    char tagArray[527];
    readFlashArray(pageAddr, 1, tagArray, tagCount * 5);

    static char printRFID[10];
    for (int a = 0; a < tagCount * 5; a = a + 5) {
      sprintf(printRFID, "%02X%02X%02X%02X%02X ",
              tagArray[a], tagArray[a + 1], tagArray[a + 2], tagArray[a + 3], tagArray[a + 4]);
      serial.print(printRFID);
      if ((a + 5) % 20 == 0) {
        serial.println();
      }
    }

    //  uint32_t fAddress = (pageAddr<<10 + 1);
    //  flashOn();
    //  SPI.transfer(0x03);                           // opcode for low freq read
    //  SPI.transfer((fAddress >> 16) & 0xFF);        // write most significant byte of Flash address
    //  SPI.transfer((fAddress >> 8) & 0xFF);         // second address byte
    //  SPI.transfer(fAddress & 0xFF);                // third address byte
    //  for (int n = 0; n < tagCount; n++) {
    //    for (int p = 0; p < 5; p++) {
    //      byte RFbyte = SPI.transfer(0);
    //      if (RFbyte < 0x10) serial.print("0");
    //      serial.print(RFbyte, HEX);
    //    }
    //    if((n+1)%4 == 0) {
    //      serial.println();
    //    } else {
    //      serial.print(" ");
    //    }
    //  }
    //  flashOff();
  }
  serial.println();
}


byte FlashGetAddr(uint8_t spacing) {

  uint16_t pageAddr = dataStartPage;  // page address: backup memory starts on page dataStartPage
  uint16_t byteAddr = 0;          // byte address; start where first written byte might be
  byte rByte = 0;
  serial.print("Finding address ");
  //serial.println(pageAddr);
  while (pageAddr < 8192) { //Max number of pages = 8192
    serial.print(".");
    //serial.println(pageAddr, HEX);
    rByte = readFlashByte(pageAddr, 0);
    //serial.print("byte read: ");
    //serial.println(rByte);
    if (rByte != 255) {
      pageAddr++;       //move to next page
    } else {
      if (pageAddr == dataStartPage) {   //if you get a 255 first thing, then the memory is empty - start at beginning.
        fMemPageAddr = dataStartPage;
        fMemByteAddr = 0;
        return 2;
      } else {
        pageAddr--;       //go back to last page with data
        break;
      }
    }
    //serial.print("page found - ");
    //serial.println(pageAddr, HEX);
  }
  while (byteAddr < 527) {
    rByte = readFlashByte(pageAddr, byteAddr);
    if (rByte == 255) {
      break;
    }
    byteAddr = byteAddr + spacing;
  }
  if (byteAddr > 527) {
    pageAddr = pageAddr + 1; //set address to beginning of next page
    byteAddr = 0;
  }
  fMemPageAddr = pageAddr;
  fMemByteAddr = byteAddr;
  return 1;
}




////////////SD CARD FUNCTIONS////////////////////



String checkForLoadFile() {
  //serial.println("checking directory...");
  String fileName = "na";
  SDstart();
  File root = SD.open("/");
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      break; // no more files
    }
    String tSt = entry.name();
    if (tSt.charAt(4) == 'L' & tSt.charAt(5) == 'O' &
        tSt.charAt(6) == 'A' & tSt.charAt(7) == 'D') {
      //serial.println(tSt);
      fileName = tSt;
    }
    entry.close();
  }
  SDstop();
  return fileName;
}

void loadParameters(String pFile) {
  serial.println ("Loading parameters from file");
  SDstart();
  File myfile = SD.open(pFile);  // attempt to open the file with a list of tag numbers
  if (!myfile) {
    serial.print(pFile);
    serial.println(" file not found");
    return;
  }
  if (myfile) {                     // if the file is available, read from it one byte at a time
    serial.print(pFile);
    serial.println(" file found");
    byte pCnt = 1;
    char read1 = 'A';
    while (myfile.available()) {
      read1 = myfile.read();
      //serial.println(read1);
      if (read1 == 58) {
        if (pCnt == 1) {
          char devID[4];
          for (byte i = 0; i < 4; i++) {
            byte addr = 4 + i;
            devID[i] = myfile.read();
            //serial.print(devID[i]);
          }
          serial.println();
          writeFlashArray(0, 4, devID, 4);    // Write the array to flash
        }
        if (pCnt == 2) {
          read1 = myfile.read();
          setMode(read1);
        }
        if (pCnt == 3) {
          read1 = myfile.read();
          writeFlashByte(0, 0x0D, read1);
        }
        if (pCnt == 4) {
          read1 = myfile.read();
          byte read2 = myfile.read();
          byte read3 = myfile.read();
          uint16_t GPSFreq = (read1 - 48) * 100 + (read2 - 48) * 10 + (read3 - 48);
          setTimeCalFreq(GPSFreq);
          break;
        }
        pCnt++;
      }
    }
  }
}

//Startup routine for the SD card
bool SDstart() {                 // Startup routine for the SD card
  digitalWrite(SDselect, HIGH);  // Deactivate the SD card if necessary
  digitalWrite(FlashCS, HIGH);   // Deactivate flash chip if necessary
  pinMode(SDon, OUTPUT);         // Make sure the SD power pin is an output
  digitalWrite(SDon, LOW);       // Power to the SD card
  delay(20);
  digitalWrite(SDselect, LOW);   // SD card turned on
  if (!SD.begin(SDselect)) {     // Return a 1 if everyting works
    //serial.println("SD fail");
    return 0;
  } else {
    return 1;
  }
}

//Stop routine for the SD card
void SDstop() {                 // Stop routine for the SD card
  delay(20);                    // delay to prevent write interruption
  SD.end();                     // End SD communication
  digitalWrite(SDselect, HIGH); // SD card turned off
  digitalWrite(SDon, HIGH);     // power off the SD card
}

// Remove a file on the SD card
void SDremoveFile(String killFile) {   // Remove a file on the SD card
  //serial.println("killing file");    // Message
  SDstart();                           // Enable SD
  SD.remove(killFile);                 // Delete the file.
  SDstop();                            // Disable SD
}

// Write an entire string of data to a file on the SD card
bool SDwriteString(String writeStr, String writeFile) {  // Write an entire string
  SDstart();                                             // Enable SD
  bool success = 0;                                      // valriable to indicate success of operation
  File dFile;
  if (!SD.exists(writeFile)) {
    dFile = SD.open(writeFile, FILE_WRITE);         // Create file if it is not there.
    if (dFile) {
      dFile.println(writeFile);                           // ...write the filename in the new file
    }
  } else {
    dFile = SD.open(writeFile, FILE_WRITE);        // Open existing file.
  }
  if (dFile) {                                           // If the file is opened successfully...
    dFile.println(writeStr);                             // ...write the string...
    dFile.close();                                       // ...close the file...
    success = 1;                                         // ...note success of operation...
    //serial.println("SD Write OK");                     // success message
  } //else {
  //serial.println("SD Write fail");                   // fail message
  //}
  SDstop();                                              // Disable SD
  return success;                                        // Indicates success (1) or failure (0)
}

void transferTags(String fileName, uint16_t pageAddr) {
  //fileName = "AAAATAGS.TXT";
  char tagArray[528];   //Byte array for RFID codes

  //First clear memory pages.
  for (uint16_t ep = 1; ep < 18; ep++) {
    pageErase(ep);
  }
  SDstart();
  File myfile = SD.open(fileName);  // attempt to open the file with a list of tag numbers
  if (!myfile) {
    serial.print(fileName);
    serial.println(" file not found");
    return;
  }
  if (myfile) {                     // if the file is available, read from it one byte at a time
    serial.print(fileName);
    serial.println(" file found");
    while (myfile.available()) {
      char read1 = myfile.read();     //each myfile.read() operation brings in one byte (one character)
      //serial.print(read1);
      if (read1 == 79) {
        break; //Letter 'O' in STOP triggers end of process.
      }

      if (read1 == 71) {       //Letter 'G' in "TAG" indicates new group of IDs

        //First get the number for the Flash page address

        while ((read1 < 48) | (read1 > 57)) { //Skip until you get a numeral
          read1 = myfile.read();              //Get first numeral
          //serial.print(read1);
        }
        char read2 = myfile.read();                //Get second numeral

        //serial.println();
        //serial.print(read1);
        //serial.println(read2);

        byte tagPage = (read1 - 48) * 10 + (read2 - 48); // Page address for writing to Flash Mem
        //serial.print("group ");
        //serial.println(tagPage, DEC);


        //Next read all tag data for the group into the byte array (tagArray)

        byte byteCnt = 1;                            // counter for for writing to array - start at 1 (leave byte zero for tag count)
        //byte tagCount = 0;                            //

        while (read1 != 76) {
          read1 = myfile.read();
          if (read1 == 76) {
            break; //use the "L" in "LIST END" to end a group.
          }
          if ((read1 > 47 & read1 < 58) | (read1 > 64 & read1 < 71)) {
            read2 = myfile.read();
            //serial.println();
            //serial.print(read1);
            //serial.println(read2);
            tagArray[byteCnt] = (asciiToHex(read1) << 4) + asciiToHex(read2);
            //if(tagArray[byteCnt] < 0x10) {serial.print("0");}
            //serial.print(tagArray[byteCnt], HEX);
            //if(byteCnt % 5 == 0) {serial.print(" ");}
            //if(byteCnt % 20 == 0) {serial.println();}
            byteCnt++;
          }
        }

        //calculate number of tags based on byteCnt and store as byte 0 in array

        tagArray[0] = byteCnt / 5;

        //now write the tag Array to Flash

        writeFlashArray(tagPage, 0, tagArray, byteCnt + 1); //add 1 to byteCnt because it starts at 0.
        //serial.println();
      }
    }
  }
  return;
} // end function transferTags



void transferTimes(String fileName, uint16_t pageAddr) {

  //fileName = "XXXXTIME.TXT";

  //First erase data page
  pageErase(17);

  SDstart();
  File myfile = SD.open(fileName);  // attempt to open the file with a list of times and tag lists
  if (!myfile) {
    serial.print(fileName);
    serial.println(" file not found");
    return;
  }
  if (myfile) {                      // if the file is available, read the file
    serial.print(fileName);
    serial.println(" file found");
    uint16_t writeByte = 0;
    byte rCnt = 0;
    byte bCnt = 0;
    byte read1;
    char dtA[6];   //Array for data and time
    byte groupSelectShift = 16;
    uint16_t groupSelect = 0;
    while (myfile.available()) {
      read1 = myfile.read();
      //serial.print(read1);
      //serial.print("   ");
      //serial.println(read1);
      if (read1 == 83) {
        //serial.println("end of times file.");
        break;  //Break if "S" in stop is encountered.
      }
      if ((read1 > 47) & (read1 < 58)) {
        if ((rCnt < 6) & (groupSelectShift == 16)) {
          byte read2 = myfile.read();
          //          //serial.println(read2);
          dtA[rCnt] = (read1 - 48) * 10 + (read2 - 48);
          rCnt++;
          groupSelect = 0;
          groupSelectShift = 16;
        } else {
          if (groupSelectShift > 0) {
            groupSelect = groupSelect + ((read1 - 48) << (groupSelectShift - 1));
            groupSelectShift = groupSelectShift - 1;
          }
        }
        if ((rCnt >= 6) & (groupSelectShift == 0)) {
          uint32_t dtL = makeUnixTime(dtA[2], dtA[0], dtA[1], dtA[3], dtA[4], dtA[5]);
          dtA[0] = dtL >> 24;
          dtA[1] = (dtL & 0x00FF0000) >> 16;
          dtA[2] = (dtL & 0x0000FF00) >> 8;
          dtA[3] = dtL & 0x000000FF;
          dtA[4] = groupSelect >> 8;
          dtA[5] = groupSelect & 0x00FF;
          writeFlashArray(17, writeByte, dtA, 6);
          writeByte = writeByte + 6;
          rCnt = 0;
          groupSelect = 0;
          groupSelectShift = 16;
        }
      } else {
        //serial.println();
      }
    }
    myfile.close();
    SDstop();
  }
  delay(10);
  return;
} // end function transferTimes


void showReadTimes() {
  uint16_t bAddr2 = 0;
  uint32_t sTime = 0;
  uint16_t sGrp = 0;
  byte stA[6];
  while (sTime < 0xFFFFFFFF) {
    readFlashByteArray(17, bAddr2, stA, 6);   //Read in Unix time value and groupSelect
    sTime = (stA[0] << 24) + (stA[1] << 16) + (stA[2] << 8) + stA[3];
    if(sTime==0xFFFFFFFF) {break;}
    sGrp = (stA[4] << 8) + stA[5];
    extractUnixTime(sTime);
    char dateLine4[20];
    sprintf(dateLine4, "%02d/%02d/%02d %02d:%02d:%02d ",
        mo, da, yr, hh, mm, ss);
    serial.print(dateLine4);
    printBits(sGrp, 16, 1);
    serial.println();
    bAddr2 = bAddr2 + 6;
  }  
}



uint32_t readTimes(uint32_t timeNow, bool show) {
  uint16_t bAddr = 0;
  uint32_t time1;
  uint32_t time0 = 0;
  uint16_t gSel1 = 0;
  uint16_t gSel0 = 0;
  nextTime = 0xFFFFFFFF;             //Default max value in case no switches are scheduled.
  byte dtA[6];
  byte wBreak = 0;
  static char dataLine[40];
  while (time0 < timeNow) {
    readFlashByteArray(17, bAddr, dtA, 6);   //Read in Unix time value and groupSelect
    time0 = (dtA[0] << 24) + (dtA[1] << 16) + (dtA[2] << 8) + dtA[3];
    gSel0 = (dtA[4] << 8) + dtA[5];

    if (bAddr == 0) {
      tagGrpSel = (dtA[4] << 8) + dtA[5];            //Just a placeholder... in case the first time > the current time
    }
    
    readFlashByteArray(17, bAddr+6, dtA, 6);   //Read in Unix time value and groupSelect
    time1 = (dtA[0] << 24) + (dtA[1] << 16) + (dtA[2] << 8) + dtA[3];
    gSel1 = (dtA[4] << 8) + dtA[5];

    if ((timeNow >= time0) & (timeNow < time1)) {
      nextTime = time1;
      nextGrpSel = gSel1;
      tagGrpSel = gSel0;
      break;
    }
    bAddr = bAddr + 6;
  }
}

byte asciiToHex(byte x) { //Text conversion for transferTags
  if (x > 0x39) x -= 7; // adjust for hex letters upper or lower case
  x -= 48;
  return x;
}

void printBits(uint16_t n, byte numBits, byte show) {
  byte bitCnt = 0;
  for (byte i = 0; i < numBits; i++) {
    uint16_t shift = (1 << (numBits - 1 - i));
    //serial.print(shift, BIN)
    selBits[bitCnt] = (n & shift) > 0 ? '1' : '0'; // slightly faster to print chars than ints (saves conversion)
    if (i < (numBits - 1) && ((numBits - i - 1) % 4 == 0 )) {
      bitCnt++;
      selBits[bitCnt] = ' ';
    }
    bitCnt++;
  }
  selBits[20] = '\0';
  if (show & Debug) {
     serial.print(selBits);
  }
}


///////Sleep Function/////////////Sleep Function/////////

void lpSleep() {
  digitalWrite(MOTR, HIGH) ;                         // Must be set high to get low power working - don't know why
  shutDownRFID();                                    // Turn off both RFID circuits
  attachInterrupt(INT1, ISR, FALLING);               // Set up interrupt to detect high to low transition on interrupt pin
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |     // Configure EIC to use GCLK1 which uses XOSC32K
                      GCLK_CLKCTRL_GEN_GCLK1 |       // This has to be done after the first call to attachInterrupt()
                      GCLK_CLKCTRL_CLKEN;
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;        // disable USB
  SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;        // disable ms clock

  __WFI();    //Enter sleep mode
  //...Sleep...wait for interrupt
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;         // enable USB
  detachInterrupt(INT1);                             // turn interrupt off
  SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;         // Enable clock
  Debug = 0;
}

void ISR() {     // dummy routine - no interrupt activity needed
  byte SLEEP_FLAG = false;
}


///////GPS Functions/////////////GPS Functions/////////

void GPSon() {
  Serial2.begin(9600);                         //Start the additional serial port to listen to the GPS
  pinPeripheral(10, PIO_SERCOM);               // Assign pins 10 & 13 SERCOM functionality
  pinPeripheral(13, PIO_SERCOM);               // Must follow Serial2.begin() (not sure why)
  //serial.println("GPS turning ON !!!!!") ;
  digitalWrite(mStby, HIGH);
  digitalWrite(MOTPWMB, HIGH);
  //digitalWrite(gpsPwr2, LOW); //Necessary if connected to pin 2
  digitalWrite(gpsPwr1, HIGH);
}

void GPSoff() {
  //serial.println("GPS turning OFF !!!!!!") ;
  Serial2.end();
  digitalWrite(mStby, LOW);
  digitalWrite(MOTPWMB, LOW);
  //digitalWrite(gpsPwr2, LOW); //Necessary if connected to pin 2
  digitalWrite(gpsPwr1, LOW);
}

void parseGPS(uint32_t millisEnd) {
  String GPSstr = "";
  String GPSstr2 = "";
  dateVal = 0;
  timeVal = 0;
  //serial.println("GETTING GPS!!");
  char Byte0 = 'x';
  while (millis() < millisEnd) {
    while (Serial2.available() > 0 & millis() < millisEnd) {
      Byte0 = Serial2.read();
      GPSstr2 += Byte0;
    }                              //Read a byte
    if (Byte0 == '$') {                                       //See if the byte signifies the beginning of a sentence
      GPSstr = "";                                           //If so clear the string and look for the correct heading
      while (GPSstr.length() < 6 & millis() < millisEnd) {   //Read in 6 bytes and keep an eye on the timer
        if (Serial2.available() > 0) {
          Byte0 = Serial2.read();                          //Load each byte into a string character
          GPSstr += Byte0;                                 //Add bytes to the string
          GPSstr2 += Byte0;
        } else {
          delay(1);                                        //Wait if no bytes are available
        }
      }
      if (GPSstr == "GPRMC,") {
        GPSstr = "";              //If so clear the string again
        while (GPSstr.length() < 6 & millis() < millisEnd) {   //Read in 6 bytes and keep an eye on the timer
          if (Serial2.available() > 0) {                      //
            Byte0 = Serial2.read();                         //Load each byte into a string character
            GPSstr2 += Byte0;
            if (Byte0 < 48 | Byte0 > 57) {
              GPSstr = "xxxxxx";
              //serial.println("digit fail 1");
            } else {
              GPSstr += Byte0;    //Add bytes to the string
            }
          } else {
            delay(1);                                        //Wait if no bytes are available
          }
        }
        //CHECK TO MAKE SURE YOU HAVE 6 NUMERALS
        timeVal = GPSstr.toInt();                      //Convert text string to numeric value

        byte commaCnt = 0;                             //Variable for counting commas
        while (commaCnt < 8 & millis() < millisEnd) {  //Search for 7 commas and keep an eye on the timer
          if (Serial2.available() > 0) {              //Wait for data availability
            if (Serial2.read() == ',') {
              commaCnt++; //Add to counter if a comma is read
            }
          }
        }
        //serial.println(commaCnt);
        GPSstr = "";
        while (GPSstr.length() < 6 & millis() < millisEnd) {   //Read in 6 bytes and keep an eye on the timer
          if (Serial2.available() > 0) {                      //
            Byte0 = Serial2.read();
            GPSstr2 += Byte0;
            if (Byte0 < 48 | Byte0 > 57) {
              GPSstr = "xxxxxx";
              //serial.println("digit fail 2");
            } else {
              GPSstr += Byte0;                                //Add bytes to the string
            }
          } else {
            delay(1);                                        //Wait if no bytes are available
          }
        }
        //CHECK TO MAKE SURE YOU HAVE 6 NUMERALS
        dateVal = GPSstr.toInt();                              //Convert text string to numeric value
      }
    }
    Byte0 = 'x';
  }

  //   while(Serial2.available() > 0){   //Clear buffer.
  //        Byte0 = Serial2.read();
  //        GPSstr2 += Byte0;
  //   }
  //serial.println(GPSstr2);
}


void printGPStime(unsigned int tVal, unsigned int dVal) {
  ss = tVal % 100;
  mm = (tVal / 100) % 100;
  hh = tVal / 10000;
  da = dVal / 10000;
  mo = (dVal / 100) % 100;
  yr = dVal % 100;
  char GPSTimeChar[18];
  sprintf_P(GPSTimeChar, PSTR("%02d/%02d/%02d %02d:%02d:%02d"), mo, da, yr, hh, mm, ss);
  GPSTimeString = GPSTimeChar;
}

void setTimeCalFreq(uint16_t timeCalF) {
  //serial.print("Writing: ");
  //serial.println(timeCalFreq, HEX);
  writeFlashByte(0, 0x0B, timeCalF >> 8);
  writeFlashByte(0, 0x0C, timeCalF & 0x00FF);
}

///////Motor Functions/////////////Motor Functions/////////

void motInit() {  //determine initial motor position and move to closed position
  serial.println("Initializing motor...");
  byte motCount = 0;                      // count attempts to reach open position
  byte motEnd = 7;                        // limit to the number of attempts
  digitalWrite(mStby, HIGH);
  bool motDir = false;              //false for down, true for up
  unsigned int motTime = 100;
  while (digitalRead(mSwitch) & motCount < motEnd) {                //mSwitch high (switch open) means door is fully open or fully closed
    if (motDir == 0) {
      digitalWrite(MOTR, HIGH);   //Set motor to move door down
      digitalWrite(MOTL, LOW);
      serial.print("move down...");
      serial.println(motTime, DEC);
    } else  {
      digitalWrite(MOTR, LOW);   //Set motor to move door up
      digitalWrite(MOTL, HIGH);
      serial.print(F("move up..."));
      serial.println(motTime, DEC);
    }
    nudgeMot(motTime);           //Move motor a little
    motTime = motTime + 100;     //add to the run time
    motDir = !motDir;            //try the other direction
    motCount = motCount + 1;
    serial.println(motCount, DEC);
  }
  serial.println(motCount, DEC);
  if (motCount >= motEnd) {
    serial.println(F("motor failure!!!"));
    doorState = 1;
  } else {
    doorState = 0;
    if (motorPresent) {
      doorClose(); //if door is in the partially open position, then close it.
    }
  }
}

void doorOpen() {
  //serial.print("door state = "); //for debugging
  //serial.println(doorState); //for debugging
  if (doorState == 1) {
    //serial.println("open"); //for debugging
    digitalWrite(mStby, HIGH);
    digitalWrite(MOTR, HIGH);
    digitalWrite(MOTL, LOW);
    runMot();
    stopMot(true);
  }
  doorState = 0;
  //serial.print("door state = "); //for debugging
  //serial.println(doorState); //for debugging
}

void doorClose() {
  //serial.print("door state = "); //for debugging
  //serial.println(doorState); //for debugging
  //serial.println("close"); //for debugging
  if (doorState == 0) {
    digitalWrite(mStby, HIGH);
    digitalWrite(MOTR, LOW);  //powers up sleep....
    digitalWrite(MOTL, HIGH);
    runMot();
    stopMot(true);
  }
  doorState = 1;
  //serial.print("door state = "); //for debugging
  //serial.println(doorState); //for debugging
}

void stopMot(bool off) {
  digitalWrite(MOTPWMA, HIGH);
  digitalWrite(MOTR, HIGH);
  digitalWrite(MOTL, HIGH);
  delay(50);
  digitalWrite(MOTPWMA, LOW);
  digitalWrite(MOTR, LOW);
  digitalWrite(MOTL, LOW);
  if (off == true) {
    if (GPSstatus == 0) {
      digitalWrite(mStby, LOW);
    }
    //serial.print("door state stop = "); //for debugging
    //serial.println(doorState); //for debugging
  }
}

void pulseMot(byte pTime) {
  unsigned long currentMil = millis();          //Determine how long to activate the motor - first note the current value of the millisecond counter
  unsigned long stopMil = currentMil + pTime;   //   next add the desired movement time
  while (stopMil > millis()) {                  //   As long as the stoptime is less than the current millisecond counter, then keep the motor moving
    digitalWrite(MOTPWMA, HIGH);
    //analogWrite(MOTPWMA, motSpeed);
    //digitalWrite(MOTPWMA, HIGH);               //Speed is determined by the ratio of the high and low pulse
    //delay(3);                                   //This delay determines the duration of the high pulse
    //digitalWrite(MOTPWMA, LOW);                //Comment this out to go full speed
    delay(5);                                   //This delay determines the duration of the low pulse.
  }
}

void runMot() {
  pinMode(mSwitch, INPUT_PULLUP); // motor switch enabled as input with internal pullup resistor
  unsigned long moMil = millis();
  unsigned long moTim = moMil + 150;
  //three stage motor routine. First run the motor until the switch is closed
  byte sw = digitalRead(mSwitch);
  pulseMot(100);
  while (sw == 1) {             //mSwitch high (switch open) means door is fully open or fully closed
    sw = digitalRead(mSwitch);
    //serial.print(sw);
    pulseMot(5);
  }
  pulseMot(200);                   //pulses to get through any switch bounce
  //serial.println("stage 2");
  //Now run the motor until the switch opens again
  //serial.print("switch ");
  while (sw == 0) {         //mSwitch low (switch closed) means door is between open and closed
    sw = digitalRead(mSwitch);
    //serial.print(sw);
    pulseMot(5);
  }
  digitalWrite(MOTR, HIGH); //Apply brakes
  digitalWrite(MOTL, HIGH);
  delay(20);
  //serial.println();
  //Now nudge the motor just a little farther if desired
  //nudgeMot(100);
}

void nudgeMot(unsigned int motTime) {
  //serial.print("nudging...");
  //serial.println(motTime, DEC);
  unsigned long currentMs = millis();                //To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
  unsigned long stopMs = currentMs + motTime;   //next add the value of polltime to the current clock time to determine the desired stop time.
  while (stopMs > millis()) {
    pulseMot(10);
  }
  stopMot(false);
}
