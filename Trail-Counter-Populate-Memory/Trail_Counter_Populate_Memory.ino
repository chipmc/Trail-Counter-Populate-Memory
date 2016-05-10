///
/// @mainpage	Trail-Counter-Populate-Memory
///
/// @details	Populates the FRAM for testing
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/10/16 9:36 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Trail_Counter_Populate_Memory.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/10/16 9:36 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
    #include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
    #include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
    #include "libpandora_types.h"
    #include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
    #include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
    #include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
    #include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
    #include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
    #include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
    #include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
    #include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
    #include "application.h"
#elif defined(ESP8266) // ESP8266 specific
    #include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
    #include "Arduino.h"
#else // error
    #   error Platform not defined
#endif // end IDE

// Set parameters
//These defines let me change the memory map without hunting through the whole program
#define VERSIONNUMBER 4       // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 16        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 14    // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4078 // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0       // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1   // For the 1st Word locations
#define DEBOUNCEADDR 0x2
#define DAILYPOINTERADDR 0x3
#define HOURLYPOINTERADDR 0x4
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8
#define CURRENTDAILYCOUNTADDR 0xA
#define CURRENTCOUNTSTIME 0xC
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1         //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2        // Count is a 16-bt value
#define DAILYBATTOFFSET 4
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6



// Include application, user and local libraries
#include <Wire.h>
#include "Adafruit_FRAM_I2C.h"
#include "RTClib.h"             // Adafruit's library which includes the DS3231

// Define structures and classes


// Define variables and constants
// Pin Value Variables
int SCLpin = 13;
int SDApin = 14;
int LedPin = 8;

// FRAM and Unix time variables
unsigned int  framAddr;
unsigned long unixTime;
DateTime currentTime;
DateTime t;
int lastHour = 0;  // For recording the startup values
int lastDate = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts
char *dateTimePointer;
char dateTimeArray[19] = "Test Array";


// Map Values
int numberDays = 10;
int numberHoursPerDay = 10;
int seedDayValue = 1000;
int seedHourValue = 100;


// Prototypes
// Prototypes From the included libraries
RTC_DS3231 rtc;                               // Init the DS3231
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM

// Prototypes From my functions
// Prototypes for FRAM Functions
unsigned long FRAMread32(unsigned long address); // Reads a 32 bit word
void FRAMwrite32(int address, unsigned long value);  // Writes a 32-bit word
int FRAMread16(unsigned int address); // Reads a 16 bit word
void FRAMwrite16(unsigned int address, int value);  // Writes a 16-bit word
uint8_t FRAMread8(unsigned int address);  // Reads a 8 bit word
void FRAMwrite8(unsigned int address, uint8_t value); //Writes a 32-bit word
void ResetFRAM();  // This will reset the FRAM - set the version and preserve delay and sensitivity
void MemoryMapReport(); // Creates a memory map report on start



// Prototypes for General Functions
void BlinkForever(); // Ends execution
boolean BuildSystemDataWord();   // Populate System Data Memory
boolean BuildCurrentDataWord();  // Populate Current Data Memory
boolean BuildDailyWord(int dayWord,unsigned long unixT, int dayValue); // Builds the Daily section
boolean BuildHourlyWord(int hourWord,unsigned long unixT, int hourValue);    // Builds the Hourly Section
unsigned long findMidnight(unsigned long unixT); // Need to break at midnight
void toArduinoDateTime(unsigned long unixT, int xAxis, int yAxis);   // From Unix to format for reporting

void setup()
{
    Serial.begin(9600);
    Wire.beginOnPins(SCLpin,SDApin);
    pinMode(LedPin, OUTPUT);// led turned on/off from the iPhone app
    
    if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
        Serial.println("Found I2C FRAM");
    }
    else {
        Serial.println("No I2C FRAM found ... check your connections\r\n");
        BlinkForever();
    }
    
    // Check to see if the memory map in the sketch matches the data on the chip
    if (fram.read8(VERSIONADDR) != VERSIONNUMBER) {
        Serial.print(F("FRAM Version Number: "));
        Serial.println(fram.read8(VERSIONADDR));
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
        while (!Serial.available());
        switch (Serial.read()) {    // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'N':
                Serial.println(F("Cannot proceed"));
                BlinkForever();
                break;
            default:
                BlinkForever();
        }
    }
    
    if (!BuildSystemDataWord()) {
        Serial.println("Error Buidling the System Data Word");
        BlinkForever();
    }
    else Serial.println("System Data Word Mapped to Memory Successfully!");
    if (!BuildCurrentDataWord()) {
        Serial.println("Error Building the Current Data Word");
        BlinkForever();
    }
    else Serial.println("Current Data Word Mapped to Memory Successfully!");
    unsigned long lastDateTime = FRAMread32(CURRENTCOUNTSTIME);
    unsigned long lastMidnight = findMidnight(lastDateTime);
    
    for (int i=0; i<= numberDays-1; i++ )
    {
        BuildDailyWord(i,lastMidnight - 86400L*(numberDays-i),seedDayValue);
        for (int n=0; n<=numberHoursPerDay-1; n++)
        {
            BuildHourlyWord(i*numberHoursPerDay+n,lastMidnight - 86400L*(numberDays-i) + 3600*(8+n),seedHourValue);
        }
    }
    
}

void loop() {
    MemoryMapReport();
    BlinkForever();
}


boolean BuildSystemDataWord()   // Populate System Data Memory
{
    fram.write8(VERSIONADDR,VERSIONNUMBER);
    fram.write8(SENSITIVITYADDR,100);
    fram.write8(DEBOUNCEADDR,200);
    fram.write8(DAILYPOINTERADDR,numberDays);
    FRAMwrite16(HOURLYPOINTERADDR,numberHoursPerDay*numberDays);
    
    if (fram.read8(VERSIONADDR) != VERSIONNUMBER) {
        Serial.println("Error writing VERSIONADDR");
        return 0;
    }
    if (fram.read8(SENSITIVITYADDR) != 100) {
        Serial.println("Error writing SENSITIVITYADDR");
        return 0;
    }
    if (fram.read8(DEBOUNCEADDR) != 200) {
        Serial.println("Error writing DEBOUNCEADDR");
        return 0;
    }
    if (fram.read8(DAILYPOINTERADDR) != numberDays) {
        Serial.println("Error writing DAILYPOINTERADDR");
        return 0;
    }
    if (FRAMread16(HOURLYPOINTERADDR) != numberHoursPerDay*numberDays) {
        Serial.println("Error writing HOURLYPOINTERADDR");
        return 0;
    }
    return 1;
}

boolean BuildCurrentDataWord()  // Populate Current Data Memory
{
    FRAMwrite16(CURRENTHOURLYCOUNTADDR,15);
    FRAMwrite16(CURRENTDAILYCOUNTADDR,150);
    FRAMwrite32(CURRENTCOUNTSTIME,1456892598);  // March 1st 2016 - 20:22 PST
    
    if (FRAMread16(CURRENTHOURLYCOUNTADDR) != 15) return 0;
    if (FRAMread16(CURRENTDAILYCOUNTADDR) != 150) return 0;
    if (FRAMread32(CURRENTCOUNTSTIME) != 1456892598) return 0;
    return 1;
}

boolean BuildDailyWord(int dayWord,unsigned long unixT, int dayValue) // Builds the Daily section
{
    DateTime timeElement(unixT);
    fram.write8((DAILYOFFSET+dayWord)*WORDSIZE,timeElement.month());
    fram.write8((DAILYOFFSET+dayWord)*WORDSIZE+DAILYDATEOFFSET,timeElement.day());
    FRAMwrite16((DAILYOFFSET+dayWord)*WORDSIZE+DAILYCOUNTOFFSET,dayValue*(dayWord+1));
    fram.write8((DAILYOFFSET+dayWord)*WORDSIZE+DAILYBATTOFFSET,random(100));
    return 1;
}

boolean BuildHourlyWord(int hourWord,unsigned long unixT, int hourValue)    // Builds the Hourly Section
{
    FRAMwrite32((HOURLYOFFSET+hourWord)*WORDSIZE,unixT);
    FRAMwrite16((HOURLYOFFSET+hourWord)*WORDSIZE+HOURLYCOUNTOFFSET,hourValue+random(-10,10));
    fram.write8((HOURLYOFFSET+hourWord)*WORDSIZE+HOURLYBATTOFFSET,random(100));
    return 1;
}

void MemoryMapReport()  // Report on memory map
{
    Serial.println(" ");
    Serial.println("The first word - System Settings");
    Serial.print("  Version number: "); Serial.println(fram.read8(VERSIONADDR));
    Serial.print("  Sensitivity: "); Serial.println(fram.read8(SENSITIVITYADDR));
    Serial.print("  Debounce: "); Serial.println(fram.read8(DEBOUNCEADDR));
    Serial.print("  Daily Pointer: "); Serial.println(fram.read8(DAILYPOINTERADDR));
    Serial.print("  Hourly Pointer: "); Serial.println(FRAMread16(HOURLYPOINTERADDR));
    Serial.println(" ");
    Serial.println("The second word - Current Settings");
    Serial.print("  Current Hourly Count: "); Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
    Serial.print("  Current Daily Count: "); Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
    Serial.print("  Current Count Time: "); Serial.println(FRAMread32(CURRENTCOUNTSTIME));
    Serial.print("  Current Time Converted: "); toArduinoDateTime(FRAMread32(CURRENTCOUNTSTIME), 0, 0);
    Serial.println(" ");
    Serial.print("  Current Time Midnight:  "); toArduinoDateTime(findMidnight(FRAMread32(CURRENTCOUNTSTIME)), 0, 0);
    Serial.println(" ");
    Serial.println("The daily count words");
    for (int i=0; i<= numberDays-1; i++ )
    {
        Serial.print("  Day: "); Serial.print(i+1); Serial.print("  \t");
        Serial.print(fram.read8((DAILYOFFSET+i)*WORDSIZE)); Serial.print("/");
        Serial.print(fram.read8((DAILYOFFSET+i)*WORDSIZE+DAILYDATEOFFSET)); Serial.print("\t");
        Serial.print("Count: "); Serial.print(FRAMread16((DAILYOFFSET+i)*WORDSIZE+DAILYCOUNTOFFSET)); Serial.print("\t");
        Serial.print("Charge: "); Serial.print(fram.read8((DAILYOFFSET+i)*WORDSIZE+DAILYBATTOFFSET)); Serial.println("%");
    }
    for (int i=0; i<= numberHoursPerDay*numberDays-1; i++ )
    {
        Serial.print("  TimeStamp: "); Serial.print(i+1); Serial.print("  \t");
        toArduinoDateTime(FRAMread32((HOURLYOFFSET+i)*WORDSIZE),0,0); Serial.print("  \t");
        Serial.print("Count: "); Serial.print(FRAMread16((HOURLYOFFSET+i)*WORDSIZE+HOURLYCOUNTOFFSET)); Serial.print("\t");
        Serial.print("Charge: "); Serial.print(fram.read8((HOURLYOFFSET+i)*WORDSIZE+HOURLYBATTOFFSET)); Serial.println("%");
    }
}

unsigned long findMidnight(unsigned long unixT) // Need to break at midnight
{
    DateTime timeElement(unixT);
    unsigned long workingNumber;
    workingNumber = unixT - 3600*timeElement.hour() -60*timeElement.minute() - timeElement.second();
    return workingNumber;
}

void toArduinoDateTime(unsigned long unixT, int xAxis, int yAxis)   // From Unix to format for reporting
{
    int columnWidth = 6;
    dateTimePointer = dateTimeArray;
    DateTime timeElement(unixT);
    
    if(timeElement.month() < 10) {
        dateTimeArray[0] = '0';
        dateTimeArray[1] = timeElement.month()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[0] = int(timeElement.month()/10)+48;
        dateTimeArray[1] = (timeElement.month()%10)+48;
    }
    dateTimeArray[2] =  '/';
    if(timeElement.day() < 10) {
        dateTimeArray[3] = '0';
        dateTimeArray[4] = timeElement.day()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[3] = int(timeElement.day()/10)+48;
        dateTimeArray[4] = (timeElement.day()%10)+48;
    }
    dateTimeArray[5] =  '/';
    int currentYear = timeElement.year() - 2000;
    if(currentYear < 10) {
        dateTimeArray[6] = '0';
        dateTimeArray[7] = currentYear+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[6] = int(currentYear/10)+48;
        dateTimeArray[7] = (currentYear%10)+48;
    }
    dateTimeArray[8] =  ' ';
    if(timeElement.hour() < 10) {
        dateTimeArray[9] = '0';
        dateTimeArray[10] = timeElement.hour()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[9] = int(timeElement.hour()/10)+48;
        dateTimeArray[10] = (timeElement.hour()%10)+48;
    }
    dateTimeArray[11] =  ':';
    if(timeElement.minute() < 10) {
        dateTimeArray[12] = '0';
        dateTimeArray[13] = timeElement.minute()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[12] = int(timeElement.minute()/10)+48;
        dateTimeArray[13] = (timeElement.minute()%10)+48;
    }
    dateTimeArray[14] =  ':';
    if(timeElement.second() < 10) {
        dateTimeArray[15] = '0';
        dateTimeArray[16] = timeElement.second()+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[15] = int(timeElement.second()/10)+48;
        dateTimeArray[16] = (timeElement.second()%10)+48;
    }
    Serial.print(dateTimePointer);
    //SimbleeForMobile.updateText(dateTimeField, dateTimePointer);
}



//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void FRAMwrite32(int address, unsigned long value)  // Writes a 32-bit word
{
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);
    
    //Write the 4 bytes into the eeprom memory.
    fram.write8(address, four);
    fram.write8(address + 1, three);
    fram.write8(address + 2, two);
    fram.write8(address + 3, one);
}

unsigned long FRAMread32(unsigned long address)
{
    //Read the 4 bytes from the eeprom memory.
    long four = fram.read8(address);
    long three = fram.read8(address + 1);
    long two = fram.read8(address + 2);
    long one = fram.read8(address + 3);
    
    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

//This function will write a 2 byte (16bit) long to the eeprom at
//the specified address to address + 1.
void FRAMwrite16(unsigned int address, int value)    // Writes a 16-bit word
{
    //Decomposition from a long to 2 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte two = (value & 0xFF);
    byte one = ((value >> 8) & 0xFF);
    
    //Write the 2 bytes into the eeprom memory.
    fram.write8(address, two);
    fram.write8(address + 1, one);
}

int FRAMread16(unsigned int address)
{
    //Read the 2 bytes from the eeprom memory.
    long two = fram.read8(address);
    long one = fram.read8(address + 1);
    
    //Return the recomposed long by using bitshift.
    return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}

// Note - have to hard code the size here due to this issue - http://www.microchip.com/forums/m501193.aspx
void ResetFRAM()  // This will reset the FRAM - set the version and preserve delay and sensitivity
{
    Serial.println("Resetting Memory");
    for (unsigned long i=3; i < 32768; i++) {  // Start at 3 to not overwrite debounce and sensitivity
        fram.write8(i,0x0);
        //Serial.println(i);
        if (i==8192) Serial.println(F("25% done"));
        if (i==16384) Serial.println(F("50% done"));
        if (i==(24576)) Serial.println(F("75% done"));
        if (i==32767) Serial.println(F("Done"));
    }
    fram.write8(VERSIONADDR,VERSIONNUMBER);  // Reset version to match #define value for sketch
}

void BlinkForever()     // For when things go very wrong
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWrite(LedPin,HIGH);
        delay(200);
        digitalWrite(LedPin,LOW);
        delay(200);
    }
}