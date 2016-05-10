
  Trail-Counter-Populate-Memory
  Project
  ----------------------------------
  Developed with embedXcode

  Project Trail-Counter-Populate-Memory
  Created by Charles McClelland on 5/10/16
  Copyright © 2016 Charles McClelland
  Licence GNU General Public Licence



  References
  ----------------------------------



  embedXcode
  embedXcode+
  ----------------------------------
  Embedded Computing on Xcode
  Copyright © Rei VILO, 2010-2016
  All rights reserved
  http://embedXcode.weebly.com

/*
This code is where is used to populate the meory map with data to support testing.
I am using the memory map v4.

License - BSD Release 3

Hardware setup:
- Simblee module on the i2c bus - Multi-Master!!!
- TI FRAM Chip on i2c Bus - Slave
- Some LEDs to indcate connectivity

Memory Map - 256kb or 32 Bytes divided into 4096 words - the  first one is reserved
Byte     Value
The first word is for system data
0        Memory Map Version (this program expects 2)
1        accelSensitivity
2        Delay
3        Daily Count Pointer
4-5      Current Hourly Count Pointer (16-bit number)
6        Reserved
7        Reserved
The second word is for storing the current count data
8-9      Current Hourly Count (16-bit)
10-11    Current Daily Count (16-bit)
12-15    EPOCH Time when last counts recorded (32-bits)
Words 3-16 are 14 days worth of daily counts - if this changes - need to change #offsets and DAILYCOUNTNUMBER
0        Month
1        Day
2 - 3    Daily Count (16-bit)
4        Daily Battery Level
5        Reserved
6        Reserved
7        Reserved
Words from 17 to the end of the memory store hourly data
0 - 3    EPOCH Time
4 - 5    Hourly count (16-bit)
6        Houlry Battery Level
7        Reserved

The part is open for an average of 12 hours per day so about 340 days of hourly data on a 256k chip
*/
