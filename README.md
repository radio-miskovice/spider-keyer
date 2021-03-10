# Spider Keyer

Spider Keyer is Arduino-based CW keyer developed by Petr, OK1FIG. Its main purpose is to be a "companion equipment" to be used with HamRacer contesting software.

The main inspiration for Spider Keyer was K3NG keyer software. Petr's main motivation to remake the original code base was (according to author):

- Simplify the sketch. K3NG's implementation amounts to 20K+ lines of code, and sometimes it is difficult to find the function that you need to modify or simply just understand.
  Petr's code is approximately 1000 lines long.
- Leave more flash space for further improvements. K3NG "minimalistic configuration" (keying, memories, serial link protocol, command mode) occupies up to 26 KB of flash memory, 
  which is very close to the limit of available program flash. ATmega 328 has 32 KB flash memory, of which approximately 2KB are occupied by Arduino bootloader. In case of 
  Chinese "compatible chip" LGT8F328P the situation is even worse - 2KB is occupied by bootloader and 2KB are used to emulate 1KB EEPROM (unlike ATMega, LGT has 
  no separate, dedicated EEPROM memory). Compiled Spider Keyer is providing approximately the same functions as K3NG, but the program size is only about 12 KB.
- Keep only the essential functions: keying, speed control, efficient and helpful PTT control. This is one of the key factors that allowed Petr to reduce program size.
- Spider keyer uses its own serial link protocol, much more simple than Winkeyer protocol version 2 emulation available in K3NG code. Sadly, Winkeyer protocol is an 
  outdated concept with lots of unpleasant limitations and quirks. Spider keyer protocol looks more contemporary, provides essential, necessary functions and allows for 
  efficient use of all capabilities provided by Arduino hardware and the keyer features.


- Spider keyer main page: http://ok1fig.nagano.cz/SpiderKeyer/SpiderKeyer.htm
- Spider keyer home made constructions: http://ok1fig.nagano.cz/SpiderKeyer/SpiderKeyerHm.htm
- Spider keyer protocol specification: http://ok1fig.nagano.cz/SpiderKeyer/SpiderKeyerSpecs.htm
- HamRacer logging software description: http://ok1fig.nagano.cz/HamRacer/HamRacer.htm

Sketch is placed on Github by Jindra, OK4RM. Version may be not the newest one, depending on what OK1FIG will be willing to share ;-)
