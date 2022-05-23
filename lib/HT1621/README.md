# HT1621 7 segment LCD Arduino library
Arduino library for 7-segment lcds based on the HT1621 driver. Very often marked as PDC-6X1.

<img src="extras/photo.jpg" alt="photo" width="500">

Based on the bitbanging efforts by [anxzhu](https://github.com/anxzhu) (2016-2018).  
APIs rewritten in 2018 to follow the LiquidCrystal format by [valerio\new](https://github.com/valerionew).

## APIs reference

* `void begin(int cs_p, int wr_p, int data_p,[int backlight_p])`  
Starts the lcd with the pin assignement declared. The backlight pin is optional

* `void clear()`  
Clears the display

* `void backlight()`  
Turns on the backlight 

* `void noBacklight()`  
Turns off the backlight

* `void setBatteryLevel(int level)`  
Accepts values from 0 to 3. Smaller values will be treated like 0, bigger values will be treated as 3. 0 turns off the battery symbol. Values from 1 to 3 will be represented by the rectangles above the battery symbol. 

* `void print(long num)`  
Prints a signed integer between -99999 and 999999. Larger and smaller values will be displayed as -99999 and 999999

* `void printCelsius(float num)`  
Prints a signed temperature between -999 and 9999 and appends the degree symbol. Larger and smaller values will be displayed as -999 and 9999. The precision is limited to 1 digit due to the fact that there is no dot on the left part of the display (it's used for the battery). 

* `void print(float num)`  
Prints a float with 3 decimals. 

* `void print(float num, int precision)`  
Prints a float with 0 to 3 decimals, based on the `precision` parameter. 

* `void print(char* str, bool leftPadded = false)`
Prints a string. If `leftPadded` is true, the string will be left padded with spaces to fit the display. Not all characters are supported, only the ones that can be displayed on the display. Higher and lower case characters are ignored and the one that is the most clear between the two is used. Currenly not supporting: 'k' (unprintable???), 'v', 'w'. 'm' is printed in a funny way but looks like it's common notation. Symbols available: 	'°' (must use '*' but prints °), '|', '-', '_'. 

* `void noDisplay()`  
Turns off the display (doesn't turn off the backlight) 

* `void display()`  
Turns the display back on after it has been disabled by `noDisplay()`


## Internal functioning

° -> 0x33 = 10 + 20 + 01 + 02  
C -> 0x1D = 10 + 01 + 04 + 08

```
  _0b1 0000_
 |          |
 0b1        0b10 0000
 |___0b10___|
 |          |
 0b0100     0b0100 0000
 |__00b1000_|

```

