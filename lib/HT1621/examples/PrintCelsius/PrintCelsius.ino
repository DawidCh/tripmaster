#include <HT1621.h>
#define LCD_CS_PIN 6
#define LCD_WR_PIN 7
#define LCD_DATA_PIN 8
#define DELAY_SINK 6000

// #define HT1621_DEBUG

HT1621 lcd;
void setup() {
  lcd.begin(LCD_CS_PIN, LCD_WR_PIN, LCD_DATA_PIN);
  lcd.backlight();
  lcd.clear();
}

void loop(){
  lcd.clear();
  lcd.printCelsius(0.1);
  delay(DELAY_SINK);
  lcd.clear();
  lcd.printCelsius(10);
  delay(DELAY_SINK);
  lcd.clear();
  lcd.printCelsius(11111.1); // saturates at 9999
  delay(DELAY_SINK);
  lcd.clear();
  lcd.printCelsius(-1111.1); // saturates at -999
  delay(DELAY_SINK);
  lcd.clear();
  lcd.printCelsius(-0.1);
  delay(DELAY_SINK);
  lcd.clear();
  lcd.printCelsius(-12.57);
  delay(DELAY_SINK);
}